#!/usr/bin/env python3
"""
PID Tuner — контроллер коллекторного двигателя (Blue Pill F103).
- Графики оборотов и тока ВО ВЕСЬ ОСТАТОК окна (главный приоритет)
- Вкладки снизу фиксированной высоты: ПИД / Разгон / Разомкнутый / Фильтры /
  Зоны / Железо / Лог-Тест / Задатчик
- Крутилка виртуального задатчика + показ реального потенциометра
- Очередь команд (txq), отложенные ползунки, CSV-лог, тест скачка
- Одна компактная строка состояния

Формат прошивки (JSON): {"cur":...,"rpm":...,"duty":...,"ol":...,"olduty":...}
"""
import struct, serial, serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, messagebox
import threading, re, json, time, queue, csv, os, datetime, math

BAUD = 115200
RPM_MIN = 30
RPM_MAX = 6000
PWM_MAX = 900
DEFAULT_SS = 6
DEFAULT_RU = 10
DEFAULT_RD = 30
STEP_WINDOW = 5.0     # окно замера перерегулирования, с

TRIP_NAMES = {1:'перегрузка', 2:'пик тока', 3:'тепловая',
              4:'энкодер', 5:'разгон', 6:'стоянка в тяге',
              7:'датчик тока', 8:'обрыв якоря', 9:'нет связи',
              10:'сброс по watchdog', 11:'аппаратный срыв (BKIN)'}


class Knob(tk.Canvas):
    """Круглая ручка. Тянешь мышью по кругу или крутишь колесом — значение.
       -135°(минимум) .. +135°(максимум), разрыв дуги снизу."""

    def __init__(self, master, vmin, vmax, value, on_change, size=150, **kw):
        super().__init__(master, width=size, height=size,
                         bg='#101418', highlightthickness=0, **kw)
        self.vmin, self.vmax = vmin, vmax
        self.value = float(value)
        self.on_change = on_change
        self.size = size
        self.r = size / 2 - 14
        self.cx = self.cy = size / 2
        self._dragging = False
        self.bind('<Button-1>', self._press)
        self.bind('<B1-Motion>', self._drag)
        self.bind('<ButtonRelease-1>', lambda e: setattr(self, '_dragging', False))
        self.bind('<MouseWheel>', self._wheel)
        self.bind('<Button-4>', lambda e: self._wheel(e, 1))
        self.bind('<Button-5>', lambda e: self._wheel(e, -1))
        self.redraw()

    def _val2ang(self, v):
        f = (v - self.vmin) / float(self.vmax - self.vmin)
        f = max(0.0, min(1.0, f))
        return math.radians(-135 + 270 * f)

    def _press(self, ev):
        self._dragging = True
        self._drag(ev)

    def _drag(self, ev):
        dx, dy = ev.x - self.cx, ev.y - self.cy
        a = math.degrees(math.atan2(dy, dx)) + 90      # 0 = вверх
        if a > 180: a -= 360
        a = max(-135, min(135, a))                     # не даём перескочить
        f = (a + 135) / 270.0
        self.value = self.vmin + f * (self.vmax - self.vmin)
        self.redraw()
        self.on_change(self.value)

    def _wheel(self, ev, direction=None):
        step = (self.vmax - self.vmin) / 100.0
        d = direction if direction is not None else (1 if ev.delta > 0 else -1)
        self.value = max(self.vmin, min(self.vmax, self.value + d * step))
        self.redraw()
        self.on_change(self.value)

    def redraw(self):
        self.delete('all')
        cx, cy, r = self.cx, self.cy, self.r
        # шкала-дуга (серая, вся)
        self.create_arc(cx-r-6, cy-r-6, cx+r+6, cy+r+6,
                        start=-45, extent=-270, style=tk.ARC,
                        outline='#374151', width=3)
        # заполненная часть (зелёная)
        f = (self.value - self.vmin) / float(self.vmax - self.vmin)
        f = max(0.0, min(1.0, f))
        self.create_arc(cx-r-6, cy-r-6, cx+r+6, cy+r+6,
                        start=-45, extent=-270*f, style=tk.ARC,
                        outline='#22c55e', width=3)
        # корпус
        self.create_oval(cx-r, cy-r, cx+r, cy+r,
                         fill='#1f2937', outline='#4b5563', width=2)
        # риска
        a = self._val2ang(self.value)
        self.create_line(cx + r*0.35*math.sin(a), cy - r*0.35*math.cos(a),
                         cx + r*0.85*math.sin(a), cy - r*0.85*math.cos(a),
                         fill='#22c55e', width=4, capstyle=tk.ROUND)
        self.create_text(cx, cy+r*0.45, text=f'{int(self.value)}',
                         fill='#e5e7eb', font=('Consolas', 13, 'bold'))

    def set_value(self, v):
        """Обновление извне (телеметрия) — без обратного вызова."""
        self.value = max(self.vmin, min(self.vmax, float(v)))
        self.redraw()


class PidTuner:
    def __init__(self, root):
        self.root = root
        self.root.title('PID Tuner — контроллер двигателя')
        self.root.geometry('1180x840')
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)
        self.ser = None
        self.running = False

        # --- Данные ---
        self.rpm_hist = [0.0] * 400
        self.cur_hist = [0.0] * 400
        self.raw = b''
        self.lock = threading.Lock()
        self.term_buf = []      # строки для терминала
        self.term_echo = False  # показывать ли поток телеметрии
        self.live = {}
        self.zones = None
        self.rpm_smooth = 0.0
        self.motor_v_smooth = 0.0
        self.power_smooth = 0.0     # сглаженная мощность, Вт
        self.power_w = 0.0
        self.u_dc_live = 325.0      # напряжение шины (после моста), В
        self.u_mot_live = 0.0       # напряжение на двигателе (ШИМ), В
        self.i_bus_live = 0.0       # ток шины (якорь × скважность), А
        self.ovr_prev = 0
        self.ovr_rate = 0
        self.ovr_tick = 0
        self.csv_file = None
        self.csv_writer = None
        self.logging = False

        # --- Очередь команд ---
        self.txq = queue.Queue()
        self._pid_job = None
        self._ramp_job = None
        self._ol_job = None
        self._filt_job = None
        self._ci_job = None
        self._rv_job = None
        self._rlim_job = None
        self._pot_anchor = None     # положение резистора на момент ухода на ПК
        self.last_frame = 0.0
        self.step_t0 = 0.0          # замер перерегулирования: 0 = не идёт
        self.step_to = 0
        self.step_from = 0
        self.step_peak = 0.0

        # --- Калибровка тока ---
        self.shunt_r = 0.075
        self.gain = 8.2

        self.panel_hidden = False
        self.dirty = False          # есть несохранённые во Flash настройки

        # --- Экран выбора порта ---
        self.port_frame = tk.Frame(root)
        self.port_frame.pack(fill=tk.BOTH, expand=True)
        self.show_ports()

    @property
    def adc_to_a(self):
        return 3.3 / (4095.0 * self.shunt_r * self.gain)

    # ================= Выбор порта =================
    def show_ports(self):
        ports = list(serial.tools.list_ports.comports())
        tk.Label(self.port_frame, text='Выбери COM-порт контроллера:',
                 font=('Arial', 14)).pack(pady=20)
        if not ports:
            tk.Label(self.port_frame, text='Нет COM-портов', fg='red').pack()
            tk.Button(self.port_frame, text='Выход', command=self.root.destroy).pack()
            return
        self.listbox = tk.Listbox(self.port_frame, height=7, font=('Consolas', 11))
        for p in ports:
            self.listbox.insert(tk.END, f'{p.device} - {p.description}')
        self.listbox.pack(fill=tk.BOTH, expand=True, padx=40, pady=5)
        tk.Button(self.port_frame, text='Подключить',
                  command=lambda: self.connect(ports),
                  bg='#3b82f6', fg='white', padx=30, pady=6).pack(pady=20)

    def connect(self, ports):
        idx = self.listbox.curselection()
        if not idx:
            return
        try:
            self.ser = serial.Serial(ports[idx[0]].device, BAUD, timeout=0.3)
            time.sleep(0.4)
            self.ser.reset_input_buffer()
            self.ser.write(b'j')
            time.sleep(0.1)
            self.port_frame.pack_forget()
            self.running = True
            threading.Thread(target=self.tx_loop, daemon=True).start()
            threading.Thread(target=self.read_loop, daemon=True).start()
            self.start_ui()
            self.root.after(1000, self.send_heartbeat)
        except Exception as e:
            err = tk.Label(self.port_frame, text=f'Ошибка: {e}', fg='red')
            err.pack(pady=5)
            self.root.after(3000, err.destroy)

    def send_heartbeat(self):
        """Признак живой связи для прошивки. Телеметрию она отдаёт сама,
        без запросов, поэтому иначе TRIP_COMM срабатывает на исправном
        кабеле — просто потому, что человек ничего не трогает."""
        try:
            if self.ser is not None and self.ser.is_open:
                self.send_cmd('PG')
        except Exception:
            pass
        self.root.after(1000, self.send_heartbeat)

    # ================= Очередь команд =================
    def send_cmd(self, cmd):
        self.txq.put(cmd)
        # Команды-настройки (не пуск/стоп, не RV/RP) — помечаем несохранёнными.
        # RP/RV — источник уставки, не настройка; он же сам ставит sp_src.
        if hasattr(self, 'btn_save') and cmd:
            if cmd.startswith(('RV', 'RP', 'RS')):
                pass                        # источник оборотов / сброс — не в Flash
            elif cmd[0] in 'CDEFIKPRSTUV':
                self.dirty = True
                self.btn_save.config(text='Сохранить во Flash •', bg='#f97316')

    def tx_loop(self):
        while self.running:
            try:
                c = self.txq.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self.ser.write((c + '\n').encode())
                self.ser.flush()
            except Exception:
                pass
            time.sleep(0.035)

    # ================= Отложенные ползунки =================
    def schedule_pid(self):
        if self._pid_job:
            self.root.after_cancel(self._pid_job)
        self._pid_job = self.root.after(150, self.send_pid)

    def schedule_ramp(self):
        if self._ramp_job:
            self.root.after_cancel(self._ramp_job)
        self._ramp_job = self.root.after(150, self.send_ramp)

    def schedule_ol(self):
        if self._ol_job:
            self.root.after_cancel(self._ol_job)
        self._ol_job = self.root.after(150, self.send_ol)

    def schedule_filt(self):
        if self._filt_job:
            self.root.after_cancel(self._filt_job)
        self._filt_job = self.root.after(150, self.send_filt)

    def schedule_ci(self):
        if self._ci_job:
            self.root.after_cancel(self._ci_job)
        self._ci_job = self.root.after(150, self.send_ci)

    def can_sync(self, var, hold=1.2):
        """Можно ли подтягивать ползунок из прошивки.
        Нельзя, пока он под мышью и ещё hold секунд после правки:
        в это окно приходят кадры со старым значением, и они откатят
        то, что человек только что выставил."""
        if getattr(var, '_drag', False):
            return False
        return (time.time() - getattr(var, '_touched', 0.0)) > hold

    def group_quiet(self, vars_, hold=1.5):
        """Группа ползунков «спокойна»: ни один не под мышью и ни один
        не правился последние hold секунд. Kp/Ki/Kd уходят в прошивку
        одной командой, поэтому и откатывать их можно только все вместе —
        иначе правка одного сбрасывает соседний."""
        now = time.time()
        for v in vars_:
            if getattr(v, '_drag', False):
                return False
            if (now - getattr(v, '_touched', 0.0)) <= hold:
                return False
        return True

    def pull_pid(self):
        """Разово подтянуть все ползунки ПИД под значения прошивки."""
        live = self.live or {}
        self.sync_var(self.var_kp, int(live.get('kp', 0)))
        self.sync_var(self.var_ki, int(live.get('ki', 0)))
        self.sync_var(self.var_kd, int(live.get('kd', 0)))
        self.sync_var(self.var_cikp, int(live.get('cikp', 0)))
        self.sync_var(self.var_ciki, int(live.get('ciki', 0)))

    def sync_group(self, pairs, job=None, hold=1.5):
        """Подтянуть группу ползунков из телеметрии.
        pairs: [(var, значение_из_прошивки), ...]
        job:   флаг отложенной отправки этой группы.
        Группа откатывается только целиком и только когда её никто
        не трогает — иначе правка одного ползунка сбрасывает соседний."""
        if job is not None:
            return
        vars_ = [v for v, _ in pairs]
        if not self.group_quiet(vars_, hold):
            return
        for v, val in pairs:
            if val is None:
                continue
            if int(val) != int(round(v.get())):
                self.sync_var(v, int(val))

    def sync_var(self, var, value):
        """Подтянуть ползунок И число рядом с ним из прошивки."""
        try:
            var._busy = True          # не даём command уйти в schedule_pid
            var.set(value)
            sv = getattr(var, '_sv', None)
            if sv is not None:
                sv.set(str(int(round(value))))
        except Exception:
            pass
        finally:
            try:
                var._busy = False
            except Exception:
                pass

    def schedule_rv(self):
        if self._rv_job:
            self.root.after_cancel(self._rv_job)
        self._rv_job = self.root.after(120, self.send_rpm_virt)

    def use_virt(self):
        """Переключить задатчик оборотов на виртуальный (с ПК).
        Якорь сбрасываем: он возьмётся из первого же кадра телеметрии,
        и отсчёт «резистор тронули» пойдёт от текущего положения ручки,
        а не от той, что была при прошлом уходе на ПК."""
        self.sp_src = 'virt'
        self._pot_anchor = None
        self.send_rpm_virt()

    def use_pot(self):
        """Переключить задатчик оборотов на реальный потенциометр (RP).
        Отменяем отложенную отправку RV — иначе виртуальный ползунок
        перекроет команду через 120 мс и вернёт прошивку в виртуальный режим."""
        if self._rv_job:
            self.root.after_cancel(self._rv_job)
            self._rv_job = None
        self._pot_anchor = None     # положение резистора на момент ухода на ПК
        self.sp_src = 'pot'
        self.send_cmd('RP')

    # ================= Интерфейс =================
    def start_ui(self):
        # --- Сверху: панель инструментов (две строки, чтобы всегда влезало) ---
        toolbar = tk.Frame(self.root, bg='#1a202c')
        toolbar.pack(side=tk.TOP, fill=tk.X)

        row1 = tk.Frame(toolbar, bg='#1a202c'); row1.pack(fill=tk.X)
        row2 = tk.Frame(toolbar, bg='#1a202c'); row2.pack(fill=tk.X)

        tk.Label(row1, text='Двиг.:', bg='#1a202c', fg='#eab308',
                 font=('Arial', 10)).pack(side=tk.LEFT, padx=4)
        self.btn_on = tk.Button(row1, text='ВКЛ', command=lambda: self.send_cmd('ON'),
                                width=5, bg='#4b5563', fg='white')
        self.btn_on.pack(side=tk.LEFT, padx=2)
        self.btn_of = tk.Button(row1, text='ВЫКЛ', command=lambda: self.send_cmd('OF'),
                                width=5, bg='#4b5563', fg='white')
        self.btn_of.pack(side=tk.LEFT, padx=2)
        self.btn_hb = tk.Button(row1, text='Кнопка', command=lambda: self.send_cmd('HB'),
                                width=6, bg='#6b7280', fg='white')
        self.btn_hb.pack(side=tk.LEFT, padx=2)
        self.btn_rs = tk.Button(row1, text='СБРОС', command=self.do_reset,
                                width=6, bg='#ef4444', fg='white')
        self.btn_rs.pack(side=tk.LEFT, padx=4)

        # Индикатор двигателя: красный при вращении, синий в покое.
        self.lbl_motor = tk.Label(row1, text=' ДВИГ ВЫКЛ ', width=12,
                                  font=('Arial', 11, 'bold'),
                                  bg='#1f2937', fg='#60a5fa',
                                  relief='solid', bd=2)
        self.lbl_motor.pack(side=tk.LEFT, padx=8)

        # --- Задатчик оборотов: резистор или задание с ПК ---
        tk.Label(row1, text='Обороты:', bg='#1a202c', fg='#eab308',
                 font=('Arial', 10)).pack(side=tk.LEFT, padx=(10, 4))
        self.btn_pot = tk.Button(row1, text='Резистор', command=self.use_pot,
                                 width=9, bg='#4b5563', fg='white')
        self.btn_pot.pack(side=tk.LEFT, padx=2)
        self.btn_virt = tk.Button(row1, text='С ПК', command=self.use_virt,
                                  width=6, bg='#4b5563', fg='white')
        self.btn_virt.pack(side=tk.LEFT, padx=2)

        tk.Label(row1, text='Защита:', bg='#1a202c', fg='#eab308',
                 font=('Arial', 10)).pack(side=tk.LEFT, padx=4)
        self.btn_p0 = tk.Button(row1, text='0', command=lambda: self.send_cmd('0'),
                                width=3, bg='#4b5563', fg='white')
        self.btn_p0.pack(side=tk.LEFT, padx=1)
        self.btn_p1 = tk.Button(row1, text='1', command=lambda: self.send_cmd('1'),
                                width=3, bg='#4b5563', fg='white')
        self.btn_p1.pack(side=tk.LEFT, padx=1)
        self.btn_p2 = tk.Button(row1, text='2', command=lambda: self.send_cmd('2'),
                                width=3, bg='#4b5563', fg='white')
        self.btn_p2.pack(side=tk.LEFT, padx=1)
        self.btn_p3 = tk.Button(row1, text='3', command=lambda: self.send_cmd('3'),
                                width=3, bg='#4b5563', fg='white')
        self.btn_p3.pack(side=tk.LEFT, padx=1)

        tk.Label(row2, text='ПИД:', bg='#1a202c', fg='#eab308',
                 font=('Arial', 10)).pack(side=tk.LEFT, padx=4)
        self.btn_pa = tk.Button(row2, text='Авто', command=lambda: self.send_cmd('a'),
                                width=5, bg='#4b5563', fg='white')
        self.btn_pa.pack(side=tk.LEFT, padx=2)
        self.btn_pm = tk.Button(row2, text='Ручн.', command=lambda: self.send_cmd('m'),
                                width=5, bg='#4b5563', fg='white')
        self.btn_pm.pack(side=tk.LEFT, padx=2)

        tk.Label(row2, text='Пульс.:', bg='#1a202c', fg='#eab308',
                 font=('Arial', 10)).pack(side=tk.LEFT, padx=4)
        self.btn_pr = tk.Button(row2, text='Вкл', command=lambda: self.send_cmd('r'),
                                width=4, bg='#4b5563', fg='white')
        self.btn_pr.pack(side=tk.LEFT, padx=2)
        self.btn_ps = tk.Button(row2, text='Выкл', command=lambda: self.send_cmd('s'),
                                width=4, bg='#4b5563', fg='white')
        self.btn_ps.pack(side=tk.LEFT, padx=2)

        tk.Label(row2, text='Формат:', bg='#1a202c', fg='#eab308',
                 font=('Arial', 10)).pack(side=tk.LEFT, padx=4)
        self.btn_j = tk.Button(row2, text='JSON', command=lambda: self.send_cmd('j'),
                               width=5, bg='#4b5563', fg='white')
        self.btn_j.pack(side=tk.LEFT, padx=2)
        self.btn_o = tk.Button(row2, text='O-кадр', command=lambda: self.send_cmd('o'),
                               width=6, bg='#4b5563', fg='white')
        self.btn_o.pack(side=tk.LEFT, padx=2)

        self.btn_hide = tk.Button(row2, text='▼ Скрыть панель',
                                  command=self.toggle_panel, width=14,
                                  bg='#374151', fg='white')
        self.btn_hide.pack(side=tk.RIGHT, padx=8)
        self.root.bind('<F9>', lambda e: self.toggle_panel())

        # --- ВНИЗУ: вкладки (фикс. высоты) и статус — ПЕРЕД графиками ---
        # Сначала вкладки — прижаты к самому низу
        # 260, не 205: на вкладке ПИД семь рядов (последний — время форсажа),
        # в 205 px нижний ряд обрезался краем контейнера.
        self.nb_holder = tk.Frame(self.root, height=260)
        self.nb_holder.pack(side=tk.BOTTOM, fill=tk.X)
        self.nb_holder.pack_propagate(False)      # не растёт, не сжимается
        nb = ttk.Notebook(self.nb_holder)
        nb.pack(fill=tk.BOTH, expand=True)

        tab_pid = tk.Frame(nb); nb.add(tab_pid, text='ПИД')
        tab_ramp = tk.Frame(nb); nb.add(tab_ramp, text='Разгон')
        tab_ol = tk.Frame(nb); nb.add(tab_ol, text='Разомкнутый')
        tab_filt = tk.Frame(nb); nb.add(tab_filt, text='Фильтры')
        tab_current = tk.Frame(nb); nb.add(tab_current, text='Ток')
        tab_tq = tk.Frame(nb); nb.add(tab_tq, text='Тяга')
        tab_zones = tk.Frame(nb); nb.add(tab_zones, text='Зоны')
        tab_hw = tk.Frame(nb); nb.add(tab_hw, text='Железо')
        tab_log = tk.Frame(nb); nb.add(tab_log, text='Лог/Тест')
        tab_term = tk.Frame(nb); nb.add(tab_term, text='Терминал')
        tab_sp = tk.Frame(nb); nb.add(tab_sp, text='Задатчик')

        self.build_pid(tab_pid)
        self.build_ramp(tab_ramp)
        self.build_ol(tab_ol)
        self.build_filt(tab_filt)
        self.build_current(tab_current)
        self.build_torque(tab_tq)
        self.build_zones(tab_zones)
        self.build_hw(tab_hw)
        self.build_log(tab_log)
        self.build_term(tab_term)
        self.build_setpoint(tab_sp)

        # Потом статус — одна строка над вкладками, с прибитой справа кнопкой
        # «Сохранить во Flash». Кнопка относится ко ВСЕМ вкладкам и видна всегда:
        # на восьмой вкладке «Железо» она обрезалась краем контейнера 205 px.
        self.bar = tk.Frame(self.root, bg='#101418')
        self.bar.pack(side=tk.BOTTOM, fill=tk.X)

        self.btn_save = tk.Button(self.bar, text='Сохранить во Flash',
                                  command=self.do_save, bg='#16a34a', fg='white',
                                  padx=14, pady=2, font=('Arial', 9, 'bold'))
        self.btn_save.pack(side=tk.RIGHT, padx=6, pady=2, anchor='n')

        # Две строки вместо одной: в одну не влезают ~200 знаков телеметрии,
        # и хвост с «TRIP=...» уезжал под кнопку сохранения. Внизу — только
        # аварии и режимы, они всегда на виду.
        txt = tk.Frame(self.bar, bg='#101418')
        txt.pack(side=tk.LEFT, fill=tk.X, expand=True)

        self.status = tk.Label(txt, text='...', font=('Consolas', 10),
                               bg='#101418', fg='#e5e7eb', anchor='w', padx=8)
        self.status.pack(side=tk.TOP, fill=tk.X)

        self.status2 = tk.Label(txt, text='', font=('Consolas', 11, 'bold'),
                                bg='#101418', fg='#e5e7eb', anchor='w', padx=8)
        self.status2.pack(side=tk.TOP, fill=tk.X)

        # --- ПОСЛЕДНИМИ: графики — забирают весь оставшийся размер ---
        self.graph = tk.Frame(self.root)
        self.graph.rowconfigure(0, weight=2)      # обороты — вдвое выше тока
        self.graph.rowconfigure(1, weight=1)
        self.graph.columnconfigure(0, weight=1)
        self.graph.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.canv_rpm = tk.Canvas(self.graph, bg='black', highlightthickness=1,
                                  highlightbackground='#333')
        self.canv_rpm.grid(row=0, column=0, sticky='nsew')
        self.canv_cur = tk.Canvas(self.graph, bg='black', highlightthickness=1,
                                  highlightbackground='#333')
        self.canv_cur.grid(row=1, column=0, sticky='nsew')

        self.draw()

    def toggle_panel(self):
        if self.panel_hidden:
            # before= должен указывать на СЕМЕНЬ соседа в том же родителе:
            # self.status лежит внутри self.bar, а nb_holder в root —
            # раньше панель попадала в список упаковки bar и обрезалась.
            self.nb_holder.pack(side=tk.BOTTOM, fill=tk.X, before=self.bar)
            self.btn_hide.config(text='▼ Скрыть панель')
        else:
            self.nb_holder.pack_forget()
            self.btn_hide.config(text='▲ Показать панель')
        self.panel_hidden = not self.panel_hidden

    # ================= Компактный ряд ползунка =================
    def slider_row(self, parent, row, text, var, lo, hi, cb, width=170, c0=0):
        tk.Label(parent, text=text, width=14, anchor='e',
                 font=('Arial', 9)).grid(row=row, column=c0, sticky='e', padx=(6, 2))
        sv = tk.StringVar(value=str(int(round(var.get()))))
        var._busy = False        # защита от рекурсии var.set → command
        var._drag = False        # ползунок под мышью
        var._touched = 0.0       # время последней правки руками

        def _on(v):
            if var._busy:
                return
            iv = int(round(float(v)))
            sv.set(str(iv))
            var._touched = time.time()
            if abs(float(v) - iv) > 1e-6:
                var._busy = True
                try:
                    var.set(iv)
                finally:
                    var._busy = False
            cb()

        s = ttk.Scale(parent, from_=lo, to=hi, variable=var,
                      orient=tk.HORIZONTAL, length=width, command=_on)
        s.grid(row=row, column=c0+1, sticky='ew', padx=2)
        # Пока кнопка мыши зажата — синхронизация не трогает ползунок,
        # иначе он вырывается из-под курсора и вернуть его невозможно.
        s.bind('<ButtonPress-1>',   lambda e: setattr(var, '_drag', True),  add='+')
        s.bind('<ButtonRelease-1>', lambda e: (setattr(var, '_drag', False),
                                               setattr(var, '_touched', time.time())), add='+')
        tk.Label(parent, textvariable=sv, width=6, anchor='w',
                 font=('Consolas', 9), fg='#3b82f6').grid(row=row, column=c0+2, sticky='w')
        # Запоминаем метку и ползунок на переменной: при программном var.set()
        # command у ttk.Scale НЕ вызывается, и число рядом остаётся старым.
        # Через var._sv его обновляет sync_var().
        var._sv = sv
        var._scale = s
        return s

    # ================= Вкладка ПИД =================
    def build_pid(self, tab):
        self.var_kp = tk.DoubleVar(value=500)
        self.var_ki = tk.DoubleVar(value=100)
        self.var_kd = tk.DoubleVar(value=0)
        self.var_cikp = tk.DoubleVar(value=400)
        self.var_ciki = tk.DoubleVar(value=200)

        # две колонки по три ряда — всё видно разом
        tab.columnconfigure(1, weight=1)
        tab.columnconfigure(4, weight=1)

        self.slider_row(tab, 0, 'Kp (×1000)', self.var_kp, 0, 2000, self.schedule_pid)
        self.slider_row(tab, 1, 'Ki (×1000)', self.var_ki, 0, 1000, self.schedule_pid)
        self.slider_row(tab, 2, 'Kd (×1000)', self.var_kd, 0,  500, self.schedule_pid)

        self.slider_row(tab, 0, 'Kp тока (CP)', self.var_cikp, 0, 5000, self.schedule_ci, 170, c0=3)
        self.slider_row(tab, 1, 'Ki тока (CI)', self.var_ciki, 0, 5000, self.schedule_ci, 170, c0=3)

        tk.Label(tab, text='в прошивке:', fg='#6b7280', font=('Arial', 9)) \
            .grid(row=3, column=0, sticky='e', padx=(6, 2))
        self.lbl_kp_fw = tk.Label(tab, text='Kp —', fg='#6b7280', font=('Consolas', 9))
        self.lbl_kp_fw.grid(row=3, column=1, sticky='w')
        self.lbl_ki_fw = tk.Label(tab, text='Ki —', fg='#6b7280', font=('Consolas', 9))
        self.lbl_ki_fw.grid(row=3, column=3, sticky='e')
        self.lbl_kd_fw = tk.Label(tab, text='Kd —', fg='#6b7280', font=('Consolas', 9))
        self.lbl_kd_fw.grid(row=3, column=4, sticky='w')

        self.lbl_ci_fw = tk.Label(tab, text='ток: CP — CI —', fg='#6b7280',
                                  font=('Consolas', 9))
        self.lbl_ci_fw.grid(row=4, column=3, columnspan=2, sticky='w', padx=8)

        self.lbl_mode = tk.Label(tab, text='Режим: —', font=('Arial', 10, 'bold'))
        self.lbl_mode.grid(row=4, column=0, columnspan=3, sticky='w', padx=8, pady=4)
        self._prev_auto = 1   # при переходе авто→ручной один раз подтянуть ползунки

        self.lbl_mul = tk.Label(tab, text='ripple: ×1.00 Kp  ×1.00 Kd',
                                fg='#6b7280', font=('Consolas', 9))
        self.lbl_mul.grid(row=5, column=3, columnspan=2, sticky='w')

        # --- Форсаж: состояние и регулировка времени ---
        self.lbl_boost = tk.Label(tab, text='Форсаж: —', fg='#9ca3af',
                                  font=('Consolas', 9), anchor='w')
        self.lbl_boost.grid(row=5, column=0, columnspan=5, sticky='w', padx=8)

        tk.Label(tab, text='Время форсажа, мс').grid(row=6, column=0, sticky='w', padx=8)
        self.var_bt = tk.IntVar(value=1500)
        tk.Scale(tab, from_=0, to=5000, resolution=100, orient='horizontal',
                 variable=self.var_bt, length=240).grid(row=6, column=1, columnspan=3, sticky='we')
        tk.Button(tab, text='Применить',
                  command=lambda: self.send_cmd('BT%d' % self.var_bt.get())
                  ).grid(row=6, column=4, padx=6)

    def send_pid(self):
        self._pid_job = None      # отложенная отправка отработала
        kp = int(round(self.var_kp.get()))
        ki = int(round(self.var_ki.get()))
        kd = int(round(self.var_kd.get()))
        self.send_cmd(f'KP{kp}')
        self.send_cmd(f'KI{ki}')
        self.send_cmd(f'KD{kd}')

    def send_ci(self):
        self._ci_job = None       # отложенная отправка отработала
        self.send_cmd(f'CP{int(round(self.var_cikp.get()))}')
        self.send_cmd(f'CI{int(round(self.var_ciki.get()))}')

    # ================= Вкладка Разгон =================
    def build_ramp(self, tab):
        self.var_ss = tk.DoubleVar(value=DEFAULT_SS)
        self.var_ru = tk.DoubleVar(value=DEFAULT_RU)
        self.var_rd = tk.DoubleVar(value=DEFAULT_RD)
        tab.columnconfigure(1, weight=1)
        # ссылки на Scale сохраняем: синхронизация не трогает ползунок,
        # пока он под мышью или ждёт отложенная отправка (_ramp_job)
        self.sc_ss = self.slider_row(tab, 0, 'SS нарастание ШИМ', self.var_ss, 1, 100, self.schedule_ramp, 300)
        self.sc_ru = self.slider_row(tab, 1, 'RU разгон уставки', self.var_ru, 1, 100, self.schedule_ramp, 300)
        self.sc_rd = self.slider_row(tab, 2, 'RD торможение уставки', self.var_rd, 1, 100, self.schedule_ramp, 300)

        # Рабочий диапазон оборотов. Потолок железа RPM_MAX=6000 остаётся
        # в прошивке, эти два ползунка только сужают диапазон внутри него:
        # полный ход потенциометра растягивается на заданный размах.
        # Минимум держим низким: высокий минимум отнимает возможность
        # остановить двигатель потенциометром — ручка в нуле будет
        # давать rpm_lo, а не ноль.
        self.var_rlo = tk.DoubleVar(value=RPM_MIN)
        self.var_rhi = tk.DoubleVar(value=RPM_MAX)
        self.slider_row(tab, 3, 'Мин. обороты', self.var_rlo,
                        RPM_MIN, 600, self.schedule_rlim, 300)
        self.slider_row(tab, 4, 'Потолок оборотов', self.var_rhi,
                        RPM_MIN + 100, RPM_MAX, self.schedule_rlim, 300)
        self.lbl_rlim = tk.Label(tab, text='диапазон: 30…6000 об/мин',
                                 fg='#6b7280', font=('Consolas', 9))
        self.lbl_rlim.grid(row=5, column=0, columnspan=3, sticky='w', padx=8)

    def send_ramp(self):
        self.send_cmd(f'SS{int(round(self.var_ss.get()))}')
        self.send_cmd(f'RU{int(round(self.var_ru.get()))}')
        self.send_cmd(f'RD{int(round(self.var_rd.get()))}')
        self._ramp_job = None        # отправлено — снова принимаем телеметрию

    def schedule_rlim(self):
        if self._rlim_job:
            self.root.after_cancel(self._rlim_job)
        self._rlim_job = self.root.after(150, self.send_rlim)

    def send_rlim(self):
        self._rlim_job = None
        lo = int(round(self.var_rlo.get()))
        hi = int(round(self.var_rhi.get()))
        # Не даём ползункам перехлестнуться: прошивка такую пару отвергнет
        # молча, и человек не поймёт, почему значение не применилось.
        if hi < lo + 100:
            if self.var_rhi._touched >= self.var_rlo._touched:
                lo = hi - 100
                self.sync_var(self.var_rlo, lo)
            else:
                hi = lo + 100
                self.sync_var(self.var_rhi, hi)
        # Порядок важен: сначала расширяем, потом сужаем, иначе прошивка
        # отвергнет промежуточную пару по правилу rlo+100 <= rhi.
        if hi > int(round(self.var_rhi.get())):
            self.send_cmd(f'RH{hi}'); self.send_cmd(f'RL{lo}')
        else:
            self.send_cmd(f'RL{lo}'); self.send_cmd(f'RH{hi}')

    # ================= Вкладка Разомкнутый =================
    def build_ol(self, tab):
        self.var_ol_enable = tk.IntVar(value=0)
        self.var_ol_duty = tk.DoubleVar(value=100)
        self.var_ol_pot = tk.IntVar(value=0)
        tab.columnconfigure(1, weight=1)

        cb = tk.Checkbutton(tab, text='Включить разомкнутый режим (OL1)',
                            variable=self.var_ol_enable, command=self.toggle_ol,
                            font=('Arial', 10, 'bold'))
        cb.grid(row=0, column=0, columnspan=2, sticky='w', padx=10, pady=6)

        self.sc_ol = self.slider_row(tab, 1, 'Задание ШИМ (0..900)', self.var_ol_duty,
                        0, PWM_MAX, self.schedule_ol, 260)
        self.lbl_ol_pct = tk.Label(tab, text='0%', fg='#3b82f6', font=('Consolas', 11))
        self.lbl_ol_pct.grid(row=1, column=3, sticky='w', padx=6)

        tk.Checkbutton(tab, text='Задание с потенциометра (OT1)',
                       variable=self.var_ol_pot, command=self.toggle_ol_pot
                       ).grid(row=2, column=0, sticky='w', padx=10, pady=6)

        self.lbl_ol_status = tk.Label(tab, text='Режим выключен', fg='#6b7280')
        self.lbl_ol_status.grid(row=3, column=0, sticky='w', padx=10)

    def toggle_ol(self):
        if self.var_ol_enable.get():
            if self.var_tq.get():
                self.var_tq.set(0)
                self.send_cmd('TQ0')
                self.lbl_tq.config(text='Обороты стабилизируются', fg='#6b7280')
            self.send_cmd('OL1')
            self.lbl_ol_status.config(text='Активен', fg='#22c55e')
        else:
            self.send_cmd('OL0')
            self.lbl_ol_status.config(text='Режим выключен', fg='#6b7280')

    def toggle_ol_pot(self):
        if self.var_ol_pot.get():
            self.send_cmd('OT1')
        else:
            self.send_cmd('OT0')

    def toggle_tq(self):
        if self.var_tq.get():
            if self.var_ol_enable.get():
                self.var_ol_enable.set(0)
                self.send_cmd('OL0')
                self.lbl_ol_status.config(text='Режим выключен', fg='#6b7280')
            self.send_cmd('TQ1')
            self.lbl_tq.config(text='ТЯГА: обороты свободны!', fg='#f59e0b')
        else:
            self.send_cmd('TQ0')
            self.lbl_tq.config(text='Обороты стабилизируются', fg='#6b7280')

    def send_tq_par(self, cmd):
        raw = self.tq_vars[cmd].get()
        try:
            if cmd in ('TM', 'TF'):
                a = float(raw)
                if a <= 0:
                    self.send_cmd(f'{cmd}0')
                    return
                adc = int(round(a / self.adc_to_a))
                if cmd == 'TM' and adc > 3000: adc = 3000
                if cmd == 'TF' and adc > 1000: adc = 1000
                self.send_cmd(f'{cmd}{adc}')
            else:
                v = int(float(raw))
                self.send_cmd(f'{cmd}{v}')
        except Exception:
            return

    def send_tq_all(self):
        for c in self.tq_vars:
            self.send_tq_par(c)

    def send_ol(self):
        self.send_cmd(f'OP{int(round(self.var_ol_duty.get()))}')
        self._ol_job = None          # отправлено — снова принимаем телеметрию

    # ================= Вкладка Фильтры =================
    def build_filt(self, tab):
        # Умолчания = заводские значения прошивки (mc.c): FR=2, FD=2, SL=4.
        # Синхронизация подтянет реальные, но при отсутствии связи видно правду.
        self.var_fr = tk.DoubleVar(value=2)
        self.var_fd = tk.DoubleVar(value=2)
        self.var_sl = tk.DoubleVar(value=4)
        tab.columnconfigure(1, weight=1)
        self.sc_fr = self.slider_row(tab, 0, 'FR фильтр оборотов', self.var_fr, 1, 4, self.schedule_filt, 300)
        self.sc_fd = self.slider_row(tab, 1, 'FD фильтр D', self.var_fd, 1, 4, self.schedule_filt, 300)
        self.sc_sl = self.slider_row(tab, 2, 'SL делитель контура', self.var_sl, 1, 8, self.schedule_filt, 300)

    def send_filt(self):
        self.send_cmd(f'FR{int(round(self.var_fr.get()))}')
        self.send_cmd(f'FD{int(round(self.var_fd.get()))}')
        self.send_cmd(f'SL{int(round(self.var_sl.get()))}')
        self._filt_job = None        # отправлено — снова принимаем телеметрию

    # ================= Вкладка Зоны =================
    ZONE_NAMES = ['Холостой ход', 'Низкие', 'Ниже средних',
                  'Средние', 'Высокие', 'Максимум']

    def build_zones(self, tab):
        # Без заголовка: вкладка и так называется «Зоны», а место ограничено
        # контейнером nb_holder (205 px). Заголовок съедал ~25 px, из-за чего
        # шестая зона («Максимум») обрезалась.
        for c, h in enumerate(['Зона', 'До об/мин', 'Kp', 'Ki', 'Kd']):
            tk.Label(tab, text=h, font=('Arial', 8, 'bold')) \
                .grid(row=0, column=c, padx=4, pady=(2, 1))

        self.zvars = []
        for i in range(6):
            tk.Label(tab, text=f'{i} {self.ZONE_NAMES[i]}', anchor='w',
                     font=('Arial', 8)).grid(row=1 + i, column=0, sticky='w', padx=6)
            row = {}
            for c, key in enumerate(['rpm', 'kp', 'ki', 'kd']):
                v = tk.StringVar(value='—')
                e = tk.Entry(tab, textvariable=v, width=7, justify='right',
                             font=('Arial', 8))
                e.grid(row=1 + i, column=1 + c, padx=2, pady=0)
                e.bind('<Return>', lambda ev, n=i: self.send_zone(n))
                row[key] = v
            self.zvars.append(row)

        # Кнопки — СПРАВА от таблицы, а не под ней: экономит целую строку.
        fr = tk.Frame(tab)
        fr.grid(row=0, column=5, rowspan=7, sticky='n', padx=10, pady=2)
        tk.Button(fr, text='Считать (ZD)', command=self.pull_zones,
                  padx=6, pady=1).pack(fill=tk.X, pady=2)
        tk.Button(fr, text='Заводские (ZR)', command=self.zones_default,
                  padx=6, pady=1).pack(fill=tk.X, pady=2)
        tk.Label(fr, text='Enter в поле —\nотправить зону',
                 font=('Arial', 7), fg='#9ca3af', justify='left').pack(anchor='w')

    def pull_zones(self):
        with self.lock:
            self.zones = None
        self.send_cmd('ZD')
        self.root.after(500, self._fill_zones)

    def _fill_zones(self):
        with self.lock:
            z = self.zones
        if not z:
            self.status.config(text='Таблица зон не пришла — нужна прошивка с ZD')
            return
        for i, row in enumerate(z[:6]):
            if len(row) < 4:
                continue
            for c, key in enumerate(['rpm', 'kp', 'ki', 'kd']):
                self.zvars[i][key].set(str(row[c]))

    def send_zone(self, n):
        """Отправить всю зону целиком — с проверкой до отправки.
        Раньше половина полей улетала в контроллер, а при опечатке в одном
        поле зона оставалась полузаписанной, и без всякого сообщения. """
        batch = []
        for key, L in [('kp', 'P'), ('ki', 'I'), ('kd', 'D'), ('rpm', 'R')]:
            raw = self.zvars[n][key].get().strip()
            if raw in ('', '—'):
                continue
            try:
                v = int(float(raw))
            except Exception:
                self.status.config(fg='#ef4444',
                                   text=f'  Зона {n}: «{raw}» — не число')
                return
            lo, hi = (50, 12000) if key == 'rpm' else (0, 5000)
            if not (lo <= v <= hi):
                self.status.config(fg='#ef4444',
                                   text=f'  Зона {n}: {v} вне диапазона {lo}..{hi}')
                return
            batch.append(f'Z{n}{L}{v}')
        for cmd in batch:
            self.send_cmd(cmd)

    def zones_default(self):
        self.send_cmd('ZR')
        self.root.after(300, self.pull_zones)

    # ================= Вкладка Ток =================
    # Пороги тока, замер холостого и защиты вынесены в ОТДЕЛЬНУЮ вкладку:
    # в «Железе» они с шунтом/UV не влезали в 180 px содержимого и уходили
    # за край окна. Здесь компактная сетка, всё видно.
    def build_current(self, tab):
        # колонки: 0 — подписи, 1 — поле, 2 — подпись-отсчёты, 3 — отступ
        tab.columnconfigure(3, weight=1)

        tk.Label(tab, text='Предел тока, А (CL):', font=('Arial', 10, 'bold')) \
            .grid(row=0, column=0, columnspan=2, sticky='w', padx=10, pady=(8, 2))
        self.var_cl = tk.StringVar(value='1.0')
        self.e_cl = tk.Entry(tab, textvariable=self.var_cl, width=10)
        self.e_cl.grid(row=1, column=0, padx=10)
        self.lbl_cl_adc = tk.Label(tab, text='', fg='#6b7280')
        self.lbl_cl_adc.grid(row=1, column=1, sticky='w')
        self.e_cl.bind('<Return>', lambda ev: self.send_cl_a())

        tk.Label(tab, text='Порог не ниже, А (CN):', font=('Arial', 10, 'bold')) \
            .grid(row=2, column=0, columnspan=2, sticky='w', padx=10, pady=(10, 2))
        self.var_cn = tk.StringVar(value='0.6')
        self.e_cn = tk.Entry(tab, textvariable=self.var_cn, width=10)
        self.e_cn.grid(row=3, column=0, padx=10)
        self.lbl_cn_adc = tk.Label(tab, text='', fg='#6b7280')
        self.lbl_cn_adc.grid(row=3, column=1, sticky='w')
        self.e_cn.bind('<Return>', lambda ev: self.send_cn_a())

        tk.Button(tab, text='Замерить холостой ход', command=self.measure_idle,
                  bg='#7c3aed', fg='white', padx=10, pady=3).grid(row=4, column=0, sticky='w', padx=10, pady=4)

        self.lbl_rc = tk.Label(tab, text='', fg='#eab308', font=('Consolas', 9), justify='left')
        self.lbl_rc.grid(row=5, column=0, columnspan=2, sticky='w', padx=10)

        # Защиты — в правую колонку, чтобы не растягивать по вертикали
        tk.Label(tab, text='Защиты:', font=('Arial', 10, 'bold')) \
            .grid(row=0, column=3, sticky='w', padx=10, pady=(8, 2))
        self.var_te = tk.IntVar(value=0)
        self.var_me = tk.IntVar(value=0)
        self.var_tl = tk.IntVar(value=1)
        tk.Checkbutton(tab, text='Тепловая I²t (TE)', variable=self.var_te,
                       command=lambda: self.send_cmd(f'TE{self.var_te.get()}')).grid(row=1, column=3, sticky='w', padx=10)
        tk.Checkbutton(tab, text='Пик тока (ME)', variable=self.var_me,
                       command=lambda: self.send_cmd(f'ME{self.var_me.get()}')).grid(row=2, column=3, sticky='w', padx=10)
        tk.Checkbutton(tab, text='Ограничение момента (TL)', variable=self.var_tl,
                       command=lambda: self.send_cmd(f'TL{self.var_tl.get()}')).grid(row=3, column=3, sticky='w', padx=10)

    # ================= Вкладка Тяга =================
    # Настройки тягового режима НЕ влезали в фикс. высоту 205 px вкладки «Ток»
    # (после блока CL/CN/защит), поэтому вынесены в отдельную вкладку.
    def build_torque(self, tab):
        self.var_tq = tk.IntVar(value=0)
        tk.Checkbutton(tab, text='ТЯГОВЫЙ — момент с PA3 (TQ)',
                       variable=self.var_tq, command=self.toggle_tq,
                       font=('Arial', 11, 'bold'), fg='#b45309'
                       ).grid(row=0, column=0, columnspan=2, sticky='w', padx=10, pady=(8, 0))

        self.lbl_tq = tk.Label(tab, text='Обороты стабилизируются',
                               fg='#6b7280', font=('Consolas', 9))
        self.lbl_tq.grid(row=1, column=0, columnspan=2, sticky='w', padx=10)

        self.f_tq = tk.Frame(tab)
        self.f_tq.grid(row=2, column=0, columnspan=2, sticky='w', padx=10, pady=2)

        self.tq_vars = {}
        # Поля размещены в 2 колонки, чтобы влезть в фикс. высоту вкладки
        tq_help = {
            'TC': 'TC — потолок оборотов, об/мин. Страховка от разноса в тяге: ' +
                  'ненагруженный мотор с постоянным моментом разгоняется, пока ' +
                  'не уйдёт в разнос. Ниже TC−TB ток плавно гасится, на TC он 0.',
            'TB': 'TB — полоса сброса, об/мин. За сколько оборотов ДО потолка начинать ' +
                  'гасить ток. 300 — плавно, без рывка; 0 — обрыв ток на самом TC.',
            'TZ': 'TZ — мёртвая зона ручки, отсчёты АЦП. Ниже этого значения ручка ' +
                  'PA3 даёт ток ровно 0. Потенциометр в нуле шумит ±20 отсчётов; без ' +
                  'зоны мотор будет подёргиваться на стоянке.',
            'TR': 'TR — темп набора тока, отсчётов за 40 мс. Рампа: резко бросили ручку — ' +
                  'ток нарастает ступеньками, а не скачком (защита от рывка вала). ' +
                  '15 ≈ полный диапазон за 2 с.',
            'TM': 'TM — свой предел тока в тяге, АМПЕРЫ (0 = брать CL). ' +
                  'В тяге часто хочется момент выше, чем в режиме оборотов.',
            'TF': 'TF — ток трогания, АМПЕРЫ (0 = выключено). Трение страгивания ' +
                  'съедает начало хода ручки. Если поставить TF — с места мотор ' +
                  'поедет сразу от этого значения.',
            'TS': 'TS — таймаут стоянки под током, мс. Стоять на месте с током > 1/3 ' +
                  'предела — мотор без обдува греется. Сколько разрешено «давить» ' +
                  'без вращения, потом TRIP_TQ_HOLD. 0 = выключено (отключено вовсе).',
        }
        pairs = [
            ('TC', 'Потолок, об/мин', '1500', 'TB', 'Сброс за, об/мин', '300'),
            ('TZ', 'Мёртвая зона',    '40',   'TR', 'Темп тока/40мс',   '15'),
            ('TM', 'Предел тяги, А',  '0',    'TF', 'Ток трогания, А',  '0'),
            ('TS', 'Стоянка макс, мс','0',    None, None,               None),
        ]
        self.tq_help_lbl = tk.Label(tab, text='Наведите мышь на поле — появится пояснение',
                                    fg='#6b7280', font=('Arial', 8), wraplength=560,
                                    justify='left', anchor='w')
        self.tq_help_lbl.grid(row=3, column=0, columnspan=2, sticky='w', padx=10, pady=(2, 0))
        for ri, p in enumerate(pairs):
            for col in (0, 1):
                cmd, label, dflt = p[col * 3], p[col * 3 + 1], p[col * 3 + 2]
                if cmd is None:
                    continue
                cx = col * 3  # 0 or 3
                tqh = tq_help[cmd]
                def _show(e, h=tqh):
                    self.tq_help_lbl.config(text=h, fg='#e5e7eb')
                def _clear(e):
                    self.tq_help_lbl.config(text='Наведите мышь на поле — появится пояснение',
                                            fg='#6b7280')
                lb = tk.Label(self.f_tq, text=label, font=('Arial', 8))
                lb.grid(row=ri, column=cx, sticky='w', pady=1)
                lb.bind('<Enter>', _show); lb.bind('<Leave>', _clear)
                var = tk.StringVar(value=dflt)
                self.tq_vars[cmd] = var
                e = tk.Entry(self.f_tq, textvariable=var, width=7)
                e.grid(row=ri, column=cx + 1, padx=2, pady=1)
                e.bind('<Return>', lambda ev, c=cmd: self.send_tq_par(c))
                e.bind('<Enter>', _show); e.bind('<Leave>', _clear)
                tk.Button(self.f_tq, text='→', width=2, font=('Arial', 7),
                          command=lambda c=cmd: self.send_tq_par(c)) \
                    .grid(row=ri, column=cx + 2)

    # ================= Вкладка Железо =================
    def build_hw(self, tab):
        tk.Label(tab, text='Шунт и датчик тока', font=('Arial', 10, 'bold')) \
            .grid(row=0, column=0, columnspan=2, sticky='w', padx=10, pady=4)

        tk.Label(tab, text='Сопротивление шунта, Ом:').grid(row=1, column=0, sticky='w', padx=10)
        self.e_shunt = tk.Entry(tab, width=10)
        self.e_shunt.insert(0, str(self.shunt_r))
        self.e_shunt.grid(row=1, column=1, sticky='w')
        self.e_shunt.bind('<Return>', lambda ev: self.apply_shunt())

        tk.Label(tab, text='Усиление датчика:').grid(row=2, column=0, sticky='w', padx=10)
        self.e_gain = tk.Entry(tab, width=10)
        self.e_gain.insert(0, str(self.gain))
        self.e_gain.grid(row=2, column=1, sticky='w')
        self.e_gain.bind('<Return>', lambda ev: self.apply_shunt())

        tk.Button(tab, text='Применить', command=self.apply_shunt,
                  bg='#3b82f6', fg='white').grid(row=3, column=0, padx=10, pady=4)

        tk.Label(tab, text='Напряжение сети, В (UV):', font=('Arial', 10, 'bold')) \
            .grid(row=4, column=0, columnspan=2, sticky='w', padx=10, pady=(10, 2))
        self.var_uv = tk.StringVar(value='230')
        tk.Entry(tab, textvariable=self.var_uv, width=10).grid(row=5, column=0, padx=10)
        tk.Button(tab, text='→', width=3,
                  command=lambda: self.send_cmd(f'UV{int(self.var_uv.get())}')).grid(row=5, column=1, sticky='w')

    # Кнопка сохранения во Flash вынесена в строку состояния (видна всегда).
    # Здесь она обрезалась краем контейнера вкладок 205 px.

    def apply_shunt(self):
        try:
            self.shunt_r = float(self.e_shunt.get())
            self.gain = float(self.e_gain.get())
            self.status.config(text=f'Масштаб тока: 1 A = {1.0/self.adc_to_a:.0f} отсчётов')
        except Exception:
            pass

    def send_cl_a(self):
        """Предел тока в АМПЕРАХ -> CL<отсчёты>."""
        try:
            a = float(self.var_cl.get())
        except Exception:
            return
        if a <= 0:
            return
        adc = int(round(a / self.adc_to_a))     # adc_to_a: отсчёты на 1 А
        if adc > 3000:
            adc = 3000
        self.send_cmd(f'CL{adc}')
        self.lbl_cl_adc.config(text=f'= {adc} отсч.')
        self.lbl_rc_update()

    def send_cn_a(self):
        """Порог не ниже, А -> CN<отсчёты> (абсолютный пол по току)."""
        try:
            a = float(self.var_cn.get())
        except Exception:
            return
        if a <= 0:
            return
        adc = int(round(a / self.adc_to_a))
        if adc < 5:
            adc = 5
        # CN не применится, если выше текущего предела (mc_cmd.c проверяет
        # v <= current_limit_set). Если так — предупреждаем, иначе команда
        # теряется молча и следующая телеметрия вернёт старый пол.
        cls_live = self.live.get('cls', 0)
        if cls_live and adc > cls_live:
            messagebox.showwarning('Порог',
                'Пол выше предела тока — сначала поднимите «Предел тока, А»')
            return
        self.send_cmd(f'CN{adc}')
        self.lbl_cn_adc.config(text=f'= {adc} отсч.')
        self.lbl_rc_update()

    def lbl_rc_update(self):
        """Живая подпись диапазона, который крутит ручка PA3."""
        try:
            cn_a = float(self.var_cn.get())
            cl_a = float(self.var_cl.get())
        except Exception:
            return
        self.lbl_rc.config(text=f'Ручка PA3 крутит порог от {cn_a:.2f} А до {cl_a:.2f} А')

    def measure_idle(self):
        """Замер тока холостого хода: усредняем rms за ~3 с, предлагаем
        поставить пол с запасом 20% сверху — чтобы не душить двигатель."""
        # Сбор: накапливаем rms в фоне, потом окно с результатом
        self._idle_start = time.time()
        self._idle_sum = 0.0
        self._idle_n = 0
        self.btn_idle_state = 'measuring'
        self.status.config(text='Замер холостого хода: подай обороты без нагрузки...')
        self.root.after(3500, self._idle_done)

    def _idle_tick(self):
        """Вызывается из read_loop при приёме JSON — копит rms во время замера."""
        if getattr(self, 'btn_idle_state', '') == 'measuring':
            rms_a = self.live.get('rms', 0) * self.adc_to_a
            self._idle_sum += rms_a
            self._idle_n += 1

    def _idle_done(self):
        self.btn_idle_state = ''
        if self._idle_n == 0:
            self.status.config(text='Замер не удался — не было данных')
            return
        avg = self._idle_sum / self._idle_n
        rec = avg * 1.2                       # запас 20% сверху
        messagebox.showinfo('Холостой ход',
                            f'Ток холостого хода: {avg:.2f} А\n\n'
                            f'Ставим пол с запасом 20%: {rec:.2f} А\n'
                            f'Впишите это в «Порог не ниже» и нажмите Enter.')
        self.var_cn.set(f'{rec:.2f}')
        self.send_cn_a()

    def do_save(self):
        if self.live.get('on', 0):
            messagebox.showwarning('Сохранение',
                                   'Останови двигатель (ВЫКЛ) — при работе Flash не пишется')
            return
        self.send_cmd('w')
        self.dirty = False
        self.btn_save.config(text='Сохранить во Flash', bg='#16a34a')
        self.status.config(text='Сохранено во Flash')

    # ================= Вкладка Лог/Тест =================
    def build_log(self, tab):
        tk.Label(tab, text='Запись в CSV', font=('Arial', 10, 'bold')) \
            .grid(row=0, column=0, columnspan=2, sticky='w', padx=10, pady=4)
        self.btn_log = tk.Button(tab, text='Начать запись', command=self.toggle_log,
                                  width=14, bg='#dc2626', fg='white')
        self.btn_log.grid(row=1, column=0, padx=10, pady=2)
        self.lbl_log = tk.Label(tab, text='', fg='#6b7280')
        self.lbl_log.grid(row=1, column=1, sticky='w')

        tk.Label(tab, text='Тест скачка уставки', font=('Arial', 10, 'bold')) \
            .grid(row=2, column=0, columnspan=2, sticky='w', padx=10, pady=(10, 4))
        self.var_step_from = tk.DoubleVar(value=500)
        self.slider_row(tab, 3, 'Скачок с:', self.var_step_from,
                        RPM_MIN, RPM_MAX, lambda: None, 200)
        self.var_step_to = tk.DoubleVar(value=1500)
        self.slider_row(tab, 4, 'до:', self.var_step_to,
                        RPM_MIN, RPM_MAX, lambda: None, 200)

        self.btn_step = tk.Button(tab, text='Выполнить скачок', command=self.do_step_test,
                                   bg='#7c3aed', fg='white', padx=12, pady=3)
        self.btn_step.grid(row=5, column=0, padx=10, pady=4)
        self.lbl_step = tk.Label(tab, text='', fg='#a855f7', font=('Arial', 9))
        self.lbl_step.grid(row=5, column=1, columnspan=3, sticky='w', padx=6)

    def toggle_log(self):
        if self.logging:
            self.logging = False
            if self.csv_file:
                self.csv_file.close()
                self.csv_file = None
            self.btn_log.config(text='Начать запись', bg='#dc2626')
            self.lbl_log.config(text='')
            return
        fname = f'log_{datetime.datetime.now():%Y%m%d_%H%M%S}.csv'
        try:
            self.csv_file = open(fname, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['t', 'rpm', 'rpm_set', 'cur', 'duty', 'iref',
                                      'esp', 'eduty', 'tq', 'cikp', 'ciki', 'bst',
                                      'eg', 'eok', 'fr', 'fd', 'sl', 'pwr',
                                      'trip', 'limit', 'on', 'auto',
                                      'kp', 'ki', 'kd', 'rlo', 'rhi'])
            self.logging = True
            self.btn_log.config(text='Стоп запись', bg='#16a34a')
            self.lbl_log.config(text=f'→ {fname}')
        except Exception as e:
            self.lbl_log.config(text=f'Ошибка: {e}', fg='#ef4444')

    def do_step_test(self):
        v_from = int(round(self.var_step_from.get()))
        v_to = int(round(self.var_step_to.get()))
        self.send_cmd(f'RV{v_from}')
        self.btn_step.config(state=tk.DISABLED)

        def tick(n):
            if n > 0:
                self.lbl_step.config(text=f'Выход на {v_from} об/мин... {n}')
                self.root.after(1000, tick, n - 1)
                return
            self.send_cmd(f'RV{v_to}')
            self.lbl_step.config(text=f'Скачок {v_from} → {v_to} — замер...')
            # Старт от ФАКТИЧЕСКИХ оборотов, а не от заданных v_from:
            # если мотор за 3 с не вышел на уставку (тяжёлый вал, малый Ki,
            # или он вообще выключен), направление и пик считаются от
            # реального положения, иначе получится фантомное перерегулирование.
            with self.lock:
                self.step_from = self.rpm_smooth
            self.step_peak = self.rpm_smooth
            self.step_to = v_to
            self.step_t0 = time.time()

        tick(3)

    # ================= Вкладка Терминал =================
    CMD_HELP = [
        ('— Мотор —', ''),
        ('ON',        'пуск'),
        ('OF',        'стоп'),
        ('HB',        'свободный выбег'),
        ('RS',        'сброс защиты (после срыва)'),
        ('CF',        'полный сброс настроек к заводским'),
        ('RP',        'обороты с потенциометра'),
        ('RV<n>',     'обороты с ПК, напр. RV500'),
        ('— Режим защиты —', ''),
        ('0',         'рабочий'),
        ('1',         'отладка'),
        ('2',         'тест: защиты сняты'),
        ('3',         'тяговый'),
        ('— PID скорости —', ''),
        ('KP<n>',     'пропорциональный, 0..5000'),
        ('KI<n>',     'интегральный'),
        ('KD<n>',     'дифференциальный'),
        ('a / m',     'автоподстройка по зонам / ручной'),
        ('r / s',     'подавление пульсаций вкл / выкл'),
        ('— PID тока —', ''),
        ('CP<n>',     'пропорциональный тока'),
        ('CI<n>',     'интегральный тока'),
        ('— Токовые пределы —', ''),
        ('CL<n>',     'предел тока'),
        ('CN<n>',     'нижний абсолютный предел'),
        ('P<n>',      'пауза после защиты, мс (100..30000)'),
        ('TE0 / TE1', 'тепловая защита выкл / вкл'),
        ('ME0 / ME1', 'максимальная защита выкл / вкл'),
        ('TL0 / TL1', 'ограничение момента выкл / вкл'),
        ('BT<n>',     'время форсажа, мс (0=выкл, 100..5000)'),
        ('— Разгон —', ''),
        ('SS<n>',     'шаг плавного пуска'),
        ('RU<n>',     'шаг разгона'),
        ('RD<n>',     'шаг торможения'),
        ('— Фильтры —', ''),
        ('FR<n>',     'фильтр оборотов, 1..4'),
        ('FD<n>',     'фильтр D-составляющей, 1..4'),
        ('SL<n>',     'делитель контура скорости, 1..8'),
        ('— Разомкнутый —', ''),
        ('OL0 / OL1', 'режим выкл / вкл'),
        ('OP<n>',     'скважность вручную'),
        ('OT0 / OT1', 'скважность с потенциометра'),
        ('— Тяга —', ''),
        ('TQ0 / TQ1', 'тяговый режим выкл / вкл'),
        ('TC<n>',     'потолок оборотов'),
        ('TB<n>',     'полоса'),
        ('TZ<n>',     'зона нечувствительности'),
        ('TR<n>',     'скорость нарастания'),
        ('TM<n>',     'предел тока тяги'),
        ('TF<n>',     'нижний порог'),
        ('TS<n>',     'удержание, мс'),
        ('— Дисплей —', ''),
        ('DD',        'показать раскладку'),
        ('DR',        'заводская раскладка'),
        ('DP<n>',     'страница слотов дисплея 0..2'),
        ('D<n>F<v>',  'поле слота n'),
        ('D<n>R<v>',  'строка сетки'),
        ('D<n>C<v>',  'колонка сетки'),
        ('D<n>W<v>',  'ширина в колонках'),
        ('D<n>Z<v>',  'размер шрифта 1..3'),
        ('D<n>A<v>',  'выравнивание 0/1/2'),
        ('D<n>P<v>',  'знаков после запятой'),
        ('D<n>K<v>',  'цвет 0..7'),
        ('D<n>E<v>',  'включён 0/1'),
        ('D<n>L<v>',  'подписи: бит0 имя, бит1 единицы'),
        ('— Зоны PID —', ''),
        ('ZD',        'показать таблицу зон'),
        ('ZR',        'заводские зоны'),
        ('Z<n>P<v>',  'Kp зоны n (0..5)'),
        ('Z<n>I<v>',  'Ki зоны'),
        ('Z<n>D<v>',  'Kd зоны'),
        ('Z<n>R<v>',  'обороты узла зоны'),
        ('— Прочее —', ''),
        ('UV<n>',     'напряжение сети'),
        ('w',         'сохранить всё во Flash'),
        ('j / o',     'телеметрия JSON вкл / выкл'),
    ]

    def build_term(self, tab):
        left = tk.Frame(tab, bg='#101418')
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.term_out = tk.Text(left, bg='#0b0f14', fg='#c8e6c9',
                                font=('Consolas', 9), height=8,
                                insertbackground='#c8e6c9')
        self.term_out.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)

        ent = tk.Frame(left, bg='#101418'); ent.pack(fill=tk.X)
        self.term_in = tk.Entry(ent, bg='#1a202c', fg='#fff',
                                font=('Consolas', 10),
                                insertbackground='#fff')
        self.term_in.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2, pady=2)
        self.term_in.bind('<Return>', self.term_send)
        # История команд стрелками — без неё подбор вслепую утомляет
        self.term_hist, self.term_hi = [], 0
        self.term_in.bind('<Up>',   lambda e: self.term_nav(-1))
        self.term_in.bind('<Down>', lambda e: self.term_nav(1))

        tk.Button(ent, text='Отправить', command=self.term_send,
                  bg='#2563eb', fg='#fff').pack(side=tk.LEFT, padx=2)
        self.var_echo = tk.IntVar(value=0)
        tk.Checkbutton(ent, text='поток', variable=self.var_echo,
                       command=lambda: setattr(self, 'term_echo',
                                               bool(self.var_echo.get())),
                       bg='#101418', fg='#9aa5b1',
                       selectcolor='#101418').pack(side=tk.LEFT)
        tk.Button(ent, text='Очистить',
                  command=lambda: self.term_out.delete('1.0', tk.END),
                  bg='#374151', fg='#fff').pack(side=tk.LEFT, padx=2)

        # Справочник справа: двойной клик подставляет команду в строку ввода
        right = tk.Frame(tab, bg='#101418')
        right.pack(side=tk.RIGHT, fill=tk.Y)
        sb = tk.Scrollbar(right); sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.term_list = tk.Listbox(right, width=58, bg='#0b0f14',
                                    fg='#9aa5b1', font=('Consolas', 11),
                                    yscrollcommand=sb.set,
                                    selectbackground='#2563eb')
        self.term_list.pack(side=tk.LEFT, fill=tk.Y)
        sb.config(command=self.term_list.yview)
        for cmd, desc in self.CMD_HELP:
            self.term_list.insert(tk.END,
                                  cmd if not desc else f'{cmd:<16} {desc}')
        self.term_list.bind('<Double-Button-1>', self.term_pick)

        self.term_poll()

    def term_pick(self, _e=None):
        sel = self.term_list.curselection()
        if not sel:
            return
        cmd = self.CMD_HELP[sel[0]][0]
        if cmd.startswith('—'):
            return
        # Из «TE0 / TE1» берём первый вариант, шаблоны <n> оставляем как есть
        cmd = cmd.split(' / ')[0]
        self.term_in.delete(0, tk.END)
        self.term_in.insert(0, cmd)
        self.term_in.focus_set()

    def term_nav(self, d):
        if not self.term_hist:
            return
        self.term_hi = max(0, min(len(self.term_hist) - 1, self.term_hi + d))
        self.term_in.delete(0, tk.END)
        self.term_in.insert(0, self.term_hist[self.term_hi])

    def term_send(self, _e=None):
        cmd = self.term_in.get().strip()
        if not cmd:
            return
        self.send_cmd(cmd)
        self.term_out.insert(tk.END, f'> {cmd}\n')
        self.term_out.see(tk.END)
        self.term_hist.append(cmd)
        self.term_hi = len(self.term_hist)
        self.term_in.delete(0, tk.END)

    def term_poll(self):
        if self.term_buf:
            with self.lock:
                lines, self.term_buf = self.term_buf, []
            for ln in lines:
                self.term_out.insert(tk.END, ln + '\n')
            self.term_out.see(tk.END)
        self.root.after(120, self.term_poll)

    # ================= Вкладка Задатчик =================
    def build_setpoint(self, tab):
        self.var_rpm_virt = tk.DoubleVar(value=RPM_MIN)
        self.var_rpm_virt._drag = False
        self.var_rpm_virt._touched = 0.0
        self.var_rpm_virt._busy = False

        left = tk.Frame(tab, bg='#101418')
        left.grid(row=0, column=0, padx=12, pady=6)
        self.lbl_virt_hdr = tk.Label(left, text='Виртуальный задатчик, об/мин',
                                     bg='#101418', fg='#eab308')
        self.lbl_virt_hdr.pack()
        self.knob = Knob(left, RPM_MIN, RPM_MAX, RPM_MIN, self._knob_change, size=140)
        self.knob.pack(pady=4)

        right = tk.Frame(tab, bg='#101418')
        right.grid(row=0, column=1, padx=12, pady=6)
        self.lbl_pot_hdr = tk.Label(right, text='Реальный резистор, об/мин',
                                    bg='#101418', fg='#6b7280')
        self.lbl_pot_hdr.pack()
        self.knob_pot = Knob(right, RPM_MIN, RPM_MAX, RPM_MIN, lambda v: None, size=140)
        self.knob_pot.unbind('<Button-1>')
        self.knob_pot.unbind('<B1-Motion>')
        self.knob_pot.unbind('<ButtonRelease-1>')
        self.knob_pot.unbind('<MouseWheel>')
        self.knob_pot.unbind('<Button-4>')
        self.knob_pot.unbind('<Button-5>')
        self.knob_pot.pack(pady=4)

        btns = tk.Frame(tab)
        btns.grid(row=1, column=0, columnspan=2, pady=6)
        # Отдельные имена: btn_pot/btn_virt уже заняты кнопками верхней
        # панели (рядом с «СБРОС»). Без своих имён эти кнопки не подсвечивались.
        self.btn_sp_virt = tk.Button(btns, text='Вирт. (RV)', width=10,
                                     bg='#16a34a', fg='white', command=self.use_virt)
        self.btn_sp_virt.pack(side=tk.LEFT, padx=4)
        self.btn_sp_pot = tk.Button(btns, text='Реал. (RP)', width=10,
                                    bg='#6b7280', fg='white', command=self.use_pot)
        self.btn_sp_pot.pack(side=tk.LEFT, padx=4)

        # быстрые уставки — строим от РАБОЧЕГО диапазона, а не жёстким списком
        # до 6000: при rhi=2081 кнопка «6000» молча зажималась до 2081,
        # и человек не понимал, сломалось или нет. Кнопки перестраиваются
        # в draw при смене границ.
        self.preset_btns = []
        for _ in range(6):
            b = tk.Button(btns, text='—', width=5)
            b.pack(side=tk.LEFT, padx=1)
            self.preset_btns.append(b)
        self._preset_range = (None, None)

        self.lbl_sp = tk.Label(tab, text='Кнопки и крутилка слева задают обороты с ПК. '
                                         'Реальный резистор — ручка на панели, с ПК не задаётся: '
                                         'покрутите её, и управление перейдёт к ней само',
                               font=('Consolas', 9), fg='#3b82f6')
        self.lbl_sp.grid(row=2, column=0, columnspan=2)

    def _knob_change(self, v):
        self.var_rpm_virt._touched = time.time()
        self.var_rpm_virt.set(v)
        # НЕ ставим sp_src='virt' здесь: случайное колесо мыши над крутилкой
        # не должно молча выключать реальный резистор. Если источник уже
        # виртуальный — send_rpm_virt уйдёт, иначе событие проигнорируется.
        if getattr(self, 'sp_src', 'virt') == 'virt':
            self.schedule_rv()

    def _preset(self, v):
        self.sp_src = 'virt'           # явный выбор ПК-задания
        self.knob.set_value(v)
        self.var_rpm_virt.set(v)
        self.send_rpm_virt()

    def send_rpm_virt(self):
        if getattr(self, 'sp_src', 'virt') != 'virt':
            return
        # Зажимаем по РАБОЧЕМУ диапазону, а не по потолку железа: иначе
        # прошивка молча отвергнет команду вне RL..RH и человек не поймёт,
        # почему обороты не меняются.
        # Берём с самой ручки, а не из var_rpm_virt: переменную может
        # затереть кадр телеметрии, а положение ручки — это ровно то,
        # что человек выставил и видит на экране.
        lo = int(round(self.var_rlo.get()))
        hi = int(round(self.var_rhi.get()))
        v = int(max(lo, min(hi, self.knob.value)))
        self.send_cmd(f'RV{v}')       # RV сам взводит rpm_virt_sw=1 в прошивке

    # ================= Чтение =================
    def read_loop(self):
        while self.running:
            try:
                n = self.ser.in_waiting
                if n == 0:
                    time.sleep(0.005)
                    continue
                data = self.ser.read(n)
                # Терминал: копим всё сырьё, JSON-кадры прячем по умолчанию —
                # иначе телеметрия зальёт окно и ответов команд не найти
                try:
                    for ln in data.decode('utf-8', errors='ignore').splitlines():
                        ln = ln.strip()
                        if not ln:
                            continue
                        if ln.startswith('{') and not self.term_echo:
                            continue
                        self.term_buf.append(ln)
                    if len(self.term_buf) > 500:
                        del self.term_buf[:-400]
                except Exception:
                    pass
                with self.lock:
                    self.raw += data
                    while True:
                        m = re.search(rb'\{[^{}]+\}', self.raw)
                        if not m:
                            break
                        chunk = m.group(0)
                        self.raw = self.raw[m.end():]
                        try:
                            o = json.loads(chunk.decode('utf-8', errors='ignore'))
                            if 'zones' in o:
                                # это дамп таблицы зон, НЕ телеметрия: в live не кладём,
                                # иначе на один тик live станет пустым, do_save разрешит
                                # запись Flash на работающем моторе
                                self.zones = o['zones']
                                continue
                            self.live = dict(o)
                            self.last_frame = time.time()
                            self._idle_tick()
                            self.rpm_smooth += (float(o.get('rpm', 0)) - self.rpm_smooth) * 0.15
                            self.motor_v_smooth += (float(o.get('mv', 0)) - self.motor_v_smooth) * 0.15
                            r = float(o.get('rpm', 0))
                            c = float(o.get('cur', 0)) * self.adc_to_a
                            self.rpm_hist.append(r); self.rpm_hist.pop(0)
                            self.cur_hist.append(c); self.cur_hist.pop(0)
                            if self.logging and self.csv_writer:
                                self.csv_writer.writerow([
                                    time.time(), r, o.get('rpm_set', 0),
                                    c, o.get('duty', 0), o.get('iref', 0), o.get('esp', 0),
                                    o.get('eduty', 0), o.get('tq', 0),
                                    o.get('cikp', 0), o.get('ciki', 0), o.get('bst', 0),
                                    o.get('eg', 0), o.get('eok', 0),
                                    o.get('fr', 0), o.get('fd', 0), o.get('sl', 0),
                                    round(self.power_smooth, 1),
                                    o.get('trip', 0), o.get('limit', 0),
                                    o.get('on', 0), o.get('auto', 0),
                                    o.get('kp', 0), o.get('ki', 0), o.get('kd', 0),
                                    o.get('rlo', 0), o.get('rhi', 0)
                                ])
                        except Exception:
                            pass
                        if len(self.raw) > 4096:
                            self.raw = self.raw[-2048:]
            except Exception:
                pass

    # ================= Отрисовка =================
    def draw(self):
        if not self.running:
            return
        # контроль связи: если кадров нет 2 секунды — плашка, не рисуем мёртвые данные
        if self.last_frame and time.time() - self.last_frame > 2.0:
            if self.step_t0:
                self.step_t0 = 0.0
                self.btn_step.config(state=tk.NORMAL)
                self.lbl_step.config(text='Замер прерван — нет связи')
            self.status.config(fg='#ef4444',
                               text='  СВЯЗЬ ПОТЕРЯНА — проверь кабель USB')
            self.root.after(200, self.draw)
            return
        try:
            with self.lock:
                rpm_h = list(self.rpm_hist)
                cur_h = list(self.cur_hist)
                live = dict(self.live)
                rpm = self.rpm_smooth
                rset = float(live.get('rpm_set', 0))
                cur = float(live.get('cur', 0)) * self.adc_to_a
                rms = float(live.get('rms', 0)) * self.adc_to_a
                duty = int(live.get('duty', 0))
                iref = int(live.get('iref', 0))
                esp = int(live.get('esp', 0))
                ovr = int(live.get('ovr', 0))
                pot = int(live.get('pot', 0))
                on = int(live.get('on', 0))
                on_sw = int(live.get('on_sw', 0))
                trip = int(live.get('trip', 0))
                limit = int(live.get('limit', 0))
                pid_auto = int(live.get('auto', 0))
                pid_rip = int(live.get('rip', 0))
                prot = int(live.get('prot', 0))
                ol = int(live.get('ol', 0))
                olduty = int(live.get('olduty', 0))
                kp_fw = int(live.get('kp', 0))
                ki_fw = int(live.get('ki', 0))
                kd_fw = int(live.get('kd', 0))
                # kpu/kiu/kdu — ЗАДАНИЕ (kp_user в прошивке).
                # kp/ki/kd — ДЕЙСТВУЮЩИЕ, с множителем ripple.
                # Ползунки синхронизируем по заданию, иначе они ездят
                # за автоматом и управлять ими невозможно.
                kpu_fw = int(live.get('kpu', kp_fw))
                kiu_fw = int(live.get('kiu', ki_fw))
                kdu_fw = int(live.get('kdu', kd_fw))
                kpm_fw = int(live.get('kpm', 1000))
                kdm_fw = int(live.get('kdm', 1000))
                cikp_fw = int(live.get('cikp', 0))
                ciki_fw = int(live.get('ciki', 0))
                climit = float(live.get('cl', 763)) * self.adc_to_a
                ss = int(live.get('ss', DEFAULT_SS))
                ru = int(live.get('ru', DEFAULT_RU))
                rd = int(live.get('rd', DEFAULT_RD))
                thermal = int(live.get('te', 0))
                maxim = int(live.get('me', 0))
                rpm_virt_sw = int(live.get('vs', 0))
                rpm_virt = int(live.get('vr', RPM_MIN))
                torq = int(live.get('tl', 1))
                tq = int(live.get('tq', 0))
                bst = int(live.get('bst', 0))
                bt  = int(live.get('bt', 0))
                eg  = int(live.get('eg', 0))
                eok = int(live.get('eok', 0))
                tq_par = {'TC': live.get('tcap', 1500), 'TB': live.get('tband', 300),
                          'TZ': live.get('tdz', 40),    'TR': live.get('trate', 15),
                          'TS': live.get('thold', 0)}
                timax = int(live.get('timax', 0))
                tfloor = int(live.get('tfloor', 0))
                tq_par['TM'] = f'{timax * self.adc_to_a:.2f}' if timax else '0'
                tq_par['TF'] = f'{tfloor * self.adc_to_a:.2f}' if tfloor else '0'
                mv = int(live.get('mv', 0))
                uv = int(live.get('uv', 230))
                json_mode = int(live.get('json', 1))
                # --- Потребляемая мощность ---
                # Датчика напряжения нет. UV — сетевое ПЕРЕМЕННОЕ (230 В, 50 Гц).
                # После моста с конденсатором на шине амплитудное: 230*sqrt(2) ≈ 325 В,
                # минус ~1.4 В на диодах. ШИМ режет уже ПОСТОЯНКУ, поэтому
                # среднее на двигателе = Uшины * d, без корня.
                if uv > 50:
                    u_dc = uv * 1.414 - 1.4
                else:
                    u_dc = 325.0
                d_frac = (olduty if ol else duty) / float(PWM_MAX)
                d_frac = min(max(d_frac, 0.0), 1.0)
                u_mot = u_dc * d_frac          # среднее на якоре

                # Мощность по шине: ток шины = ток якоря × скважность
                # (якорь непрерывный, обратный диод замыкает его в закрытой
                # фазе). P = Uшины × Iшины.
                i_bus = rms * d_frac
                self.i_bus_live = i_bus
                self.power_w = u_dc * i_bus
                self.power_smooth += (self.power_w - self.power_smooth) * 0.15
                self.u_dc_live = u_dc
                self.u_mot_live = u_mot

            # --- Подсветка кнопок ---
            ON = '#16a34a'; OFF = '#4b5563'
            self.btn_on.config(bg=ON if on_sw == 1 else OFF)
            self.btn_of.config(bg=ON if on_sw == 2 else OFF)
            self.btn_hb.config(bg=ON if on_sw == 0 else OFF)
            self.btn_p0.config(bg=ON if prot == 0 else OFF)
            self.btn_p1.config(bg=ON if prot == 1 else OFF)
            self.btn_p2.config(bg=ON if prot == 2 else OFF)
            self.btn_p3.config(bg=ON if prot == 3 else OFF)
            self.btn_pa.config(bg=ON if pid_auto else OFF)
            self.btn_pm.config(bg=ON if not pid_auto else OFF)
            self.btn_pr.config(bg=ON if pid_rip else OFF)
            self.btn_ps.config(bg=ON if not pid_rip else OFF)
            self.btn_j.config(bg=ON if json_mode else OFF)
            self.btn_o.config(bg=ON if not json_mode else OFF)
            # Источник берём из телеметрии (vs), а не из sp_src:
            # показываем то, что реально стоит в прошивке.
            self.btn_virt.config(bg=ON if rpm_virt_sw else OFF)
            self.btn_pot.config(bg=OFF if rpm_virt_sw else ON)

            # Индикатор двигателя: 'on' — фактическое состояние (mc_inverter_on),
            # 'on_sw' — только положение команды (может опережать реальность).
            if on:
                self.lbl_motor.config(text=' ДВИГ ВКЛ ', bg='#dc2626', fg='white')
            elif on_sw:
                self.lbl_motor.config(text=' ЗАПРЕТ ', bg='#f59e0b', fg='#111827')
            else:
                self.lbl_motor.config(text=' ДВИГ ВЫКЛ ', bg='#1f2937', fg='#60a5fa')

            # --- Тяговый режим: состояние берём из прошивки, а не из галочки:
            # режим мог включиться ручкой дисплея или остаться после перезапуска
            if tq != self.var_tq.get():
                self.var_tq.set(tq)
                self.lbl_tq.config(
                    text='ТЯГА: обороты свободны!' if tq else 'Обороты стабилизируются',
                    fg='#f59e0b' if tq else '#6b7280')
            for c, v in tq_par.items():          # поля не перетираем при вводе
                w = self.root.focus_get()
                if isinstance(w, tk.Entry):
                    continue
                cur_s = self.tq_vars[c].get()
                if c in ('TM', 'TF'):
                    # числовое сравнение: ввели «1.5», вернулось «1.50» — это одно
                    try:
                        if abs(float(cur_s) - float(v)) < 0.01:
                            continue
                    except Exception:
                        pass
                elif cur_s == str(v):
                    continue
                self.tq_vars[c].set(str(v))

            # --- Индикация авто/ручной и расхождение с прошивкой ---
            mkp = (kpu_fw != int(round(self.var_kp.get())))
            mki = (kiu_fw != int(round(self.var_ki.get())))
            mkd = (kdu_fw != int(round(self.var_kd.get())))
            self.lbl_kp_fw.config(text=f'Kp {kpu_fw}→{kp_fw}',
                                  fg='#ef4444' if mkp else '#22c55e')
            self.lbl_ki_fw.config(text=f'Ki {kiu_fw}→{ki_fw}',
                                  fg='#ef4444' if mki else '#22c55e')
            self.lbl_kd_fw.config(text=f'Kd {kdu_fw}→{kd_fw}',
                                  fg='#ef4444' if mkd else '#22c55e')
            if hasattr(self, 'lbl_mul'):
                self.lbl_mul.config(
                    text=f'ripple: ×{kpm_fw/1000.0:.2f} Kp  ×{kdm_fw/1000.0:.2f} Kd',
                    fg='#eab308' if (kpm_fw != 1000 or kdm_fw != 1000) else '#6b7280')
            mode_text = 'АВТО (зоны)' if pid_auto else 'РУЧНОЙ'
            mode_col = '#22c55e' if pid_auto else '#eab308'
            if on:
                hint = '  (в работе показаны действующие коэффициенты, с форсажем)'
            elif mkp or mki or mkd:
                hint = '  (задание расходится с прошивкой)'
            else:
                hint = ''
            self.lbl_mode.config(text=f'Режим: {mode_text}{hint}', fg=mode_col)

            # Синхронизация ползунков по ЗАДАНИЮ (kpu/kiu/kdu). Оно от
            # оборотов не зависит, поэтому ограничения not on/not bst не нужны.
            pid_vars = (self.var_kp, self.var_ki, self.var_kd)
            if not pid_auto and self._pid_job is None and self.group_quiet(pid_vars):
                if mkp: self.sync_var(self.var_kp, kpu_fw)
                if mki: self.sync_var(self.var_ki, kiu_fw)
                if mkd: self.sync_var(self.var_kd, kdu_fw)
            self._prev_auto = pid_auto

            # То же для ПИД тока: значения читаем, метки обновляем
            mci = (cikp_fw != int(round(self.var_cikp.get())) or
                   ciki_fw != int(round(self.var_ciki.get())))
            if hasattr(self, 'lbl_ci_fw'):
                self.lbl_ci_fw.config(text=f'ток: CP {cikp_fw} CI {ciki_fw}',
                                      fg='#ef4444' if mci else '#22c55e')
            ci_vars = (self.var_cikp, self.var_ciki)
            if self._ci_job is None and self.group_quiet(ci_vars):
                if cikp_fw != int(round(self.var_cikp.get())):
                    self.sync_var(self.var_cikp, cikp_fw)
                if ciki_fw != int(round(self.var_ciki.get())):
                    self.sync_var(self.var_ciki, ciki_fw)

            # --- Форсаж: состояние и время из телеметрии ---
            if bst:
                self.lbl_boost.config(text='Форсаж: АКТИВЕН  ×1.6  (%d мс)' % bt,
                                      fg='#f97316')
            else:
                self.lbl_boost.config(text='Форсаж: ждёт  (%d мс)' % bt,
                                      fg='#9ca3af')

            # --- Синхронизация ползунков ---
            # Не трогаем, пока стоит отложенная отправка (_ramp_job) — иначе
            # телеметрия сбрасывает ползунок под курсором быстрее, чем уходит
            # команда, и он выглядит «неактивным».
            if not self._ramp_job:
                self.sync_group([(self.var_ss, ss), (self.var_ru, ru),
                                 (self.var_rd, rd)], self._ramp_job, hold=0.6)

            # Фильтры FR/FD/SL — прошивка отдаёт их в телеметрии (fr/fd/sl)
            self.sync_group([(self.var_fr, live.get('fr')),
                             (self.var_fd, live.get('fd')),
                             (self.var_sl, live.get('sl'))], self._filt_job, hold=0.6)

            # Рабочий диапазон оборотов: подтягиваем ползунки границ
            self.sync_group([(self.var_rlo, live.get('rlo')),
                             (self.var_rhi, live.get('rhi'))],
                            self._rlim_job, hold=0.6)
            rlo = int(live.get('rlo', RPM_MIN))
            rhi = int(live.get('rhi', RPM_MAX))
            if hasattr(self, 'lbl_rlim'):
                self.lbl_rlim.config(
                    text=f'диапазон: {rlo}…{rhi} об/мин  '
                         f'(полный ход ручки = {rhi} об/мин)')
            # Шкалу ручки растягиваем под рабочий диапазон: иначе
            # верхняя часть хода остаётся мёртвой — send_rpm_virt
            # всё равно зажмёт значение по rhi.
            if self.knob.vmax != rhi or self.knob.vmin != rlo:
                self.knob.vmin, self.knob.vmax = rlo, rhi
                self.knob_pot.vmin, self.knob_pot.vmax = rlo, rhi
                self.knob.redraw(); self.knob_pot.redraw()

            # --- Синхронизация CL/CN из прошивки (в амперах) ---
            if hasattr(self, 'var_cl'):
                cls_live = live.get('cls', None)
                if cls_live:
                    self.lbl_cl_adc.config(text=f'= {int(cls_live)} отсч.')
                    # Не трогаем поле, пока в нём фокус (пользователь набирает).
                    # Сравнение в отсчётах: cl_a[А]/adc_to_a[А/отсч] = отсч.
                    if self.root.focus_get() is not self.e_cl:
                        try:
                            cl_a = float(self.var_cl.get())
                            if abs(cl_a / self.adc_to_a - int(cls_live)) > 1:
                                self.var_cl.set(f'{int(cls_live) * self.adc_to_a:.2f}')
                        except Exception:
                            pass
                cn_live = live.get('cn', None)
                if cn_live:
                    self.lbl_cn_adc.config(text=f'= {int(cn_live)} отсч.')
                    if self.root.focus_get() is not self.e_cn:
                        try:
                            cn_a = float(self.var_cn.get())
                            if abs(cn_a / self.adc_to_a - int(cn_live)) > 1:
                                self.var_cn.set(f'{int(cn_live) * self.adc_to_a:.2f}')
                        except Exception:
                            pass
                self.lbl_rc_update()

            # --- Синхронизация крутилок ---
            # Пересчёт строго по той же формуле, что в прошивке (mc.c:1272):
            # rlo + pot*(rhi-rlo)/4095. По RPM_MIN/RPM_MAX считать нельзя —
            # шкала разойдётся с реальной уставкой, как только рабочий
            # диапазон отличается от границ железа.
            self.knob_pot.set_value(rlo + pot * (rhi - rlo) / 4095.0)
            # Авто-переключение на реальный резистор: если ПК-режим активен,
            # а ручка сместилась от опорной точки больше чем на 120 отсчётов —
            # отдаём управление резистору. Обнуляем якорь при движении, чтобы
            # переключение происходило от свежей позиции, а не от дрейфующей.
            if rpm_virt_sw:
                if getattr(self, '_pot_anchor', None) is None:
                    self._pot_anchor = pot
                elif abs(pot - self._pot_anchor) > 120:
                    self._pot_anchor = None
                    self.use_pot()
            else:
                self._pot_anchor = None
            # Подтягиваем ручку из прошивки ТОЛЬКО когда человек её не трогает
            # и нет висящей отправки. Иначе кадр телеметрии (40 мс) успевает
            # затереть свежее значение до того, как schedule_rv отправит его
            # через 120 мс — и уходит старая уставка вместо новой.
            if (rpm_virt_sw
                    and self._rv_job is None
                    and not self.knob._dragging
                    and time.time() - self.var_rpm_virt._touched > 0.6):
                self.sync_var(self.var_rpm_virt, rpm_virt)
                self.knob.set_value(rpm_virt)

            # Подсветка активного задатчика по ТЕЛЕМЕТРИИ, а не по sp_src:
            # прошивка — источник истины, она может уйти на резистор сама
            # (автоподхват по движению ручки), и sp_src об этом не знает.
            if rpm_virt_sw:
                self.lbl_virt_hdr.config(fg='#eab308')
                self.lbl_pot_hdr.config(fg='#4b5563')
                self.btn_sp_virt.config(bg='#16a34a', relief=tk.SUNKEN)
                self.btn_sp_pot.config(bg='#374151', relief=tk.RAISED)
            else:
                self.lbl_virt_hdr.config(fg='#4b5563')
                self.lbl_pot_hdr.config(fg='#eab308')
                self.btn_sp_virt.config(bg='#374151', relief=tk.RAISED)
                self.btn_sp_pot.config(bg='#16a34a', relief=tk.SUNKEN)

            # Перестраиваем пресеты только при смене границ, а не каждый кадр
            if self._preset_range != (rlo, rhi):
                self._preset_range = (rlo, rhi)
                for i, b in enumerate(self.preset_btns):
                    v = int(round(rlo + (rhi - rlo) * (i + 1) / 6.0))
                    b.config(text=str(v), command=lambda x=v: self._preset(x))

            # --- Разомкнутый: процент ---
            self.lbl_ol_pct.config(text=f'{(olduty if ol else duty)/PWM_MAX*100.0:.0f}%')

            # --- График RPM ---
            c = self.canv_rpm
            c.delete('all')
            w = c.winfo_width() or 900
            h = c.winfo_height() or 220
            c.create_text(w//2, 12, text='ОБОРОТЫ, об/мин (жёлтый = уставка)',
                          fill='#888', font=('Consolas', 9))
            mx = max(max(rpm_h), rset, 1)
            mx = max(mx, 100)
            def px_rpm(v):
                return h - int(h * v / (mx * 1.1))
            c.create_line(0, px_rpm(rset), w, px_rpm(rset), fill='#a16207', width=2)
            pts = []
            for i, v in enumerate(rpm_h):
                x = w * i / (len(rpm_h)-1) if len(rpm_h) > 1 else 0
                pts += [x, px_rpm(v)]
            c.create_line(*pts, fill='#22c55e', width=2)

            # --- График тока ---
            c = self.canv_cur
            c.delete('all')
            w = c.winfo_width() or 900
            h = c.winfo_height() or 140
            c.create_text(w//2, 12, text=f'ТОК, А (RMS={rms:.2f}A пунктир, 1 A = {1.0/self.adc_to_a:.0f} отсч.)',
                          fill='#888', font=('Consolas', 9))
            cmx = max(max(cur_h), 0.2)
            def px_cur(v):
                return h - int(h * v / (cmx * 1.2))
            pts = []
            for i, v in enumerate(cur_h):
                x = w * i / (len(cur_h)-1) if len(cur_h) > 1 else 0
                pts += [x, px_cur(v)]
            c.create_line(*pts, fill='#38bdf8', width=2)
            y_rms = px_cur(rms)
            c.create_line(0, y_rms, w, y_rms, fill='#38bdf8', dash=(4,4))

            # --- Статус: одна строка, цвет по худшему ---
            # esp = размах 60 периодов = ОДИН оборот вала. На низких оборотах
            # это окно длится сотни мс, и в размах попадает не биение диска,
            # а само ускорение — цифра распухает до 1000+ на исправном датчике.
            # Показываем только на установившихся оборотах: rpm>400 (окно
            # короче 150 мс) и без заметного изменения оборотов.
            drpm = abs(rpm - getattr(self, '_esp_rpm_prev', rpm))
            self._esp_rpm_prev = rpm
            steady = rpm > 400 and drpm < 15
            esp_pct = esp / 10.0 if steady else None
            esp_txt = '—' if esp_pct is None else f'{esp_pct:.1f}%'
            esc = '#22c55e'; est = ''
            if esp_pct is not None:
                if esp_pct > 15: esc, est = '#ef4444', 'МЕХАНИКА!'
                elif esp_pct > 8: esc, est = '#eab308', 'биение'

            now_ms = int(time.time() * 1000)
            if self.ovr_tick == 0:
                self.ovr_tick = now_ms
                self.ovr_prev = ovr
            elif now_ms - self.ovr_tick >= 1000:
                self.ovr_rate = max(0, ovr - self.ovr_prev)
                self.ovr_prev = ovr
                self.ovr_tick = now_ms
            ovc = '#22c55e' if self.ovr_rate == 0 else '#ef4444'

            seg = []
            seg.append(('#e5e7eb',
                f'{rpm:5.0f}/{rset:<5.0f} об  |  {cur:5.2f} A  RMS {rms:4.2f}  |  '
                f'{self.power_smooth:4.0f} Вт  |  '
                f'ШИМ {duty/PWM_MAX*100.0:4.1f}%  |  Uдв {self.u_mot_live:.0f} В  |  '
                f'Iшины {self.i_bus_live:.2f} А  |  '
                f'Порог {climit:4.2f} A  |  Kp {kp_fw} Ki {ki_fw} Kd {kd_fw}  |  '
                f'CP {cikp_fw} CI {ciki_fw}  |  '
                f'SS {ss} RU {ru} RD {rd}  |  '))
            # Всё, что добавлено после этой строки — аварии и режимы —
            # во вторую строку статуса, чтобы не уезжало под кнопку.
            HEAD_N = len(seg)
            if eok:
                eg_pct = 100.0 * eg / (eg + eok)
                ec = '#22c55e' if eg_pct < 0.5 else ('#eab308' if eg_pct < 3.0 else '#ef4444')
                seg.append((ec, f'дребезг {eg_pct:.1f}% (eg {eg}/eok {eok})'))
            if boost_active := bst:
                seg.append(('#f97316', 'ФОРСАЖ ×1.6'))
            if trip:
                seg.append(('#ef4444', 'TRIP={}:{}!'.format(trip, TRIP_NAMES.get(trip, '?'))))
            if limit:
                seg.append(('#eab308', 'ЗАЩИТА!'))
            if ol:
                seg.append(('#22c55e', '[OL] '))
            if rpm_virt_sw:
                seg.append(('#3b82f6', '[Вирт] '))
            seg.append((esc, f'esp {esp_txt} '))
            seg.append((ovc, f'ovr {self.ovr_rate}/с '))
            if on:
                seg.append(('#22c55e', '[ДВИГ ВКЛ]'))
            else:
                seg.append(('#9ca3af', '[ДВИГ ВЫКЛ]'))

            # Замер перерегулирования: пик оборотов после скачка
            if self.step_t0:
                up = self.step_to >= self.step_from
                self.step_peak = (max(self.step_peak, rpm) if up
                                  else min(self.step_peak, rpm))
                if time.time() - self.step_t0 > STEP_WINDOW:
                    base = max(1.0, float(self.step_to))
                    over = abs(self.step_peak - base) / base * 100.0
                    self.lbl_step.config(
                        text=f'Перерегулирование {over:.1f}%   пик {self.step_peak:.0f} об/мин')
                    self.btn_step.config(state=tk.NORMAL)
                    self.step_t0 = 0.0

            worst = '#ef4444' if any(c == '#ef4444' for c, _ in seg[1:]) else \
                    '#eab308' if any(c == '#eab308' for c, _ in seg[1:]) else '#22c55e'
            # Первая строка — числа, вторая — аварии и режимы.
            head = seg[:HEAD_N]
            tail = seg[HEAD_N:]
            self.status.config(fg='#e5e7eb', text=''.join(t for _, t in head))
            self.status2.config(fg=worst, text=''.join(t for _, t in tail))
        except Exception:
            pass
        self.root.after(40, self.draw)

    def do_reset(self):
        self.send_cmd('RS')
        self.status.config(text='Защита сброшена')

    def on_close(self):
        self.running = False
        if self.logging and self.csv_file:
            self.csv_file.close()
        try:
            if self.ser:
                self.ser.write(b'o')
                self.ser.close()
        except Exception:
            pass
        self.root.destroy()


def main():
    root = tk.Tk()
    PidTuner(root)
    root.mainloop()


if __name__ == '__main__':
    main()