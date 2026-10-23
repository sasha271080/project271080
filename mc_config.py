#!/usr/bin/env python3
"""
mc_config — настройка контроллера двигателя без перепрошивки.
Читает текущие значения из телеметрии, пишет команды, сохраняет во Flash ('w').
"""
import serial, serial.tools.list_ports, tkinter as tk
from tkinter import ttk, messagebox
import threading, re, json, time

BAUD = 115200

PARAMS = [
    ('kp',   'KP', 'Kp — контур скорости',      0, 5000, 'жёсткость удержания оборотов'),
    ('ki',   'KI', 'Ki — контур скорости',      0, 5000, 'устраняет статическую ошибку'),
    ('kd',   'KD', 'Kd — контур скорости',      0, 5000, 'гасит раскачку'),
    ('cikp', 'CP', 'Kp — контур тока',          1, 5000, 'форма пускового тока'),
    ('ciki', 'CI', 'Ki — контур тока',          0, 5000, 'форма пускового тока'),
    ('cl',   'CL', 'Предел тока, отсчёты АЦП', 50, 3000, '763 = 1.00 А, 1500 ≈ 2.0 А'),
    ('ss',   'SS', 'SS — нарастание ШИМ',       1,  100, 'цифровой конденсатор'),
    ('ru',   'RU', 'RU — разгон уставки',       1,  100, 'об/мин за шаг'),
    ('rd',   'RD', 'RD — торможение уставки',   1,  100, 'об/мин за шаг'),
    ('uv',   'UV', 'Напряжение сети, В',      100,  300, 'для пересчёта на двигателе'),
]

# Параметры без обратного чтения (нет в JSON, но можно писать)
WRITE_ONLY = [
    ('pause','P',  'Пауза после защиты, мс',  100,30000, 'авторестарт по перегрузке'),
]

FLAGS = [
    ('te', 'TE', 'Тепловая защита I²t'),
    ('me', 'ME', 'Защита по пику тока'),
    ('tl', 'TL', 'Ограничение момента'),
]

MODES = [('0', 'Рабочий'), ('1', 'Отладка'), ('2', 'Тест'), ('3', 'Тяговый')]


class ShuntCalc(tk.Frame):
    def __init__(self, master, on_apply):
        super().__init__(master, padx=10, pady=8)
        self.on_apply = on_apply
        tk.Label(self, text='Калькулятор предела тока',
                 font=('Arial', 11, 'bold')).grid(row=0, column=0, columnspan=3, sticky='w')
        self.e_r = self._row(1, 'Шунт, Ом:', '0.075')
        self.e_g = self._row(2, 'Усиление датчика:', '8.2')
        self.e_i = self._row(3, 'Желаемый предел, А:', '1.0')
        tk.Button(self, text='Посчитать', command=self.calc).grid(row=4, column=0, pady=6)
        self.out = tk.Label(self, text='', fg='#166534', font=('Consolas', 10), justify='left')
        self.out.grid(row=4, column=1, columnspan=2, sticky='w')
        tk.Button(self, text='Подставить в «Предел тока»',
                  command=self.apply).grid(row=5, column=0, columnspan=3, pady=4)
        self.adc = None
        self.calc()

    def _row(self, r, label, default):
        tk.Label(self, text=label).grid(row=r, column=0, sticky='w', pady=2)
        e = tk.Entry(self, width=12)
        e.insert(0, default)
        e.grid(row=r, column=1, sticky='w')
        e.bind('<Return>', lambda ev: self.calc())
        return e

    def calc(self):
        try:
            r = float(self.e_r.get()); g = float(self.e_g.get()); i = float(self.e_i.get())
            mv = r * i * 1000.0
            adc = int(round(4095.0 * (r * i * g) / 3.3))
            adc = max(0, min(4095, adc))
            self.adc = adc
            warn = ''
            if mv > 300:   warn = '\n⚠ >300 мВ — абсолютный максимум датчика!'
            elif mv > 250: warn = '\n⚠ >250 мВ — вне линейного диапазона'
            elif mv > 200: warn = '\n⚠ близко к границе линейности'
            self.out.config(text=f'На шунте {mv:.0f} мВ → АЦП {adc}{warn}',
                            fg='#b91c1c' if warn else '#166534')
        except Exception:
            self.out.config(text='Введи числа', fg='#b91c1c')

    def apply(self):
        if self.adc:
            self.on_apply('cl', self.adc)


class App:
    def __init__(self, root):
        self.root = root
        root.title('Настройки контроллера двигателя')
        root.geometry('720x760')
        self.ser = None
        self.running = False
        self.lock = threading.Lock()
        self.raw = b''
        self.live = {}
        self.zones = None
        self.zvars = []
        self.vars = {}
        self.flag_vars = {}
        self.mode_var = tk.StringVar(value='0')
        self.dirty = False
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.build_connect()

    def build_connect(self):
        self.fr_conn = tk.Frame(self.root)
        self.fr_conn.pack(fill=tk.BOTH, expand=True)
        tk.Label(self.fr_conn, text='Выбери COM-порт контроллера:',
                 font=('Arial', 13)).pack(pady=20)
        self.ports = list(serial.tools.list_ports.comports())
        if not self.ports:
            tk.Label(self.fr_conn, text='Портов не найдено', fg='red').pack()
            return
        self.lb = tk.Listbox(self.fr_conn, height=8, font=('Consolas', 10))
        for p in self.ports:
            self.lb.insert(tk.END, f'{p.device} — {p.description}')
        self.lb.pack(fill=tk.X, padx=40)
        self.lb.selection_set(0)
        tk.Button(self.fr_conn, text='Подключить', command=self.connect,
                  bg='#2563eb', fg='white', padx=30, pady=6).pack(pady=16)

    def connect(self):
        sel = self.lb.curselection()
        if not sel: return
        try:
            self.ser = serial.Serial(self.ports[sel[0]].device, BAUD, timeout=0.3)
            time.sleep(0.4)
            self.ser.reset_input_buffer()
            self.ser.write(b'j')
            self.running = True
            threading.Thread(target=self.read_loop, daemon=True).start()
            self.fr_conn.destroy()
            self.build_main()
            self.root.after(600, self.pull)
        except Exception as e:
            messagebox.showerror('Ошибка', str(e))

    def read_loop(self):
        while self.running:
            try:
                n = self.ser.in_waiting
                if n == 0:
                    time.sleep(0.005)
                    continue
                data = self.ser.read(n)
                with self.lock:
                    self.raw += data
                    if len(self.raw) > 8192:
                        self.raw = self.raw[-4096:]
                    while True:
                        m = re.search(rb'\{[^{}]+\}', self.raw)
                        if not m: break
                        chunk = m.group(0)
                        self.raw = self.raw[m.end():]
                        try:
                            o = json.loads(chunk.decode('utf-8', errors='ignore'))
                            if 'zones' in o:
                                self.zones = o['zones']
                            else:
                                self.live = dict(o)
                        except Exception: pass
            except Exception: pass

    def build_main(self):
        nb = ttk.Notebook(self.root)
        nb.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)
        tab1 = tk.Frame(nb); nb.add(tab1, text='Параметры')
        tab2 = tk.Frame(nb); nb.add(tab2, text='Защиты и режимы')
        tab3 = tk.Frame(nb); nb.add(tab3, text='Калькулятор шунта')
        tab4 = tk.Frame(nb); nb.add(tab4, text='Автонастройка (зоны)')
        self.build_zones(tab4)

        for i, (key, cmd, label, lo, hi, hint) in enumerate(PARAMS):
            tk.Label(tab1, text=label, anchor='w', font=('Arial', 10)) \
                .grid(row=i, column=0, sticky='w', padx=8, pady=3)
            v = tk.StringVar(value='—')
            self.vars[key] = v
            e = tk.Entry(tab1, textvariable=v, width=10, justify='right')
            e.grid(row=i, column=1, padx=4)
            e.bind('<Return>', lambda ev, k=key: self.send_one(k))
            tk.Button(tab1, text='→', width=3,
                      command=lambda k=key: self.send_one(k)).grid(row=i, column=2)
            tk.Label(tab1, text=f'{lo}…{hi}  {hint}', fg='#6b7280',
                     font=('Arial', 8)).grid(row=i, column=3, sticky='w', padx=6)

        tk.Button(tab1, text='Записать всё в контроллер', command=self.send_all,
                  bg='#2563eb', fg='white', padx=16, pady=5) \
            .grid(row=len(PARAMS), column=0, columnspan=2, pady=12)
        tk.Button(tab1, text='Считать с контроллера', command=self.pull,
                  padx=16, pady=5).grid(row=len(PARAMS), column=2, columnspan=2, pady=12)

        for i, (key, cmd, label) in enumerate(FLAGS):
            bv = tk.IntVar(value=1)
            self.flag_vars[key] = bv
            tk.Checkbutton(tab2, text=label, variable=bv, font=('Arial', 10),
                           command=lambda k=key: self.send_flag(k)) \
                .grid(row=i, column=0, sticky='w', padx=10, pady=4)

        tk.Label(tab2, text='Режим токовой защиты:', font=('Arial', 10, 'bold')) \
            .grid(row=10, column=0, sticky='w', padx=10, pady=(14, 2))
        for i, (code, name) in enumerate(MODES):
            tk.Radiobutton(tab2, text=f'{code} — {name}', variable=self.mode_var,
                           value=code, command=self.send_mode) \
                .grid(row=11 + i, column=0, sticky='w', padx=24)

        tk.Label(tab2, text='ПИД:', font=('Arial', 10, 'bold')) \
            .grid(row=20, column=0, sticky='w', padx=10, pady=(14, 2))
        fr = tk.Frame(tab2); fr.grid(row=21, column=0, sticky='w', padx=24)
        tk.Button(fr, text='Автоподстройка (a)', width=18,
                  command=lambda: self.cmd('a')).pack(side=tk.LEFT, padx=3)
        tk.Button(fr, text='Ручной (m)', width=12,
                  command=lambda: self.cmd('m')).pack(side=tk.LEFT, padx=3)

        fr2 = tk.Frame(tab2); fr2.grid(row=22, column=0, sticky='w', padx=24, pady=6)
        tk.Button(fr2, text='Компенсация пульсаций вкл (r)', width=28,
                  command=lambda: self.cmd('r')).pack(side=tk.LEFT, padx=3)
        tk.Button(fr2, text='выкл (s)', width=10,
                  command=lambda: self.cmd('s')).pack(side=tk.LEFT, padx=3)

        tk.Button(tab2, text='СБРОС ЗАЩИТЫ (RS)', command=lambda: self.cmd('RS'),
                  bg='#dc2626', fg='white', padx=16, pady=5) \
            .grid(row=30, column=0, sticky='w', padx=10, pady=18)

        ShuntCalc(tab3, self.set_field).pack(fill=tk.BOTH, expand=True)

        bottom = tk.Frame(self.root, bg='#111827')
        bottom.pack(fill=tk.X)
        tk.Button(bottom, text='Сохранить во Flash (w)', command=self.do_save,
                  bg='#16a34a', fg='white', padx=20, pady=5, font=('Arial', 11)) \
            .pack(side=tk.LEFT, padx=10, pady=8)
        self.stat = tk.Label(bottom, text='...', bg='#111827', fg='#eab308',
                             font=('Consolas', 10), anchor='w')
        self.stat.pack(side=tk.LEFT, padx=10, fill=tk.X, expand=True)

    ZONE_NAMES = ['Холостой ход', 'Низкие', 'Ниже средних',
                  'Средние', 'Высокие', 'Максимум']

    def build_zones(self, tab):
        tk.Label(tab, text='Коэффициенты по оборотам',
                 font=('Arial', 11, 'bold')).grid(row=0, column=0, columnspan=6,
                                                  sticky='w', padx=8, pady=(8, 2))
        tk.Label(tab, text='Между строками значения меняются плавно. Берётся по '
                           'ФАКТИЧЕСКИМ оборотам, работает только в режиме «Авто» (a).',
                 fg='#6b7280', font=('Arial', 8), justify='left') \
            .grid(row=1, column=0, columnspan=6, sticky='w', padx=8)

        for c, h in enumerate(['Зона', 'До скольких об/мин', 'Kp', 'Ki', 'Kd']):
            tk.Label(tab, text=h, font=('Arial', 9, 'bold')) \
                .grid(row=2, column=c, padx=4, pady=(8, 2))

        self.zvars = []
        for i in range(6):
            tk.Label(tab, text=f'{i}  {self.ZONE_NAMES[i]}', anchor='w',
                     font=('Arial', 9)).grid(row=3 + i, column=0, sticky='w',
                                             padx=8, pady=2)
            row = {}
            for c, key in enumerate(['rpm', 'kp', 'ki', 'kd']):
                v = tk.StringVar(value='—')
                e = tk.Entry(tab, textvariable=v, width=8, justify='right')
                e.grid(row=3 + i, column=1 + c, padx=4)
                e.bind('<Return>', lambda ev, n=i: self.send_zone(n))
                row[key] = v
            self.zvars.append(row)
            tk.Button(tab, text='→', width=3,
                      command=lambda n=i: self.send_zone(n)).grid(row=3 + i, column=5, padx=4)

        fr = tk.Frame(tab)
        fr.grid(row=10, column=0, columnspan=6, sticky='w', padx=8, pady=12)
        tk.Button(fr, text='Считать таблицу', command=self.pull_zones,
                  padx=12, pady=4).pack(side=tk.LEFT, padx=3)
        tk.Button(fr, text='Записать всю таблицу', command=self.send_zones_all,
                  bg='#2563eb', fg='white', padx=12, pady=4).pack(side=tk.LEFT, padx=3)
        tk.Button(fr, text='Заводские (ZR)', command=self.zones_default,
                  padx=12, pady=4).pack(side=tk.LEFT, padx=3)
        tk.Label(tab, text='После правки нажми «Сохранить во Flash» — иначе таблица '
                           'вернётся к прежней после перезагрузки.',
                 fg='#b45309', font=('Arial', 8)) \
            .grid(row=11, column=0, columnspan=6, sticky='w', padx=8)

    def pull_zones(self):
        with self.lock:
            self.zones = None
        self.cmd('ZD')
        self.root.after(500, self._fill_zones)

    def _fill_zones(self):
        with self.lock:
            z = self.zones
        if not z:
            self.status('Таблица не пришла — нужна прошивка с поддержкой ZD')
            return
        for i, row in enumerate(z[:6]):
            if len(row) < 4:
                continue
            for c, key in enumerate(['rpm', 'kp', 'ki', 'kd']):
                self.zvars[i][key].set(str(row[c]))
        self.status('Таблица считана')

    def send_zone(self, n):
        for key, L in [('kp', 'P'), ('ki', 'I'), ('kd', 'D'), ('rpm', 'R')]:
            raw = self.zvars[n][key].get().strip()
            if raw in ('', '—'):
                continue
            try:
                v = int(float(raw))
            except Exception:
                self.status(f'Зона {n}: нужно целое число')
                return
            lo, hi = (50, 12000) if key == 'rpm' else (0, 5000)
            if not (lo <= v <= hi):
                self.status(f'Зона {n}: {key} вне {lo}…{hi}')
                return
            self.cmd(f'Z{n}{L}{v}')
            time.sleep(0.05)
        self.dirty = True
        self.status(f'Зона {n} записана')

    def send_zones_all(self):
        for n in range(6):
            self.send_zone(n)
        self.status('Таблица записана')

    def zones_default(self):
        self.cmd('ZR')
        self.root.after(300, self.pull_zones)

    def cmd(self, c):
        try:
            if self.ser:
                self.ser.write(c.encode()); self.ser.flush()
        except Exception: pass

    def send_one(self, key):
        var = self.vars.get(key)
        if var is None:
            return
        for k, cmd, label, lo, hi, _ in PARAMS:
            if k != key:
                continue
            try:
                v = int(float(var.get()))
            except Exception:
                self.status(f'{label}: нужно целое число')
                return
            if not (lo <= v <= hi):
                self.status(f'{label}: допустимо {lo}…{hi}')
                return
            self.cmd(f'{cmd}{v}')
            self.dirty = True
            self.status(f'{label} = {v}')
            return

    def send_all(self):
        for key in self.vars:
            self.send_one(key)
            time.sleep(0.05)
        self.status('Записано')

    def send_flag(self, key):
        cmd_map = {'te':'TE','me':'ME','tl':'TL'}
        c = cmd_map.get(key)
        if not c: return
        v = self.flag_vars[key].get()
        self.ser.write(f'{c}{v}'.encode()); self.ser.flush()

    def send_mode(self):
        self.cmd(self.mode_var.get())

    def set_field(self, key, val):
        if key in self.vars:
            self.vars[key].set(str(val))

    def pull(self):
        time.sleep(0.3)
        kv = self.live
        if not kv:
            self.status('Нет данных: проверь кабель и режим j')
            return
        m = {'kp':'kp','ki':'ki','kd':'kd','cikp':'cikp','ciki':'ciki',
             'cl':'cls','ss':'ss','ru':'ru','rd':'rd','uv':'uv',
             'te':'te','me':'me','tl':'tl'}
        for k, vk in m.items():
            val = kv.get(vk, kv.get('cl' if vk == 'cls' else vk))
            if val is not None and k in self.vars:
                self.vars[k].set(str(val))
        if 'te' in kv: self.flag_vars['te'].set(kv['te'])
        if 'me' in kv: self.flag_vars['me'].set(kv['me'])
        if 'tl' in kv: self.flag_vars['tl'].set(kv['tl'])
        if 'prot' in kv:
            self.mode_var.set(str(kv['prot']))
        self.status('Считано')

    def do_save(self):
        if self.live.get('on', 0):
            self.status('Сначала выключи двигатель (on=0)!')
            return
        self.cmd('w')
        self.status('Сохранено во Flash')

    def status(self, t):
        self.stat.config(text=t)
        self.root.after(2000, lambda: self.stat.config(text='...'))

    def close(self):
        self.running = False
        try:
            if self.ser:
                self.ser.write(b'o')
                self.ser.close()
        except Exception: pass
        self.root.destroy()


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()

if __name__ == '__main__':
    main()