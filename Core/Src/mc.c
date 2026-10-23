/**
  ******************************************************************************
  * @file    mc.c
  * @brief   Ядро контроллера ДПТ — каскадная структура управления.
  *
  *        ┌──────────┐  rpm_ref  ┌──────────────┐  i_ref  ┌────────────┐ duty
  *  PA2 → │  рампа   │──────────▶│ контур       │────────▶│ контур     │────▶ ШИМ
  *        │ уставки  │           │ СКОРОСТИ ПИД │  клип   │ ТОКА  ПИ   │
  *        └──────────┘           └──────────────┘    ▲    └────────────┘
  *                                      ▲            │           ▲
  *                                 rpm_meas      PA3 / защиты   i_meas
  *                                 (энкодер)                   (PA0-PA1)
  *
  *  Смысл переделки: внешний контур больше НЕ управляет ШИМ напрямую. Он
  *  просит ТОК. Ток — это момент, поэтому «ограничение момента»,
  *  «пусковой ток» и «тяговый режим» становятся одним и тем же действием —
  *  клипом уставки тока. Пусковой ток ограничен по построению, а не
  *  таймерами и min-селекторами.
  ******************************************************************************
  */

#include "main.h"
#include "mc.h"
#include "mc_wifi.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;

/* ==========================================================================
 *  БУФЕР АЦП
 *
 *  ИСПРАВЛЕНА ГОНКА. Раньше DMA работал в CIRCULAR и непрерывно писал в
 *  буфер, а process_frame() читал этот же буфер целиком — первые отсчёты
 *  кадра затирались прямо во время расчёта. Теперь обрабатывается та
 *  половина, в которую DMA в данный момент НЕ пишет.
 * ========================================================================== */
volatile uint16_t mc_adc_buf[ADC_BUF_LEN];

static volatile uint8_t  half_ready = 0;
static volatile uint8_t  full_ready = 0;
static volatile uint32_t adc_overrun = 0;   /* не успели обработать */

/* ==========================================================================
 *  ВРЕМЯ: TIM2 расширен до 32 бит
 *
 *  Раньше период энкодера считался по 16-битному TIM2->CNT: на оборотах
 *  ниже ~15 об/мин счётчик переполнялся и rpm вычислялся из мусора.
 * ========================================================================== */
static volatile uint32_t tim2_high = 0;

void mc_tim2_overflow(void)
{
  tim2_high++;
}

uint32_t mc_us(void)
{
  uint32_t h, cnt;
  uint32_t pm = __get_PRIMASK();
  __disable_irq();

  cnt = TIM2->CNT;
  h   = tim2_high;

  /* ВАЖНО: проверка UIF, а не сравнение tim2_high до/после.
     Мы можем быть внутри EXTI энкодера (приоритет 0), который перебивает
     TIM2 (приоритет 1) — обработчик переполнения ЗАПУСТИТЬСЯ НЕ МОЖЕТ,
     и tim2_high гарантированно отстаёт. Сравнение h1!=h2 этот случай не
     видит (оба чтения дают одно старое значение), время прыгает назад на
     65536 мкс, в mc_enc_edge беззнаковое вычитание даёт период ~4.29e9,
     кольцо забивается мусором и rpm залипает на нуле -> ложный TRIP_ENC. */
  if (TIM2->SR & TIM_SR_UIF)
  {
    cnt = TIM2->CNT;   /* мог обнулиться между строками — перечитываем */
    h++;
  }

  if (!pm) __enable_irq();
  return (h << 16) | cnt;
}

/* ==========================================================================
 *  ЭНКОДЕР: M/T-метод
 *
 *  Было: rpm = 1e6 / период_одного_импульса, пересчёт только по фронту.
 *  Две беды — шум квантования на малых оборотах и, главное, ЗАМИРАНИЕ
 *  показаний при остановке вала (фронтов нет -> rpm_meas навсегда держит
 *  последнее значение, регулятор считает, что мотор крутится).
 *
 *  Стало:
 *    - в прерывании только накопление: счётчик импульсов и суммарное время;
 *    - расчёт в контуре скорости: rpm = 60e6 * N / (PPR * dt);
 *    - фронтов не было дольше ENC_TIMEOUT_US — обороты 0, честно;
 *    - между фронтами показание сползает вниз по физическому потолку.
 * ========================================================================== */
/* ==========================================================================
 *  ЭНКОДЕР: M/T-метод с кольцевым буфером на ПОЛНЫЙ ОБОРОТ
 *
 *  Проблема исходного кода: окно ~12 мс ловило случайный кусок периода
 *  биения диска (осевое качание в оптопаре). Биение повторяется раз за
 *  оборот — это синусоида, и её интеграл за период равен нулю.
 *  Усреднение ровно по ENC_PPR импульсам (полный оборот) убирает биение
 *  тождественно, без калибровки и без фазовой привязки.
 * ========================================================================== */
static volatile uint32_t enc_ring[ENC_PPR];   /* кольцо периодов */
static volatile uint32_t enc_ring_sum = 0;    /* сумма периодов в кольце */
static volatile uint8_t  enc_ring_cnt = 0;    /* сколько накоплено (до 60) */
static volatile uint8_t  enc_ring_idx = 0;    /* индекс записи */
static volatile uint8_t  enc_anom_cnt = 0;    /* подряд аномальных периодов */
static volatile uint32_t enc_last_us = 0;
static volatile uint8_t  enc_first   = 1;
static volatile uint32_t enc_glitch  = 0;
static volatile uint32_t enc_good    = 0;

/* --- измерение по ЦЕНТРАМ прорезей ------------------------------------
   У щелевого датчика плавает ШИРИНА импульса (порог компаратора, засветка,
   фреза, RC на входе), а не положение прорези. Всё это симметрично
   раздвигает/сжимает края, центр прорези при этом стоит — он задан
   делением окружности на ЧПУ. Меряем разность соседних центров:
   постоянный сдвиг в разности исчезает, остаётся честная геометрия.
   ENC_USE_CENTER в mc.h: 1 = метод центров, 0 = старый (по нарастающим). */
static volatile uint32_t enc_t_rise    = 0;   /* время нарастающего фронта */
static volatile uint8_t  enc_have_rise = 0;   /* фронт поймали, ждём спад  */
static volatile uint32_t enc_prev_ctr  = 0;   /* центр предыдущей прорези  */
static volatile uint8_t  enc_have_ctr  = 0;   /* есть с чем сравнивать     */
static volatile uint32_t enc_last_w    = 0;   /* ширина, для диагностики   */

/* Приём готового периода в кольцо. Логика НЕ ИЗМЕНЕНА — это дословно
   прежний блок из mc_enc_edge, только период приходит параметром,
   а не вычисляется внутри. */
static void enc_push_period(uint32_t d, uint32_t now)
{
  if (d > ENC_TIMEOUT_US)
  {
    enc_glitch++;
    enc_last_us = now;
    return;
  }

  if (d < ENC_MIN_PULSE_US)
  {
    enc_glitch++;                      /* дребезг: период не принимаем */
    return;
  }

  enc_good++;
  enc_last_us = now;

  /* Антизалипание нужно там, где период короткий и аномалия означает
     помеху. На низах (период > 3 мс) вдвое больший период — это
     нормальное торможение, а не глюк: фильтр только мешает трогаться. */
  if (enc_ring_cnt >= ENC_PPR && enc_ring_sum > 0u &&
      (enc_ring_sum / ENC_PPR) < 3000u)
  {
    uint32_t avg = enc_ring_sum / ENC_PPR;
    if (d > avg * 2u || d * 2u < avg)
    {
      uint8_t slowing = (d > avg * 2u);
      enc_anom_cnt++;

      if (enc_anom_cnt >= 8u)
      {
        enc_anom_cnt = 0;
        for (uint8_t i = 0; i < ENC_PPR; i++)
          enc_ring[i] = d;
        enc_ring_sum = (uint32_t)ENC_PPR * d;
        enc_ring_idx = 0;
      }
      else if (!slowing)
      {
        enc_glitch++;
      }
      return;
    }
    else enc_anom_cnt = 0;
  }

  /* кольцевой буфер периодов: полный оборот = ровно ENC_PPR шт */
  uint32_t old = enc_ring[enc_ring_idx];
  enc_ring[enc_ring_idx] = d;
  enc_ring_sum += d - old;
  if (enc_ring_cnt < ENC_PPR) enc_ring_cnt++;
  enc_ring_idx = (enc_ring_idx + 1u) % ENC_PPR;
}

void mc_enc_edge(void)
{
  uint32_t now = mc_us();

#if ENC_USE_CENTER

  /* Уровень читаем сразу: EXTI на оба фронта не говорит, какой именно
     это был, а на 2500 об/мин полка длится ~200 мкс — успеваем. */
  uint8_t level = (GPIOB->IDR & GPIO_PIN_5) ? 1u : 0u;

  if (level)
  {
    /* Начало прорези. Период ещё не готов — только запоминаем момент.
       Если предыдущий спад потерян, затираем старое: пропустим одну
       прорезь, следующий период окажется двойным и его поймает фильтр
       аномалий. Само вылечится. */
    enc_t_rise    = now;
    enc_have_rise = 1u;
    return;
  }

  /* Конец прорези. */
  if (!enc_have_rise) return;          /* спад без начала — игнорируем */
  enc_have_rise = 0u;

  uint32_t w = now - enc_t_rise;       /* ширина импульса */
  if (w < ENC_MIN_WIDTH_US || w > ENC_TIMEOUT_US)
  {
    enc_glitch++;
    return;
  }
  enc_last_w = w;

  /* СЕРЕДИНА прорези: w уже разность, прибавление к enc_t_rise корректно
     в беззнаковой арифметике даже через границу 32 бит. */
  uint32_t ctr = enc_t_rise + (w >> 1);

  if (!enc_have_ctr)
  {
    enc_prev_ctr = ctr;
    enc_have_ctr = 1u;
    enc_last_us  = now;
    enc_first    = 0u;
    return;
  }

  uint32_t d = ctr - enc_prev_ctr;
  enc_prev_ctr = ctr;   /* двигаем ВСЕГДА, даже если период отбракуют */
  enc_push_period(d, now);

#else   /* старый способ: по нарастающим фронтам */

  if (enc_first) { enc_first = 0u; enc_last_us = now; return; }
  enc_push_period(now - enc_last_us, now);

#endif
}

/* Скользящая статистика качества датчиков.
   Было: счётчики только росли и никогда не сбрасывались, поэтому один
   плохой контакт при включении держал аварийную индикацию до перезагрузки. */
static uint32_t enc_glitch_win = 0, enc_good_win = 0, enc_stat_us = 0;
static uint32_t pot_glitch_win = 0;

static int32_t enc_read_rpm(uint32_t dt_us)
{
  uint32_t cnt;
  __disable_irq();
  cnt = enc_ring_cnt;
  __enable_irq();

  uint32_t since = mc_us() - enc_last_us;

  if (since > ENC_TIMEOUT_US)
    return 0;                          /* вал стоит — без вариантов */

  int32_t rpm = 0;
  if (cnt > 0)
  {
    /* Окно усреднения ~60 мс, а НЕ полный оборот.
       На высоких оборотах в 60 мс влезает весь оборот — биение диска
       гасится ровно как раньше. На низких берём меньше периодов:
       при 100 об/мин полный оборот это 600 мс запаздывания. */
    __disable_irq();
    uint8_t  idx  = enc_ring_idx;
    uint32_t last = enc_ring[(idx + ENC_PPR - 1u) % ENC_PPR];
    __enable_irq();

    uint32_t n = (last > 0u) ? (60000u / last) : (uint32_t)ENC_PPR;
    if (n < 4u)                n = 4u;      /* меньше — шумно */
    if (n > (uint32_t)ENC_PPR) n = ENC_PPR;
    if (n > (uint32_t)cnt)     n = cnt;

    uint32_t s = 0;
    __disable_irq();
    for (uint32_t i = 1; i <= n; i++)
      s += enc_ring[(idx + ENC_PPR - i) % ENC_PPR];
    __enable_irq();

    if (s > 0)
      rpm = (int32_t)((60000000ULL * n) / ((uint64_t)ENC_PPR * s));
  }
  else if (dt_us > 0 && since > 0)
  {
    rpm = (int32_t)(60000000ULL / ((uint64_t)ENC_PPR * since));
  }

  /* Физический потолок: показание не может превышать предел, задаваемый
     временем с последнего фронта — отсекает выброс от сдвоенных импульсов */
  if (since > 0 && rpm > 0)
  {
    int32_t cap = (int32_t)(60000000ULL / ((uint64_t)ENC_PPR * since));
    if (rpm > cap) rpm = cap;
  }
  if (rpm > RPM_MAX * 2) rpm = RPM_MAX * 2;
  return rpm;
}

/* Размах периодов в кольце, в промилле от среднего (биение за оборот).
   0 = идеальный диск, 100 = разброс 10 %. Выводится в телеметрию. */
static uint32_t enc_spread_pct(void)
{
  uint32_t copy[ENC_PPR];
  __disable_irq();
  for (uint8_t i = 0; i < ENC_PPR; i++)
    copy[i] = enc_ring[i];
  __enable_irq();

  uint32_t mn = 0xFFFFFFFFu, mx = 0u, s = 0u, nz = 0u;
  for (uint8_t i = 0; i < ENC_PPR; i++)
  {
    uint32_t p = copy[i];
    if (p == 0u) continue;
    nz++;
    if (p < mn) mn = p;
    if (p > mx) mx = p;
    s += p;
  }
  if (nz < 2u || s == 0u) return 0u;
  uint32_t avg = s / nz;
  if (avg == 0u) return 0u;
  return (uint32_t)(((uint64_t)(mx - mn) * 1000u) / avg);
}

/* Геттеры для mc_disp.c: статические переменные не видны из другого файла. */
uint32_t mc_get_adc_overrun(void) { return adc_overrun; }
int32_t  mc_get_enc_spread_pct(void) { return (int32_t)enc_spread_pct(); }

/* Счётчики дребезга за последнюю секунду. Сырые enc_glitch/enc_good
   обнуляются каждую секунду, поэтому наружу отдаём именно _win-снимок:
   иначе поймаем случайную фазу. Читаются в телеметрии (eg, eok). */
uint32_t mc_get_enc_glitch(void) { return enc_glitch_win; }
uint32_t mc_get_enc_good(void)   { return enc_good_win; }

/* Скважность прорези в промилле: 500 = прорезь равна перемычке.
   Сильное отклонение от 500 — прямое доказательство, что ширина и период
   не совпадают, то есть метод центров работает не зря. */
uint32_t mc_get_enc_duty(void)
{
  uint32_t w = enc_last_w;
  uint32_t avg = (enc_ring_cnt >= ENC_PPR) ? (enc_ring_sum / ENC_PPR) : 0u;
  if (avg == 0u) return 0u;
  return (uint32_t)(((uint64_t)w * 1000u) / avg);
}

/* ==========================================================================
 *  ФИЛЬТРЫ
 *
 *  Всё в масштабе x256. Было: `x += (new - x) / 4` на целых, и при разнице
 *  меньше 4 приращение обнулялось — фильтр замирал ровно там, где нужна
 *  точность (малая ошибка). Тот же дефект был в интерполяции
 *  коэффициентов: `kp += (target - kp) / 8` никогда не доходил до цели.
 * ========================================================================== */
#define FX 8                           /* дробных бит */
#define TO_FX(x)   ((int32_t)(x) << FX)
#define FROM_FX(x) ((x) >> FX)

static inline int32_t lp_fx(int32_t state_fx, int32_t input, int32_t shift)
{
  return state_fx + ((TO_FX(input) - state_fx) >> shift);
}

static inline int32_t clamp(int32_t v, int32_t lo, int32_t hi)
{
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* ==========================================================================
 *  СОСТОЯНИЕ
 * ========================================================================== */
volatile int32_t  mc_rpm_meas = 0;
volatile int32_t  mc_rpm_set  = 0;
volatile int32_t  mc_i_meas   = 0;
volatile int32_t  mc_i_ref    = 0;
volatile uint32_t mc_duty     = 0;
volatile uint32_t mc_rms      = 0;
static uint32_t mc_rms_sq     = 0;
volatile uint32_t mc_peak     = 0;
volatile uint32_t mc_pot_raw  = 0;
volatile uint32_t mc_inverter_on = 0;
volatile mc_trip_t mc_trip    = TRIP_NONE;

volatile int32_t pid_kp = 300, pid_ki = 110, pid_kd = 260;
/* Ручные коэффициенты живут ОТДЕЛЬНО от pid_kp/ki/kd.
   pid_kp/ki/kd теперь показывают «что действует сейчас», их пишет автомат,
   и раньше из-за этого ручной режим стартовал с авто-значений. */
volatile int32_t kp_user = 300, ki_user = 110, kd_user = 260;
volatile int32_t ci_kp  = 5, ci_ki  = 250;    /* контур тока — как в рабочем архиве */

volatile uint8_t  json_mode = 1;
volatile uint8_t  current_prot_mode = 0;
volatile uint8_t  pid_auto = 1;
volatile uint8_t  mc_zone_dump_req = 0;   /* запрос выдачи таблицы зон */
volatile uint32_t rpm_filt_shift = 2;     /* FR — сдвиг фильтра оборотов */
volatile uint32_t derr_filt_shift = 2;    /* FD — сдвиг фильтра D-звена   */
volatile uint32_t speed_loop_div = 16;     /* SL — делитель контура скорости */
volatile uint8_t  pid_ripple = 1;
volatile uint8_t  torque_limit = 1;
volatile uint8_t  thermal_enable = 0;
volatile uint8_t  max_enable = 0;
volatile uint8_t  inverter_sw = 0;
volatile uint8_t  rpm_virt_sw = 0;
volatile uint32_t rpm_virt = RPM_MIN;

/* Рабочий диапазон оборотов — настраивается из тюнера (команды RL/RH).
   RPM_MIN/RPM_MAX остаются ПОТОЛКОМ ЖЕЛЕЗА и границами проверок: рабочий
   диапазон только сужается внутри них, расширить его командой нельзя. */
volatile uint32_t rpm_lo = RPM_MIN;
volatile uint32_t rpm_hi = RPM_MAX;

volatile uint32_t current_limit = CURRENT_LIMIT;
volatile uint32_t current_limit_set = CURRENT_LIMIT;   /* задаётся командой CL */
volatile uint32_t cl_min_abs = 458;   /* пол току (отсчёты АЦП, ~0.6 А):
                                         нижняя граница, до которой PA3 может
                                         опустить порог. АБСОЛЮТНАЯ, а не процент:
                                         иначе при смене CL пол разъезжался и
                                         душил двигатель ниже холостого хода. */
volatile uint32_t pause_time_ms = OVERLOAD_RETRY_MS;
volatile uint32_t ss_step = 25;   /* было 6: подъём за 2 с вместо 8 */
volatile uint32_t ramp_step_up = 10;
volatile uint32_t ramp_step_down = 30;
volatile uint32_t line_voltage = 230;
volatile uint8_t  mc_cfg_save_req = 0;
volatile uint8_t  mc_cfg_reset_req = 0;   /* CF — стереть конфиг и перезагрузиться */
volatile uint32_t last_cmd_ms = 0;        /* признак живой связи с ПК */

/* --- Режим разомкнутого контура (отладка) ------------------------------
   Оба контура и защита энкодера отключены: ШИМ задаётся напрямую с
   терминала или потенциометром. Токовая защита ПРОДОЛЖАЕТ работать —
   без неё ошибка в задании ШИМ уходит прямо в ключ. */
volatile uint8_t  ol_mode = 0;    /* 0 — обычный, 1 — разомкнутый */
volatile uint32_t ol_duty = 0;    /* задание ШИМ в промилле, 0..PWM_MAX */
volatile uint8_t  ol_pot  = 0;    /* 1 — ШИМ задаётся потенциометром */

/* --- тяговый режим ---------------------------------------------------- */
volatile uint8_t  tq_mode    = 0;
volatile uint32_t tq_rpm_cap = TQ_CAP_DEF;
volatile uint32_t tq_band    = TQ_BAND_DEF;
volatile uint32_t tq_dz      = TQ_DZ_DEF;
volatile uint32_t tq_rate    = TQ_RATE_DEF;
volatile uint32_t tq_imax    = 0;        /* 0 = брать current_limit_set */
volatile uint32_t tq_floor   = TQ_FLOOR_DEF;
volatile uint32_t tq_hold_ms = TQ_HOLD_DEF;

/* СЫРОЕ задание с PA3, без пола CN. В режиме оборотов пол нужен, в тяге
   он превращает ручку в неотпускаемую педаль: ток не уходит в ноль. */
static volatile uint32_t tq_pot_raw = 0;
static int32_t  tq_ref_ramp = 0;   /* текущий ток после рампы */
static uint32_t tq_ramp_us  = 0;
static uint32_t tq_hold_us  = 0;
static uint32_t peak_frames = 0;   /* счётчик блоков под пиковым током */
static uint32_t boost_us    = 0;   /* накопленное время форсажа, мкс */
static uint32_t boost_rest  = 0;   /* пауза между форсажами, мкс */
volatile uint32_t boost_time_ms = 300;   /* BT — длительность форсажа, мс */

/* внутреннее */
static int32_t  i_filt_fx = 0;
static int32_t  i_offset  = 0;        /* КАЛИБРОВКА НУЛЯ датчика — новое */
static int32_t  i_off_acc = 0;
static uint32_t i_off_n   = 0;
static uint8_t  i_off_done = 0;

static int32_t  rpm_ramp    = RPM_MIN;
static int32_t  rpm_target  = RPM_MIN;
static int32_t  rpm_filt_fx = 0;

static int32_t  sp_integral_fx = 0;
static int32_t  sp_err_prev_fx = 0;
static int32_t  sp_err_filt_fx = 0;
static int32_t  ci_integral_fx = 0;

static uint32_t cl_from_pot = 0;
static uint32_t soft_start  = 0;
static uint32_t thermal_integral = 0;
static uint32_t oc_frames = 0;
static uint32_t trip_time_ms = 0;
static uint32_t start_tick = 0;
static uint32_t on_tick = 0;
static uint32_t pot_prev = 0;
static int32_t  pot_dir = 0;
static uint32_t pot_zero_tick = 0;
static uint32_t btn_raw_prev = 1;
static uint32_t btn_deb_tick = 0;
static uint32_t tick_cnt = 0;
static uint32_t ramp_last_us = 0;
static uint32_t telem_last_us = 0;
static uint32_t loop_alive_us = 0;
static uint32_t enc_fault_us = 0;

static char sbuf[2][1024];            /* двойной буфер JSON: худший кадр ~712 байт,
                                         плюс запас на длинные числа (ovr, eok) */
static uint8_t jsel = 0;
static uint8_t txbuf[2][128 + 512];   /* двойной буфер O-кадров */
static uint8_t txsel = 0;
static int16_t diff[ADC_HALF_SETS];

/* ==========================================================================
 *  КОНФИГУРАЦИЯ ВО FLASH
 *
 *  ИСПРАВЛЕНО: раньше Cfg_Save() вызывался прямо из прерывания USB. Стирание
 *  страницы — 20..40 мс с остановом конвейера, и всё это время контур
 *  управления стоял, а ШИМ держал последнее заполнение. Теперь команда 'w'
 *  только поднимает флаг; запись идёт в главном цикле и ТОЛЬКО когда
 *  двигатель остановлен.
 * ========================================================================== */
#define CFG_MAGIC      0x4D4F5437u     /* "MOT7" — без дисплея, TIM1_BKIN; сброс старых настроек */
#define CFG_FLASH_ADDR 0x0801FC00u     /* последняя страница 1 КБ F103C8 */

typedef struct
{
  uint32_t magic;
  int32_t  kp, ki, kd;
  int32_t  cikp, ciki;
  uint8_t  prot_mode, pid_auto, pid_ripple, torque_limit;
  uint8_t  thermal_en, max_en, rsv1, rsv2;
  uint32_t climit, pause_ms, ss_step, ramp_up, ramp_down, line_v;
  int32_t  z_rpm[ZONES_N], z_kp[ZONES_N], z_ki[ZONES_N], z_kd[ZONES_N];
  /* Резерв под раскладку дисплея в экспериментальном проекте. В итоговом
     (без дисплея) не используется; поле сохранено, чтобы не сдвигать
     смещения остальных настроек. 12 слотов × 9 байт = 108 байт. */
  uint8_t  disp_reserved[12][9];
  uint32_t cl_min_abs;
  /* Тяговый режим — тоже в конец, чтобы не сдвигать старые поля */
  uint8_t  tq_mode;
  uint32_t tq_cap, tq_band, tq_dz, tq_rate, tq_imax, tq_floor, tq_hold;
  uint32_t bt;   /* длительность форсажа, мс — переживает перепрошивку */
  uint32_t rlo, rhi;   /* рабочий диапазон оборотов — дописан в конец */
  uint32_t crc;
} Cfg_t;

static uint32_t cfg_crc(const Cfg_t *c)
{
  const uint8_t *p = (const uint8_t *)c;
  uint32_t crc = 0xA5A5A5A5u;
  for (uint32_t i = 0; i < sizeof(Cfg_t) - 4u; i++)
    crc = (crc << 5) ^ (crc >> 27) ^ p[i];
  return crc;
}

void mc_cfg_save(void)
{
  if (mc_inverter_on) return;               /* только на остановленном */

  Cfg_t cfg = {0};
  cfg.magic = CFG_MAGIC;
  cfg.kp = kp_user;  cfg.ki = ki_user;  cfg.kd = kd_user;
  cfg.cikp = ci_kp; cfg.ciki = ci_ki;
  cfg.prot_mode = current_prot_mode;
  cfg.pid_auto = pid_auto;
  cfg.pid_ripple = pid_ripple;
  cfg.torque_limit = torque_limit;
  cfg.thermal_en = thermal_enable;
  cfg.max_en = max_enable;
  cfg.climit = current_limit_set;
  cfg.pause_ms = pause_time_ms;
  cfg.ss_step = ss_step;
  cfg.ramp_up = ramp_step_up;
  cfg.ramp_down = ramp_step_down;
  cfg.line_v = line_voltage;
  for (int i = 0; i < ZONES_N; i++)
  {
    cfg.z_rpm[i] = zone_rpm[i]; cfg.z_kp[i] = zone_kp[i];
    cfg.z_ki[i]  = zone_ki[i];  cfg.z_kd[i] = zone_kd[i];
  }
  /* Раскладка дисплея больше НЕ пишется во Flash: источник истины —
     прошивка (mc_disp.c, страницы). Поле в структуре оставлено, чтобы
     не сдвигать смещения остальных настроек. */
  cfg.cl_min_abs = cl_min_abs;
  cfg.tq_mode = tq_mode;
  cfg.tq_cap = tq_rpm_cap;  cfg.tq_band = tq_band;
  cfg.tq_dz = tq_dz;        cfg.tq_rate = tq_rate;
  cfg.tq_imax = tq_imax;    cfg.tq_floor = tq_floor;
  cfg.tq_hold = tq_hold_ms;
  cfg.bt = boost_time_ms;   /* форсаж тоже переносится через перепрошивки */
  cfg.rlo = rpm_lo;  cfg.rhi = rpm_hi;   /* рабочий диапазон оборотов */
  cfg.crc = cfg_crc(&cfg);

  FLASH_EraseInitTypeDef f = {0};
  uint32_t err = 0;
  f.TypeErase   = FLASH_TYPEERASE_PAGES;
  f.PageAddress = CFG_FLASH_ADDR;
  f.NbPages     = 1;

  HAL_FLASH_Unlock();
  if (HAL_FLASHEx_Erase(&f, &err) == HAL_OK)
  {
    const uint32_t *p = (const uint32_t *)&cfg;
    for (uint32_t i = 0; i < sizeof(Cfg_t) / 4u; i++)
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, CFG_FLASH_ADDR + i * 4u, p[i]);
  }
  HAL_FLASH_Lock();
}

static void cfg_load(void)
{
  const Cfg_t *c = (const Cfg_t *)CFG_FLASH_ADDR;
  if (c->magic != CFG_MAGIC) return;
  if (c->crc != cfg_crc(c))  return;        /* битая запись — игнор */

  /* Каждое поле проверяется диапазоном: повреждённая ячейка не сможет
     задать невозможный порог тока или нулевой коэффициент */
  if (c->kp   >= 0 && c->kp   <= 5000) kp_user = c->kp;
  if (c->ki   >= 0 && c->ki   <= 5000) ki_user = c->ki;
  if (c->kd   >= 0 && c->kd   <= 5000) kd_user = c->kd;
  if (c->cikp >  0 && c->cikp <= 5000) ci_kp  = c->cikp;
  if (c->ciki >= 0 && c->ciki <= 5000) ci_ki  = c->ciki;
  if (c->prot_mode <= 3) current_prot_mode = c->prot_mode;
  pid_auto       = c->pid_auto     & 1u;
  pid_ripple     = c->pid_ripple   & 1u;
  torque_limit   = c->torque_limit & 1u;
  thermal_enable = c->thermal_en   & 1u;
  max_enable     = c->max_en       & 1u;
  if (c->climit   >= 50  && c->climit   <= 3000) current_limit_set  = c->climit;
  if (c->pause_ms >= 100 && c->pause_ms <= 30000) pause_time_ms  = c->pause_ms;
  if (c->ss_step  >= 1   && c->ss_step  <= 100)   ss_step        = c->ss_step;
  if (c->ramp_up  >= 1   && c->ramp_up  <= 100)   ramp_step_up   = c->ramp_up;
  if (c->ramp_down>= 1   && c->ramp_down<= 100)   ramp_step_down = c->ramp_down;
  if (c->line_v   >= 100 && c->line_v   <= 300)   line_voltage   = c->line_v;

  /* Таблица зон: каждое значение проверяется отдельно, битая ячейка
     оставит заводское значение, а не обнулит коэффициент */
  for (int i = 0; i < ZONES_N; i++)
  {
    if (c->z_rpm[i] >= 50 && c->z_rpm[i] <= RPM_MAX * 2) zone_rpm[i] = c->z_rpm[i];
    if (c->z_kp[i]  >= 0  && c->z_kp[i]  <= 5000)        zone_kp[i]  = c->z_kp[i];
    if (c->z_ki[i]  >= 0  && c->z_ki[i]  <= 5000)        zone_ki[i]  = c->z_ki[i];
    if (c->z_kd[i]  >= 0  && c->z_kd[i]  <= 5000)        zone_kd[i]  = c->z_kd[i];
  }
  mc_zones_fix();

  /* Раскладку дисплея НЕ читаем из Flash: она живёт в прошивке.
     Поле в структуре осталось для совместимости смещений. */

  /* Пол по току (отсчёты АЦП) — поле дописано в конец структуры.
     Проверка диапазона: битая ячейка не должна задушить двигатель. */
  if (c->cl_min_abs >= 5 && c->cl_min_abs <= 3000)
    cl_min_abs = c->cl_min_abs;

  /* Тяговый режим — каждое значение с проверкой границ, иначе мусор
     из пустой Flash даст неуправляемый привод */
  /* Параметры тяги из Flash берём, а сам режим — нет: иначе после
     подачи питания привод сразу в тяге, и при ненулевой ручке PA3
     мотор тронется без команды. */
  tq_mode = 0;
  if (c->tq_cap  >= RPM_MIN && c->tq_cap <= RPM_MAX)   tq_rpm_cap = c->tq_cap;
  if (c->tq_band <= 2000u)                             tq_band    = c->tq_band;
  if (c->tq_dz   <= 500u)                              tq_dz      = c->tq_dz;
  if (c->tq_rate >= 1u && c->tq_rate <= 200u)          tq_rate    = c->tq_rate;
  if (c->tq_imax <= 3000u)                             tq_imax    = c->tq_imax;
  if (c->tq_floor<= 1000u)                             tq_floor   = c->tq_floor;
  if (c->tq_hold <= 600000u)                           tq_hold_ms = c->tq_hold;

  if (c->bt <= 5000u)   boost_time_ms = c->bt;   /* форсаж: 0=выкл, 100..5000 */

  /* Рабочий диапазон: проверяем и границы железа, и взаимный порядок —
     битая пара значений иначе обнулит размах задатчика. */
  if (c->rlo >= RPM_MIN && c->rhi <= RPM_MAX && c->rhi > c->rlo + 100)
  { rpm_lo = c->rlo; rpm_hi = c->rhi; }
}

/* Полный сброс настроек к заводским: стираем страницу конфигурации.
   Вызывается из главного цикла (не из прерывания USB) по флагу. */
void mc_cfg_reset(void)
{
  if (mc_inverter_on) return;

  FLASH_EraseInitTypeDef f = {0};
  uint32_t err = 0;
  f.TypeErase   = FLASH_TYPEERASE_PAGES;
  f.PageAddress = CFG_FLASH_ADDR;
  f.NbPages     = 1;

  HAL_FLASH_Unlock();
  HAL_FLASHEx_Erase(&f, &err);
  HAL_FLASH_Lock();

  /* После стирания применить все текущие дефолты прямо в RAM —
     иначе до перезагрузки будут жить старые значения. Простейший путь:
     сохранить заново, что запишет дефолты, — но проще вызвать defaultы. */
  mc_zones_default();
  cl_min_abs = 458;
  current_limit_set = CURRENT_LIMIT;
}

/* ==========================================================================
 *  АДАПТИВНЫЕ КОЭФФИЦИЕНТЫ
 *
 *  ИСПРАВЛЕН КОНФЛИКТ. Раньше Pid_AutoTune и Pid_RippleControl каждый кадр
 *  писали в одни и те же pid_kp/pid_kd: ripple добавлял +20 к Kd, autotune
 *  тут же тянул его назад к зональному значению. Оба были включены по
 *  умолчанию, поэтому коэффициенты жили своей жизнью и поведение
 *  не воспроизводилось.
 *
 *  Стало: единственный источник базы — зоны (или ручные значения), а
 *  ripple крутит МНОЖИТЕЛИ поверх базы. Они не спорят, а складываются.
 * ========================================================================== */
/* Зоны БОЛЬШЕ НЕ const: правятся с ПК командами Z<n><P|I|D|R><число>
   и сохраняются во Flash вместе с остальной конфигурацией. */
int32_t zone_rpm[ZONES_N] = {  150,  500, 1000, 2000, 4000, 6000 };
int32_t zone_kp[ZONES_N]  = {  200,  180,  240,  300,  350,  380 };
int32_t zone_ki[ZONES_N]  = {  100,   90,  110,  130,  140,  150 };
int32_t zone_kd[ZONES_N]  = {  120,  280,  260,  200,  180,  160 };

/* Значения «с завода» — для команды ZR */
static const int32_t zone_rpm_def[ZONES_N] = {  150,  500, 1000, 2000, 4000, 6000 };
static const int32_t zone_kp_def[ZONES_N]  = {  200,  180,  240,  300,  350,  380 };
static const int32_t zone_ki_def[ZONES_N]  = {  100,   90,  110,  130,  140,  150 };
static const int32_t zone_kd_def[ZONES_N]  = {  120,  280,  260,  200,  180,  160 };

void mc_zones_default(void)
{
  for (int i = 0; i < ZONES_N; i++)
  {
    zone_rpm[i] = zone_rpm_def[i]; zone_kp[i] = zone_kp_def[i];
    zone_ki[i] = zone_ki_def[i];   zone_kd[i] = zone_kd_def[i];
  }
}

/* Обороты зон обязаны строго возрастать: интерполяция делит на (r1-r0),
   и равные или убывающие узлы дали бы деление на ноль. */
void mc_zones_fix(void)
{
  for (int i = 0; i < ZONES_N; i++)
    zone_rpm[i] = clamp(zone_rpm[i], 50, RPM_MAX * 2);
  for (int i = 1; i < ZONES_N; i++)
    if (zone_rpm[i] <= zone_rpm[i - 1]) zone_rpm[i] = zone_rpm[i - 1] + 10;
}

static int32_t kp_base_fx = TO_FX(300), ki_base_fx = TO_FX(110), kd_base_fx = TO_FX(260);
int32_t kp_mul = 1000, kd_mul = 1000;              /* промилле, ripple */
static int32_t kp_act = 300, ki_act = 110, kd_act = 260;  /* действующие */

#define RIPPLE_WIN        12
#define RIPPLE_SPREAD_HI  120
#define RIPPLE_SPREAD_LO  30
#define RIPPLE_HOLD       25
static int32_t  ripple_win[RIPPLE_WIN] = {0};
static uint32_t ripple_idx = 0, ripple_hold = 0;

static void gains_update(int32_t rpm)
{
  if (pid_auto)
  {
    /* Интерполяция ВНУТРИ зоны, а не ступенькой: между узлами коэффициент
       меняется линейно, при переходе зоны нет скачка */
    int32_t a = (rpm < 0) ? -rpm : rpm;
    int i = 0;
    while (i < ZONES_N - 1 && a > zone_rpm[i]) i++;

    int32_t kp_t, ki_t, kd_t;
    if (i == 0 || a >= zone_rpm[ZONES_N - 1])
    {
      kp_t = zone_kp[i]; ki_t = zone_ki[i]; kd_t = zone_kd[i];
    }
    else
    {
      int32_t r0 = zone_rpm[i - 1], r1 = zone_rpm[i];
      int32_t w  = ((a - r0) << FX) / (r1 - r0);      /* 0..256 */
      kp_t = zone_kp[i-1] + (((zone_kp[i] - zone_kp[i-1]) * w) >> FX);
      ki_t = zone_ki[i-1] + (((zone_ki[i] - zone_ki[i-1]) * w) >> FX);
      kd_t = zone_kd[i-1] + (((zone_kd[i] - zone_kd[i-1]) * w) >> FX);
    }
    kp_base_fx = lp_fx(kp_base_fx, kp_t, 3);
    ki_base_fx = lp_fx(ki_base_fx, ki_t, 3);
    kd_base_fx = lp_fx(kd_base_fx, kd_t, 3);
  }
  else
  {
    /* Тот же фильтр, что в автомате: при переключении режима коэффициент
       не прыгает ступенькой, а переползает за ~10 тиков. Мотор не дёргается,
       но работает уже с ПОЛЬЗОВАТЕЛЬСКИМИ числами, а не с авто-снимком. */
    kp_base_fx = lp_fx(kp_base_fx, kp_user, 3);
    ki_base_fx = lp_fx(ki_base_fx, ki_user, 3);
    kd_base_fx = lp_fx(kd_base_fx, kd_user, 3);
  }

  /* На стоящем двигателе spread всегда 0, и ветка "мало пульсаций" гонит
     kp_mul к потолку 1600: коэффициенты уползают сами, ещё до пуска.
     Крутим множители только когда двигатель реально вращается. */
  if (pid_ripple && mc_inverter_on && rpm > 50)
  {
    if (ripple_hold > 0) ripple_hold--;
    else
    {
      ripple_win[ripple_idx] = (rpm < 0) ? -rpm : rpm;
      ripple_idx = (ripple_idx + 1u) % RIPPLE_WIN;

      int32_t mn = INT32_MAX, mx = 0;
      for (int i = 0; i < RIPPLE_WIN; i++)
      {
        if (ripple_win[i] < mn) mn = ripple_win[i];
        if (ripple_win[i] > mx) mx = ripple_win[i];
      }
      int32_t spread = mx - mn;

      /* Раскачка -> больше демпфирования, меньше усиления. И наоборот.
         Правим МНОЖИТЕЛИ, база остаётся за зонами/ручным режимом. */
      if (spread > RIPPLE_SPREAD_HI)
      {
        kd_mul += 60; kp_mul -= 60; ripple_hold = RIPPLE_HOLD;
      }
      else if (spread < RIPPLE_SPREAD_LO)
      {
        kd_mul -= 40; kp_mul += 40; ripple_hold = RIPPLE_HOLD;
      }
      kp_mul = clamp(kp_mul, 400, 1600);
      kd_mul = clamp(kd_mul, 400, 2500);
    }
  }
  else
  {
    /* Плавный возврат к 1.00: если бы просто обнулили, коэффициенты
       скакнули ступенькой и мотор дёрнулся бы на пуске. */
    kp_mul += (1000 - kp_mul) / 8;
    kd_mul += (1000 - kd_mul) / 8;
  }

  kp_act = (FROM_FX(kp_base_fx) * kp_mul) / 1000;
  ki_act =  FROM_FX(ki_base_fx);
  kd_act = (FROM_FX(kd_base_fx) * kd_mul) / 1000;

  /* Наружу отдаём действующие значения — тюнер на ПК видит правду.
     В ручном режиме НЕ перетираем то, что прислал пользователь.
     Было (pid_auto || pid_ripple): подавление пульсаций (по умолчанию
     включено) крутит множители kp_mul/kd_mul и записывало результат
     обратно в pid_kp/ki/kd даже при pid_auto=0 — в ручном режиме
     коэффициенты «уползали» сами собой, выглядело как «нажал Ручной,
     а работает автомат». Пульсации продолжают работать через множители,
     база остаётся пользовательской. */
  /* Наружу всегда отдаём ДЕЙСТВУЮЩИЕ коэффициенты — и в авто, и в ручном.
     Ручные при этом остаются в kp_user/ki_user/kd_user и не перетираются. */
  pid_kp = kp_act; pid_ki = ki_act; pid_kd = kd_act;
}

/* ==========================================================================
 *  ВНЕШНИЙ КОНТУР: СКОРОСТЬ -> УСТАВКА ТОКА
 *
 *  Выход — ТОК, а не ШИМ. В этом вся суть переделки: контур скорости
 *  просит момент, а выдать его или упереться в предел — дело внутреннего
 *  контура. Поэтому «ограничение пускового тока», «тяговый режим» и
 *  «ограничение момента с PA3» — одно и то же действие: клип i_max.
 *
 *  Масштабы: умножение ДО деления, 64 бита — целочисленных обнулений нет.
 *  Анти-виндап — back-calculation: интеграл копится только если выход не
 *  в упоре либо ошибка толкает обратно в диапазон.
 * ========================================================================== */
static int32_t speed_loop(int32_t sp, int32_t meas, int32_t i_max, uint32_t dt_us)
{
  int32_t err = sp - meas;

  /* D по сглаженной ошибке: шум квантования энкодера (60 имп/об) иначе
     колотит дифференциальную составляющую на малых оборотах */
  int32_t err_fx = TO_FX(err);
  sp_err_filt_fx += (err_fx - sp_err_filt_fx) >> (derr_filt_shift >= 1 ? derr_filt_shift : 1);
  int32_t d_fx = sp_err_filt_fx - sp_err_prev_fx;
  sp_err_prev_fx = sp_err_filt_fx;

  int32_t p_term = (int32_t)(((int64_t)err * kp_act * I_CNT_PER_A) / 1000000LL);
  int32_t i_term = (int32_t)(((int64_t)sp_integral_fx * ki_act * I_CNT_PER_A)
                             / 100000000LL);
  int32_t d_term = (int32_t)(((int64_t)FROM_FX(d_fx) * kd_act * I_CNT_PER_A)
                             / 1000000LL);

  int32_t out     = p_term + i_term + d_term;
  int32_t out_sat = clamp(out, 0, i_max);

  if ((out == out_sat) ||
      (out > out_sat && err < 0) ||
      (out < out_sat && err > 0))
  {
    /* интеграл в [об/мин * мс] */
    sp_integral_fx += (int32_t)(((int64_t)err * (int64_t)dt_us) / 1000LL);
    sp_integral_fx = clamp(sp_integral_fx, -2000000, 2000000);
  }
  return out_sat;
}

/* ==========================================================================
 *  ВНУТРЕННИЙ КОНТУР: ТОК -> ШИМ
 *
 *  ПИ-регулятор на каждом тике (~344 Гц). Именно он делает пусковой ток
 *  безопасным по построению: сколько бы ни просил контур скорости, ШИМ
 *  поднимается ровно до того уровня, при котором ток равен уставке.
 * ========================================================================== */
static uint32_t current_loop(int32_t i_ref, int32_t i_meas,
                             int32_t duty_max, uint32_t dt_us)
{
  int32_t err = i_ref - i_meas;

  int32_t p_term = (int32_t)(((int64_t)err * ci_kp * PWM_MAX)
                             / (1000LL * I_CNT_PER_A));
  int32_t i_term = (int32_t)(((int64_t)ci_integral_fx * ci_ki * PWM_MAX)
                             / (1000000LL * I_CNT_PER_A));

  int32_t out     = p_term + i_term;
  int32_t out_sat = clamp(out, 0, duty_max);

  if ((out == out_sat) ||
      (out > out_sat && err < 0) ||
      (out < out_sat && err > 0))
  {
    ci_integral_fx += (int32_t)(((int64_t)err * (int64_t)dt_us) / 1000LL);
    ci_integral_fx = clamp(ci_integral_fx, 0, 3000000);
  }
  return (uint32_t)out_sat;
}

/* ==========================================================================
 *  ШИМ
 * ========================================================================== */
static void pwm_apply(uint32_t promille)
{
  if (promille > PWM_MAX) promille = PWM_MAX;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (promille * PWM_ARR) / 1000u);
}

static void control_reset(void)
{
  sp_integral_fx = 0;
  sp_err_prev_fx = 0;
  sp_err_filt_fx = 0;
  ci_integral_fx = 0;
  enc_fault_us = 0;
  soft_start = 0;
  oc_frames = 0;
  rpm_ramp = 100;      /* стартуем всегда со 100, ползём вниз по уставке */
  mc_duty = 0;
  mc_i_ref = 0;
  kp_mul = 1000; kd_mul = 1000;
  ripple_hold = 0;
  for (int i = 0; i < RIPPLE_WIN; i++) ripple_win[i] = 0;
  tq_ref_ramp = 0;
  tq_hold_us  = 0;
  tq_ramp_us  = mc_us();
  pwm_apply(0);
}

/* ==========================================================================
 *  АВАРИЯ
 *
 *  Раньше и пик, и I^2t, и режим 0 проверялись раз в кадр (~40 мс). Для
 *  силового ключа это вечность. Теперь пик ловится внутри обработки
 *  половины буфера — задержка ~2.9 мс, причём по каждому отсчёту.
 *
 *  Замечание по железу (схему не меняем): полностью аппаратный срыв ШИМ
 *  даёт только TIM1_BKIN, у TIM3 входа BREAK нет. Если однажды захотите
 *  абсолютную защиту — перенос ШИМ на TIM1_CH1 и компаратор шунта на BKIN.
 *  Код к этому готов: pwm_apply() — единственная точка выхода на ключ.
 * ========================================================================== */
static void trip_set(mc_trip_t t)
{
  if (mc_trip == TRIP_NONE || t == TRIP_PEAK)
  {
    mc_trip = t;
    trip_time_ms = HAL_GetTick();
  }
  mc_duty = 0;
  pwm_apply(0);
  ci_integral_fx = 0;
  sp_integral_fx = 0;
  soft_start = 0;
}

/* Внешняя установка защёлки — для трипов, обнаруженных вне mc.c
   (например, сброс по watchdog, который виден только при старте) */
void mc_trip_set_external(mc_trip_t t) { trip_set(t); }

/* Аппаратная авария по входу TIM1_BKIN (компаратор шунта). Вызывается из
   TIM1_BRK_IRQHandler. Ключ уже погашен железом (MOE сброшен аппаратурой),
   здесь фиксируем защёлку и приводим ПО в состояние «снято». */
void mc_hw_trip(void)
{
  /* Не затираем уже активную программную аварию: если сначала сработал
     TRIP_PEAK/TRIP_THERMAL, а следом компаратор, в логе должен остаться
     первый (более информативный для подбора порогов) код. Аппаратный
     срыв пишется только когда аварий не было. */
if (mc_trip == TRIP_NONE) { mc_trip = TRIP_HW; trip_time_ms = HAL_GetTick(); }
  mc_inverter_on = 0;
  mc_duty    = 0;
  ci_integral_fx = 0;
  sp_integral_fx = 0;
  soft_start  = 0;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
}

/* ===== Сброс защёлки защиты (команда RS) =====
   Сбрасывает trip, тепловой интеграл и счётчики перегрузки.
   Без этого RS только снимал флаг, но thermal_integral оставался
   выше порога и защита срабатывала снова на следующем тике. */
void mc_trip_reset(void)
{
  mc_trip = TRIP_NONE;
  thermal_integral = 0;
  oc_frames    = 0;
  peak_frames  = 0;   /* иначе недобитый счётчик сработает сразу */
  tq_hold_us   = 0;   /* сброс таймера стоянки под током */
  tq_ref_ramp  = 0;   /* момент вернётся плавно, а не рывком */
  trip_time_ms = HAL_GetTick();
  /* После аппаратной аварии (TIM1_BKIN) MOE сброшен железом —
     возвращаем его командой reset. Если источник аварии устранён,
     включение кнопкой поднимется нормально. */
  __HAL_TIM_MOE_ENABLE(&htim1);
}

/* ==========================================================================
 *  ОБРАБОТКА ПОЛОВИНЫ БУФЕРА АЦП
 * ========================================================================== */
void mc_adc_half(void) { if (half_ready || full_ready) adc_overrun++; half_ready = 1; }
void mc_adc_full(void) { if (half_ready || full_ready) adc_overrun++; full_ready = 1; }

static void process_block(const volatile uint16_t *buf)
{
  const uint32_t n = ADC_HALF_SETS;
  int64_t sum_sq = 0;
  int32_t dc = 0;
  uint32_t pot_sum = 0, cl_sum = 0;
  int32_t peak_raw = 0;

  for (uint32_t k = 0; k < n; k++)
  {
    int32_t d = (int32_t)buf[k * 4 + 0] - (int32_t)buf[k * 4 + 1];
    d -= i_offset;                       /* вычитаем калиброванный ноль */
    diff[k] = (int16_t)d;
    dc += d;
    sum_sq += (int64_t)d * d;
    pot_sum += buf[k * 4 + 2];
    cl_sum  += buf[k * 4 + 3];

    int32_t ad = (d < 0) ? -d : d;
    if (ad > peak_raw) peak_raw = ad;
  }

  dc /= (int32_t)n;
  mc_pot_raw = pot_sum / n;
  uint32_t cl_raw = cl_sum / n;

  /* Фильтр тока в фикспойнте: без мёртвой зоны на малых значениях */
  i_filt_fx = lp_fx(i_filt_fx, dc, 1);
  mc_i_meas = FROM_FX(i_filt_fx);

  mc_rms_sq = (uint32_t)(sum_sq / (int64_t)n);
  mc_peak = (uint32_t)peak_raw;

  /* --- Калибровка нуля датчика тока при duty=0 и выключенном инверторе.
         Раньше смещение диффпары не измерялось вообще, и весь ток был
         сдвинут на постоянную величину. --- */
  if (!mc_inverter_on && mc_duty == 0)
  {
    i_off_acc += dc + i_offset;          /* накапливаем СЫРОЕ значение */
    if (++i_off_n >= 64u)
    {
      int32_t off = i_off_acc / 64;
      /* Смещение диффпары на исправном датчике — единицы-десятки отсчётов.
         Сотни означают, что усилитель в упоре или шунт оборван: такой
         «ноль» запоминать нельзя, иначе весь ток поедет на эту величину
         и токовые защиты будут считать не то. */
      if (off > -300 && off < 300) { i_offset = off; i_off_done = 1; }
      else                         { i_off_done = 0; }
      i_off_acc = 0; i_off_n = 0;
    }
  }
  else { i_off_acc = 0; i_off_n = 0; }

  /* --- Устав PA3 (момент), с нижним ограничением cl_min_abs (абсолютный пол по току).
       cl_min_abs — отсчёты АЦП, не процент: иначе при смене CL пол разъезжается
       и душит двигатель. Если пол оказался выше предела — limit не подпускает
       распил, clamp работает корректно. */
  int32_t clv = (int32_t)((cl_raw * current_limit_set) / ADC_FULL);
  /* В тяге с заданным TM ручка PA3 масштабируется на TM, а не на CL:
     иначе полный ход даёт ровно CL, и поднять выше CL нельзя. */
  int32_t tq_raw = clv;
  if (tq_mode && tq_imax)
    tq_raw = (int32_t)((cl_raw * (int32_t)tq_imax) / ADC_FULL);
  tq_pot_raw = (uint32_t)(tq_raw < 0 ? 0 : tq_raw);   /* до применения пола CN */
  int32_t cl_min = (int32_t)cl_min_abs;
  if (cl_min > (int32_t)current_limit_set) cl_min = current_limit_set;
  cl_from_pot = (uint32_t)clamp(clv, cl_min, current_limit_set);

  /* --- БЫСТРЫЙ ПИК: по мгновенному максимуму блока, не по среднему ---
     БЫЛО: срабатывание по одному отсчёту (peak_raw) — одиночная
     коммутационная иголка на шунте, невидимая в среднем токе, давала
     ложный TRIP_PEAK на холостом ходу. СТАЛО: 3 блока подряд —
     иголка отсекается, настоящее КЗ держится и ловится за единицы мс. */
  uint32_t peak_lim = current_limit_set * 2u;
  if (max_enable && mc_inverter_on && peak_raw >= (int32_t)peak_lim)
  {
    if (++peak_frames >= 10u) { trip_set(TRIP_PEAK); peak_frames = 0; }
  }
  else peak_frames = 0;
}

/* ==========================================================================
 *  КОНТУР СКОРОСТИ + ЗАЩИТЫ (каждый SPEED_LOOP_DIV-й тик)
 * ========================================================================== */
static void speed_and_limits(uint32_t dt_us)
{
  int32_t rpm = enc_read_rpm(dt_us);
  /* На низких оборотах данные и так редкие — дополнительное сглаживание
     только добавляет запаздывание. Ниже 500 об/мин фильтруем слабее. */
  uint32_t fsh = rpm_filt_shift >= 1u ? rpm_filt_shift : 1u;
  if (rpm < 500 && fsh > 1u) fsh = 1u;
  rpm_filt_fx = lp_fx(rpm_filt_fx, rpm, (int32_t)fsh);
  mc_rpm_meas = FROM_FX(rpm_filt_fx);

  /* По УСТАВКЕ, а не по измеренным оборотам: при набросе нагрузки вал
     просаживается, и выбор зоны по факту снижал Kp именно тогда, когда
     нужно вытаскивать обороты обратно (положительная обратная связь). */
  gains_update(mc_rpm_set);

  /* --- Действующий порог тока ------------------------------------------ */
  uint32_t now = HAL_GetTick();
  uint32_t lim;
  if (tq_mode)
  {
    lim = tq_imax ? tq_imax : current_limit_set;
  }
  else if (torque_limit)
  {
    if (mc_inverter_on && (now - on_tick) < START_LIMIT_TIME_MS)
      lim = current_limit_set;                /* окно пуска — полный момент */
    else
      lim = cl_from_pot;
  }
  else lim = current_limit_set;               /* TL0 — настройка из CL */

  /* Наброс нагрузки: пускаем ток до 1.6 длительного предела не дольше
     boost_time_ms (BT). Тепловая модель I^2t это учтёт — она уже есть.
     Только в режиме оборотов: в тяге предел задаёт TM, его не трогаем.
     Гистерезис: между форсажами пауза 1 с, иначе ток дёргается
     и I^2t копится до срыва.
     Форсаж включается ТОЛЬКО если мотор реально потребляет ток близко
     к пределу: на пустых качелях оборотов ошибка большая, а ток мал —
     подливать нечего, форсаж добавил бы раскачку. */
  int32_t err_abs = mc_rpm_set - mc_rpm_meas;
  if (boost_rest > 0)                      /* отдых после форсажа */
  {
    boost_rest = (boost_rest > dt_us) ? boost_rest - dt_us : 0;
  }
  else if (boost_time_ms > 0 && !tq_mode && err_abs > mc_rpm_set / 4 &&
           mc_i_meas > (int32_t)(lim * 7 / 10))
  {
    boost_us += dt_us;
    if (boost_us < boost_time_ms * 1000u) lim = (lim * 16u) / 10u;
    else { boost_us = 0; boost_rest = 1000000u; }   /* 1 с остыть */
  }
  else if (err_abs < mc_rpm_set / 10) boost_us = 0;
  current_limit = lim;

  /* --- Защита от разгона и от обрыва энкодера ---------------------------
     enc_fault_us сбрасывается ниже при живом rpm (rpm >= 10). Окно выросло
     до 3 с: при отрисовке дисплея цикл может подтормаживать, и устаревший
     ноль rpm не должен читаться как обрыв датчика. */
  if (mc_rpm_meas > (RPM_MAX + RPM_MAX / 5))
    trip_set(TRIP_OVERSPEED);

  /* Обрыв датчика: ШИМ есть, отклика нет. Но если при этом ток ВЫСОКИЙ —
     вал реально стоит (заклинило, перегруз), и это не датчик: для такого
     есть TRIP_ARM. Иначе медленный набор тока на неподвижном валу через
     3 с давал ложный TRIP_ENC на исправном энкодере. */
  if (!ol_mode && !tq_mode && mc_inverter_on && !soft_start &&
      mc_duty > (uint32_t)(PWM_MAX / 5) && mc_rpm_meas < 10 &&
      mc_i_meas < (int32_t)(current_limit / 2))
  {
    if (enc_fault_us == 0) enc_fault_us = now;
    else if ((now - enc_fault_us) > ENC_FAULT_MS) trip_set(TRIP_ENC);
  }
  else enc_fault_us = 0;

  /* --- Залипание датчика: заполнение под потолком, а обороты не растут.
        Было: антизалипание кольца не давало разгону перезаполнить буфер
        периодов, rpm залипал (в логе — 307 при росте duty вдвое), регулятор
        честно отрабатывал враньё и гнал мотор вразнос. Если после исправления
        кольца rpm снова встанет колом при высоком заполнении — это уже не
        антизалипание, а реальный обрыв/дребезг датчика, и лучше остановиться
        по защите, чем жечь мотор. */
  {
    static int32_t  rpm_prev = 0;
    static uint32_t stuck_us = 0;
    if (!ol_mode && !tq_mode && mc_inverter_on && mc_duty > (uint32_t)(PWM_MAX * 3 / 4))
    {
      int32_t d = mc_rpm_meas - rpm_prev;
      if (d < 0) d = -d;
      /* Порог 5, а не точное равенство: на низких оборотах фильтрованный
         rpm подолгу стоит на одном числе из-за грубости 60 PPR. */
      if (d < 5)
      {
        if (stuck_us == 0) stuck_us = now;
        else if ((now - stuck_us) > 1500000u) trip_set(TRIP_ENC);
      }
      else { stuck_us = 0; rpm_prev = mc_rpm_meas; }
    }
    else { stuck_us = 0; rpm_prev = mc_rpm_meas; }
  }

  /* --- Достоверность датчика тока ------------------------------------------
     Есть заметный ШИМ — ток ОБЯЗАН быть. Нулевой ток при работающем
     двигателе означает обрыв шунта или усилителя, а не лёгкую нагрузку.
     Без этой проверки обрыв датчика молча снимает ВСЕ токовые защиты
     (OC, I2t, PEAK), а контур тока видит «тока нет» и открывает ключ
     до упора.
     Порог и выдержка взяты с запасом: 0.05 А заведомо ниже тока холостого
     хода любого якоря под ШИМ, 0.5 с длиннее переходного процесса. */
  {
    static uint32_t i_dead_us = 0;
    const int32_t I_MIN_OK = 763 / 20;          /* 0.05 А в отсчётах */

    if (mc_inverter_on && !ol_mode &&
        mc_duty > (uint32_t)(PWM_MAX / 5) &&
        mc_i_meas < I_MIN_OK && mc_rms < (uint32_t)I_MIN_OK)
    {
      if (i_dead_us == 0) i_dead_us = now;
      else if ((now - i_dead_us) > 500000u)
      {
        /* Ток ноль и вал стоит — это обрыв ЯКОРЯ, а не датчика.
           Различаем, чтобы диагностика не врала. */
        if (mc_rpm_meas < 10) trip_set(TRIP_ARM);
        else                  trip_set(TRIP_ISENS);
      }
    }
    else i_dead_us = 0;

    /* Калибровка нуля не прошла: смещение неизвестно, значит неизвестен
       и ток. Работать с неизвестным током нельзя.
       Даём калибровке успеть набрать 64 блока после подачи питания:
       иначе пуск сразу после включения даёт ложный ISEN. */
    if (mc_inverter_on && !ol_mode && !i_off_done && HAL_GetTick() > 2000u)
      trip_set(TRIP_ISENS);
  }

  /* --- Потеря связи с ПК: управление идёт с ПК — обязаны видеть ПК ---
     Таймаут 5 с при пинге раз в секунду: пять пропущенных подряд
     считаются обрывом. Запас от случайной задержки USB. */
  if (rpm_virt_sw && mc_inverter_on && last_cmd_ms &&
      (HAL_GetTick() - last_cmd_ms) > 5000u)
  {
    trip_set(TRIP_COMM);
  }

  /* --- I^2t: тепловая модель. ТЕПЕРЬ С ЗАЩЁЛКОЙ.
         Порог масштабируется под действующий current_limit, иначе при
         поднятии лимита защита срабатывала бы намного позже (квадрат
         тока). current_limit учитывает и ручку PA3, и окно пуска. --- */
  if (thermal_enable)
  {
    int32_t ie = mc_i_meas - (int32_t)current_limit;
    if (ie > 0) thermal_integral += (uint32_t)(ie * ie) / 640u;
    else if (thermal_integral > 0) thermal_integral -= thermal_integral / 2560u;

    uint32_t th_max = (uint32_t)(((uint64_t)THERMAL_MAX_LIMIT
                       * current_limit * current_limit)
                       / ((uint64_t)CURRENT_LIMIT * CURRENT_LIMIT));
    if (thermal_integral > th_max)
      trip_set(TRIP_THERMAL);
  }
  else thermal_integral = 0;

  /* --- Режим 0/1: длительная перегрузка с авторестартом ----------------- */
  if (mc_inverter_on && !tq_mode && mc_trip == TRIP_NONE &&
      (current_prot_mode == 0 || current_prot_mode == 1))
  {
    /* Считаем по тикам контура скорости (~4.5 мс при блоке 0.28 мс и
       SL=16): 250 тиков это ~1.1 с, 1500 — ~6.7 с непрерывного
       превышения. Окно в 60 тиков (0.27 с) рубило бы на каждой работе
       «на пороге», которую TL как раз и задаёт. */
    uint32_t need = (current_prot_mode == 0) ? 250u : 1500u;
    /* «Ограничение момента» (TL) держит ток НА пороге — это штатный
       режим, а не авария. Трип только при выходе за 1.5 действующего
       порога (порог уже учитывает форсаж и CL от ручки). */
    uint32_t lim_oc = current_limit;
    if (torque_limit) lim_oc = (lim_oc * 3u) / 2u;
    if (mc_i_meas >= (int32_t)lim_oc)
    {
      if (++oc_frames > need) trip_set(TRIP_OC);
    }
    else oc_frames = 0;
  }

  /* --- Рампа уставки: строго раз в 40 мс, чтобы RU/RD/SS сохранили смысл */
  /* Границы сузили на ходу — подтягиваем уставку под новый потолок.
     Через рампу, а не рывком: мгновенное изменение уставки даёт удар
     тока и просадку, а RD для этого и существует. */
  if (rpm_virt_sw) {
    if (rpm_virt > rpm_hi) rpm_virt = rpm_hi;
    if (rpm_virt < rpm_lo) rpm_virt = rpm_lo;
  }
  if ((mc_us() - ramp_last_us) >= RAMP_PERIOD_US)
  {
    ramp_last_us = mc_us();

    rpm_target = rpm_virt_sw ? (int32_t)rpm_virt
                 : (int32_t)(rpm_lo + ((mc_pot_raw * (rpm_hi - rpm_lo)) / ADC_FULL));

    if (rpm_ramp + (int32_t)ramp_step_up < rpm_target)
      rpm_ramp += (int32_t)ramp_step_up;
    else if (rpm_ramp > rpm_target + (int32_t)ramp_step_down)
      rpm_ramp -= (int32_t)ramp_step_down;
    else
      rpm_ramp = rpm_target;
    rpm_ramp = clamp(rpm_ramp, 0, (int32_t)rpm_hi);

    if (mc_inverter_on && soft_start < (uint32_t)PWM_MAX)
      soft_start += ss_step;
  }
  mc_rpm_set = mc_inverter_on ? rpm_ramp : rpm_target;
}

/* ==========================================================================
 *  КНОПКА
 * ========================================================================== */
static void button_task(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t hw = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) ? 0u : 1u;  /* LOW = ВКЛ */
  uint32_t next;

  if (inverter_sw == 0)
  {
    if (hw != btn_raw_prev) { btn_deb_tick = now; btn_raw_prev = hw; }
    next = ((now - btn_deb_tick) > BTN_DEB_MS) ? hw : mc_inverter_on;
  }
  else
  {
    next = (inverter_sw == 1) ? 1u : 0u;
    btn_raw_prev = hw; btn_deb_tick = now;
  }

  if (next && !mc_inverter_on)
  {
    /* ПОЛНЫЙ СБРОС при каждом пуске. Раньше trac_duty, mode0_oc_cnt и
       thermal_integral были static и сохранялись между пусками: после
       срабатывания токового потолка следующий пуск начинался с нулевого
       разрешённого ШИМ, и мотор просто не трогался с места. */
    control_reset();
    thermal_integral = 0;
    on_tick = now;
    start_tick = now;
    /* защёлки: перегрузку по току и обрыв энкодера кнопка снимает
       (обрыв датчика — не износ, его осмысленно пробовать повторно).
       Стоянка под током — тоже не износ, а лимит времени — снимаем. */
    if (mc_trip == TRIP_OC || mc_trip == TRIP_ENC ||
        mc_trip == TRIP_TQ_HOLD) mc_trip = TRIP_NONE;
    /* Аппаратная авария по TIM1_BKIN тоже снимается кнопкой: ключ уже
       погашен железом, здесь возвращаем MOE (RS делал бы то же самое).
       MOE поднимается ТОЛЬКО при снятии TRIP_HW: если активна защёлка,
       которую кнопка не снимает (PEAK/THERMAL/OVERSPEED), выход остаётся
       закрытым — иначе состояние противоречиво: «авария есть, выход жив». */
    if (mc_trip == TRIP_HW)
    {
      mc_trip = TRIP_NONE;
      __HAL_TIM_MOE_ENABLE(&htim1);
    }
  }
  if (!next && mc_inverter_on)
  {
    control_reset();
  }
  mc_inverter_on = next;
}

/* ==========================================================================
 *  ТЕЛЕМЕТРИЯ — формат кадров сохранён, ПК-софт менять не нужно
 * ========================================================================== */
static char zbuf[256];

static void zones_dump(void)
{
  int n = snprintf(zbuf, sizeof(zbuf), "{\"zones\":[");
  for (int i = 0; i < ZONES_N && n > 0 && n < (int)sizeof(zbuf) - 24; i++)
    n += snprintf(zbuf + n, sizeof(zbuf) - n, "%s[%ld,%ld,%ld,%ld]",
                  i ? "," : "",
                  (long)zone_rpm[i], (long)zone_kp[i],
                  (long)zone_ki[i], (long)zone_kd[i]);
  n += snprintf(zbuf + n, sizeof(zbuf) - n, "]}\r\n");

  if (n > 0 && n < (int)sizeof(zbuf))
  {
    if (CDC_Transmit_FS((uint8_t *)zbuf, (uint16_t)n) == USBD_OK)
      mc_zone_dump_req = 0;      /* занят USB — повторим следующим тиком */
  }
}

static void telemetry(void)
{
  mc_rms = (uint32_t)sqrtf((float)mc_rms_sq);
  if (json_mode)
  {
    if (mc_zone_dump_req) { zones_dump(); return; }

    /* ИСПРАВЛЕН РАСЧЁТ НАПРЯЖЕНИЯ. Было: duty*UV*141/PWM_ARR/100 — но duty
       хранится в ПРОМИЛЛЕ, а делили на ARR=2400. При 90 % выходило ~121 В
       вместо ~292 В. Правильный делитель — 1000. */
    uint32_t motor_v = (uint32_t)(((uint64_t)mc_duty * line_voltage * 141ULL) / 1000ULL / 100ULL);

    int len = snprintf(sbuf[jsel], sizeof(sbuf[jsel]),
      "{\"cur\":%ld,\"rms\":%lu,\"pot\":%lu,\"rpm\":%ld,\"rpm_set\":%ld,"
      "\"limit\":%u,\"auto\":%u,\"rip\":%u,\"on\":%u,\"on_sw\":%u,\"cl\":%lu,\"cls\":%lu,"
      "\"cn\":%lu,"
      "\"fr\":%lu,\"fd\":%lu,\"sl\":%lu,"
      "\"tl\":%u,\"ss\":%lu,\"ru\":%lu,\"rd\":%lu,\"te\":%u,\"me\":%u,\"vs\":%u,"
      "\"vr\":%lu,\"rlo\":%lu,\"rhi\":%lu,\"prot\":%u,\"json\":%u,\"mv\":%lu,\"uv\":%lu,"
      "\"ol\":%u,\"olduty\":%lu,"
      "\"tq\":%u,\"tcap\":%lu,\"tband\":%lu,\"tdz\":%lu,"
      "\"trate\":%lu,\"timax\":%lu,\"tfloor\":%lu,\"thold\":%lu,"
      "\"iref\":%ld,\"duty\":%lu,\"trip\":%u,\"kp\":%ld,\"ki\":%ld,\"kd\":%ld,"
      "\"kpu\":%ld,\"kiu\":%ld,\"kdu\":%ld,\"kpm\":%ld,\"kdm\":%ld,"
      "\"cikp\":%ld,\"ciki\":%ld,\"ovr\":%lu,\"esp\":%lu,\"eduty\":%lu,"
      "\"eg\":%lu,\"eok\":%lu,"
      "\"bst\":%u,\"bt\":%lu,"
      "\"wtx\":%lu,\"wdr\":%lu,\"wrx\":%lu,\"won\":%u}\r\n",
      (long)mc_i_meas, (unsigned long)mc_rms, (unsigned long)mc_pot_raw,
      (long)mc_rpm_meas, (long)mc_rpm_set,
      (unsigned)(mc_trip != TRIP_NONE), (unsigned)pid_auto, (unsigned)pid_ripple,
      (unsigned)mc_inverter_on, (unsigned)inverter_sw,
      (unsigned long)current_limit, (unsigned long)current_limit_set, (unsigned long)cl_min_abs,
      (unsigned long)rpm_filt_shift, (unsigned long)derr_filt_shift, (unsigned long)speed_loop_div, (unsigned)torque_limit,
      (unsigned long)ss_step, (unsigned long)ramp_step_up, (unsigned long)ramp_step_down,
      (unsigned)thermal_enable, (unsigned)max_enable,
      (unsigned)rpm_virt_sw, (unsigned long)rpm_virt,
      (unsigned long)rpm_lo, (unsigned long)rpm_hi,
      (unsigned)current_prot_mode, (unsigned)json_mode,
      (unsigned long)motor_v, (unsigned long)line_voltage,
      (unsigned)ol_mode, (unsigned long)ol_duty,
      (unsigned)tq_mode, (unsigned long)tq_rpm_cap, (unsigned long)tq_band,
      (unsigned long)tq_dz, (unsigned long)tq_rate, (unsigned long)tq_imax,
      (unsigned long)tq_floor, (unsigned long)tq_hold_ms,
      (long)mc_i_ref, (unsigned long)mc_duty, (unsigned)mc_trip,
      (long)pid_kp, (long)pid_ki, (long)pid_kd,
      /* задание (kp_user...) и действующие множители ripple — с ними тюнер
         показывает и «что выставили», и «что реально работает» */
      (long)kp_user, (long)ki_user, (long)kd_user,
      (long)kp_mul, (long)kd_mul,
      (long)ci_kp, (long)ci_ki, (unsigned long)adc_overrun,
      (unsigned long)enc_spread_pct(), (unsigned long)mc_get_enc_duty(),
      (unsigned long)mc_get_enc_glitch(), (unsigned long)mc_get_enc_good(),
      (unsigned)(boost_us > 0 && boost_us < boost_time_ms * 1000u),
      (unsigned long)boost_time_ms,
      /* Диагностика Wi-Fi-моста: сколь долго канал справляется */
      (unsigned long)wifi_tx_frames, (unsigned long)wifi_tx_drops,
      (unsigned long)wifi_rx_cmds, (unsigned)mc_wifi_online());

    if (len > 0 && len < (int)sizeof(sbuf[0]))
    {
      /* Запоминаем, в КАКОЙ буфер сейчас положен свежий кадр.
         Дальше jsel может переключиться (или не переключиться —
         если USB занят), но cur уже зафиксирован. */
      uint8_t cur = jsel;

      /* Двойной буфер: CDC не перезапишет кадр, пока USB передаёт прошлый */
      if (CDC_Transmit_FS((uint8_t *)sbuf[cur], (uint16_t)len) == USBD_OK)
        jsel ^= 1u;

      /* Тот же кадр — в Wi-Fi модуль, но реже: WIFI_PERIOD_US вместо 40 мс.
         Берём ИМЕННО cur, а не sbuf[jsel^1]: jsel переключается только
         при успешной отправке по USB, и при отключённом кабеле выражение
         jsel^1 навсегда указывало бы на старый буфер — в Wi-Fi уходил бы
         замороженный кадр с длиной от нового. Кадр КОПИРУЕТСЯ внутри
         mc_wifi_send() в отдельный tx_buf, потому что DMA читает его
         асинхронно, а sbuf через 40 мс перезапишется.
         O-кадры (осциллограф) в Wi-Fi не шлём вообще — 2.3 МБ/с туда
         не влезет ни при каких настройках. */
      static uint32_t wifi_last_us = 0;
      if ((mc_us() - wifi_last_us) >= WIFI_PERIOD_US)
      {
        wifi_last_us = mc_us();
        mc_wifi_send(sbuf[cur], (uint16_t)len);
      }
    }
  }
  else
  {
    uint8_t *b = txbuf[txsel];             /* двойной буфер */
    txsel ^= 1u;

    /* dt на канал: 4 канала x (13.5+12.5) циклов при ADCCLK 12 МГц ≈ 9 мкс.
       В исходнике стояло 23 — ось времени в осциллографе на ПК врала. */
    int len = snprintf((char *)b, 128,
      "O,0,%u,%u,%ld,%lu,%lu,%lu,%ld,%ld,%u;",
      (unsigned)ADC_HALF_SETS, 9u,
      (long)mc_i_meas, (unsigned long)mc_rms, (unsigned long)mc_peak,
      (unsigned long)mc_pot_raw, (long)mc_rpm_meas, (long)mc_rpm_set,
      (unsigned)(mc_trip != TRIP_NONE));

    if (len > 0)
    {
      memcpy(b + len, diff, ADC_HALF_SETS * 2u);
      CDC_Transmit_FS(b, (uint16_t)(len + (int)(ADC_HALF_SETS * 2u)));
    }
  }
}

/* ==========================================================================
 *  ИНДИКАЦИЯ
 *
 *  Код неисправности читается по частоте вспышек, а счётчики качества
 *  датчиков — скользящие (обнуляются раз в секунду), поэтому одна помеха
 *  при включении больше не держит аварийную индикацию до перезагрузки.
 * ========================================================================== */
uint32_t mc_led_pattern(void)
{
  uint32_t now = HAL_GetTick();
  switch (mc_trip)
  {
    case TRIP_PEAK:      return 100u;
    case TRIP_THERMAL:   return 200u;
    case TRIP_ENC:       return 300u;
    case TRIP_OVERSPEED: return 175u;
    case TRIP_TQ_HOLD:   return 250u;
    case TRIP_OC:        return 500u;
    case TRIP_HW:        return 700u;   /* аппаратная авария по TIM1_BKIN */
    default: break;
  }
  if (enc_glitch_win > enc_good_win && enc_glitch_win > 10u) return 150u;
  if (pot_glitch_win > 5u) return 125u;
  if (mc_inverter_on) return (now % 2000u < 40u) ? 1000u : 0u;   /* «жив» */
  return 0u;
}

/* ==========================================================================
 *  ГЛАВНАЯ ЗАДАЧА
 * ========================================================================== */
void mc_task(void)
{
  uint8_t processed = 0;

  /* БЫЛО: один блок за вызов — всё, что пришло за время отрисовки
     экрана, терялось (ovr рос). СТАЛО: разбираем все готовые блоки
     подряд в цикле, пока буфер снова не освободится. */
  while (half_ready || full_ready)
  {
    const volatile uint16_t *blk;
    if (half_ready)      { half_ready = 0; blk = &mc_adc_buf[0]; }
    else                 { full_ready = 0; blk = &mc_adc_buf[ADC_BUF_LEN / 2]; }

    loop_alive_us = mc_us();
    processed = 1;

    uint32_t now_us = mc_us();
    uint32_t dt_us = BLOCK_DT_US;

    process_block(blk);
    button_task();

  /* --- статистика датчиков: скользящее окно 1 с --- */
  {
    int32_t delta = (int32_t)mc_pot_raw - (int32_t)pot_prev;
    int32_t dir = (delta > 30) ? 1 : ((delta < -30) ? -1 : 0);
    if (dir != 0 && pot_dir != 0 && dir != pot_dir) pot_glitch_win++;
    if (dir != 0) pot_dir = dir;
    pot_prev = mc_pot_raw;

    if ((now_us - enc_stat_us) > 1000000u)
    {
      enc_stat_us = now_us;
      enc_glitch_win = enc_glitch; enc_good_win = enc_good;
      enc_glitch = 0; enc_good = 0;
      pot_glitch_win = 0;
    }
  }

  /* --- контур скорости: каждый 4-й тик --- */
  if ((++tick_cnt % (speed_loop_div ? speed_loop_div : 4u)) == 0u)
    speed_and_limits(dt_us * (speed_loop_div ? speed_loop_div : 4u));

  /* ======================= ВЫРАБОТКА ШИМ ============================== */
  if (!mc_inverter_on)
  {
    control_reset();
    pot_zero_tick = 0;
  }
  else if (mc_trip != TRIP_NONE)
  {
    mc_duty = 0;
    pwm_apply(0);
    ci_integral_fx = 0;
    sp_integral_fx = 0;

    /* Авторестарт ТОЛЬКО для перегрузки по току в режиме 0. Пик, I^2t,
       обрыв энкодера и разгон — защёлки, снимаются командой RS или
       перезапуском кнопкой. */
    if (mc_trip == TRIP_OC && current_prot_mode == 0 &&
        (HAL_GetTick() - trip_time_ms) >= pause_time_ms)
    {
      mc_trip = TRIP_NONE;
      oc_frames = 0;
      soft_start = 0;
    }
  }
  else
  {
    /* ===== РАЗОМКНУТЫЙ КОНТУР =====================================
       Оба ПИД-контура пропускаем целиком: ШИМ идёт на ключ как есть.
       Интегралы держим в нуле, иначе при возврате в обычный режим они
       выстрелят накопленной за время отладки суммой. */
    if (ol_mode)
    {
      uint32_t d = ol_duty;

      if (ol_pot)   /* потенциометр -> ШИМ напрямую, 0..PWM_MAX */
        d = (mc_pot_raw * (uint32_t)PWM_MAX) / 4095u;

      if (d > (uint32_t)PWM_MAX) d = (uint32_t)PWM_MAX;

      /* Плавное нарастание оставляем: скачок ШИМ с нуля на полную —
         это бросок тока через ключ. */
      if (soft_start < (uint32_t)PWM_MAX && d > soft_start) d = soft_start;

      mc_duty = d;
      pwm_apply(mc_duty);

      ci_integral_fx = 0;
      sp_integral_fx = 0;
      mc_i_ref = 0;
      continue;          /* не return: мы внутри цикла по блокам АЦП */
    }

    /* Потенциометр в нуле — стоп (как было, с выдержкой 150 мс).
       В тяговом режиме смотрим на РУЧКУ ТОКА PA3: при отпущенной в ноль
       ручке оборотов (PA2) мотор не должен останавливаться — PA2 в тяге
       не участвует. */
    uint32_t pot_stop = tq_mode ? tq_pot_raw : mc_pot_raw;
    if (pot_stop < 50u)
    {
      if (pot_zero_tick == 0) pot_zero_tick = HAL_GetTick();
      if ((HAL_GetTick() - pot_zero_tick) > 150u)
      {
        mc_duty = 0; pwm_apply(0);
        sp_integral_fx = 0; ci_integral_fx = 0;
        continue;      /* не return: мы внутри цикла по блокам АЦП */
      }
    }
    else pot_zero_tick = 0;

    /* --- 1. внешний контур: сколько тока просим --- */
    int32_t i_max = (int32_t)current_limit;

    /* Режим 2 (тест): защиты отключены, но ток всё равно физически
       ограничен внутренним контуром — по построению, а не таймером */
    if (current_prot_mode == 2) i_max = (int32_t)(current_limit_set * 2u);

    if (tq_mode)
    {
      /* ---- ТЯГА: ток = ручка PA3, обороты свободны ----------------- */
      int32_t want = (int32_t)tq_pot_raw;

      /* свой предел момента: в тяге его обычно хотят выше */
      int32_t tq_lim = (int32_t)(tq_imax ? tq_imax : current_limit_set);
      if (want > tq_lim) want = tq_lim;

      /* мёртвая зона: шум потенциометра в нуле иначе дёргает вал */
      if (want < (int32_t)tq_dz) want = 0;
      else if (tq_floor && want < (int32_t)tq_floor)
        want = (int32_t)tq_floor;   /* ток трогания: сразу от порога */

      /* плавный сброс у потолка оборотов. Обрыв на потолке даёт рывок
         и раскачку, поэтому душим током за tq_band до него. */
      int32_t soft = (int32_t)tq_rpm_cap - (int32_t)tq_band;
      if (soft < 0) soft = 0;
      if (mc_rpm_meas > soft && tq_band > 0)
      {
        int32_t over = mc_rpm_meas - soft;
        int32_t k = 256 - (over * 256) / (int32_t)tq_band;
        if (k < 0) k = 0;
        want = (want * k) >> 8;
      }

      /* рампа тока — ограничение рывка. Строго раз в 40 мс, как RU/RD. */
      if ((mc_us() - tq_ramp_us) >= RAMP_PERIOD_US)
      {
        tq_ramp_us = mc_us();
        int32_t step = (int32_t)(tq_rate ? tq_rate : 15u);
        if (tq_ref_ramp < want)
          tq_ref_ramp = (want - tq_ref_ramp > step) ? tq_ref_ramp + step : want;
        else if (tq_ref_ramp > want)
          tq_ref_ramp = (tq_ref_ramp - want > step) ? tq_ref_ramp - step : want;
      }
      mc_i_ref = tq_ref_ramp;

      /* стоянка под током: мотор без обдува с полным током греется
         быстрее, чем это увидит I^2t. Отдельный честный счётчик. */
      if (tq_hold_ms && mc_rpm_meas < 30 && mc_i_meas > (int32_t)(tq_lim / 3))
      {
        if (tq_hold_us == 0) tq_hold_us = HAL_GetTick();
        else if ((HAL_GetTick() - tq_hold_us) > tq_hold_ms)
          trip_set(TRIP_TQ_HOLD);
      }
      else tq_hold_us = 0;

      sp_integral_fx = 0;   /* контур скорости спит — интеграл не копим */
    }
    else
    {
      mc_i_ref = speed_loop(mc_rpm_set, mc_rpm_meas, i_max,
                            dt_us * (speed_loop_div ? speed_loop_div : 4u));
      tq_ref_ramp = mc_i_meas;   /* бесшовный переход в тягу */
      tq_hold_us = 0;
    }

    /* --- 2. потолок ШИМ: soft-start + окно пуска --- */
    int32_t duty_max = PWM_MAX;
    if (soft_start < (uint32_t)PWM_MAX) duty_max = (int32_t)soft_start;

    if (START_PWM_MAX_PCT > 0 &&
        (HAL_GetTick() - start_tick) < START_PWM_TIME_MS)
    {
      int32_t sm = (PWM_MAX * (int32_t)START_PWM_MAX_PCT) / 100;
      if (duty_max > sm) duty_max = sm;
    }

    /* --- 3. внутренний контур: ШИМ под заданный ток --- */
    mc_duty = current_loop(mc_i_ref, mc_i_meas, duty_max, dt_us);
    pwm_apply(mc_duty);
  }
  }  /* конец while (half_ready || full_ready) */

  /* Сторож контура: ни одного блока не пришло (сбой АЦП/DMA) */
  if (!processed)
  {
    if (loop_alive_us && (mc_us() - loop_alive_us) > 100000u)
    {
      mc_duty = 0; pwm_apply(0);
    }
  }

  /* --- телеметрия строго раз в 40 мс, один раз за вызов --- */
  if ((mc_us() - telem_last_us) >= TELEM_PERIOD_US)
  {
    telem_last_us = mc_us();
    telemetry();
  }
}

/* ==========================================================================
 *  ИНИЦИАЛИЗАЦИЯ
 * ========================================================================== */
void mc_init(void)
{
  cfg_load();

  /* TIM2 — счётчик 1 МГц с прерыванием переполнения (расширение до 32 бит) */
  __HAL_RCC_TIM2_CLK_ENABLE();
  TIM2->PSC  = 71;
  TIM2->ARR  = 0xFFFF;
  TIM2->EGR  = TIM_EGR_UG;
  TIM2->SR   = 0;
  TIM2->DIER = TIM_DIER_UIE;
  TIM2->CR1  = TIM_CR1_CEN;
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  /* Кнопка PB4 — вход с подтяжкой, активный LOW (обрыв провода = ВЫКЛ) */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef g = {0};
  g.Pin  = GPIO_PIN_4;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &g);

  /* ПРИОРИТЕТЫ ПРЕРЫВАНИЙ. Раньше у всех стоял 0 и они конкурировали:
     USB мог задержать фронт энкодера и исказить измерение скорости.
       0 — энкодер (точность измерения времени важнее всего)
       1 — TIM2 overflow
       2 — DMA АЦП
       3 — USB (самый терпеливый) */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
  HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 3, 0);

  mc_inverter_on = 0;
  btn_raw_prev = 1;
  mc_trip = TRIP_NONE;
  control_reset();

  ramp_last_us  = mc_us();
  telem_last_us = mc_us();
  start_tick    = HAL_GetTick();

  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)mc_adc_buf, ADC_BUF_LEN);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  __HAL_TIM_MOE_ENABLE(&htim1);
  pwm_apply(0);

  /* Разрешить прерывание аппаратного срыва. NVIC уже разрешён в Msp, но
     без бита BIE в TIM1->DIER таймер не выставит запрос вообще: железо
     погасит ключ, а прошивка об этом не узнает — trip останется 0,
     светодиод не мигнёт кодом 700, а регулятор накрутит интеграл в
     закрытый выход (при снятии аварии — бросок тока). */
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK);

  /* Мост телеметрии в Wi-Fi модуль (USART3 + DMA). Ставится последним,
     чтобы основной контур уже был поднят; если модуль не подключён или
     не запитан — просто молчит, на управление не влияет. */
  mc_wifi_init();
}
