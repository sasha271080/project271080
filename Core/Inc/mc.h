/**
  ******************************************************************************
  * @file    mc.h
  * @brief   Ядро контроллера ДПТ: измерение, каскадное управление, защиты,
  *          конфигурация во Flash, телеметрия.
  *
  *  ЖЕЛЕЗО НЕ ИЗМЕНЕНО. Распиновка и периферия — ровно как в исходном проекте:
  *    PA0  ADC1_IN0  датчик тока ACPL-C790 Vout+   \ дифференциальная пара
  *    PA1  ADC1_IN1  датчик тока ACPL-C790 Vout-   / (шунт 0.075 Ом, x8.2)
  *    PA2  ADC1_IN2  переменный резистор — задание оборотов
  *    PA3  ADC1_IN3  переменный резистор — задание тока (момента)
  *    PB5  EXTI5     энкодер, 60 имп/оборот, внешний pull-up 10k
  *    PB12 GPIO in   кнопка ВКЛ/ВЫКЛ, активный LOW (обрыв = ВЫКЛ)
  *    PC13 GPIO out  светодиод Blue Pill (0 = горит)
  *    PB4  TIM3_CH1  ШИМ 30 кГц на драйвер ключа
  *    PA11/PA12      USB CDC (телеметрия и команды)
  *    TIM2           свободный счётчик 1 МГц -> расширен до 32 бит (мкс)
  *
  *  ЧТО ИЗМЕНЕНО — АЛГОРИТМ И СТРУКТУРА:
  *    1. Каскадное управление: внешний контур скорости задаёт УСТАВКУ ТОКА,
  *       внутренний контур тока задаёт ШИМ. Ограничение момента становится
  *       обычным клипом уставки тока, режимы 0/1/2/3 больше не дублируют
  *       логику регулятора.
  *    2. Обработка половины буфера АЦП (half/full callback) — снята гонка
  *       DMA с расчётом.
  *    3. Скорость: M/T-метод + потолок по времени с последнего фронта, с
  *       затуханием к нулю. Замирание rpm при остановке вала исключено.
  *    4. Все интеграторы с back-calculation анти-виндапом по фактическому ШИМ.
  *    5. Защёлки защит с явной причиной, быстрый пик по каждому отсчёту
  *       АЦП (2.9 мс вместо 40 мс).
  *    6. Запись Flash вынесена из прерывания USB в главный цикл.
  *    7. IWDG + сторож контура управления.
  ******************************************************************************
  */

#ifndef MC_H
#define MC_H

#include <stdint.h>

/* ==========================================================================
 *  1. КОНСТАНТЫ ЖЕЛЕЗА
 * ========================================================================== */

#define ADC_CH_N        4u
#define ADC_SETS        64u
#define ADC_BUF_LEN     (ADC_SETS * ADC_CH_N)
#define ADC_HALF_SETS   (ADC_SETS / 2u)
#define ADC_FULL        4095

#define BLOCK_DT_US     277u    /* длительность полубуфера АЦП, мкс */

#define PWM_ARR         2400u   /* 30 кГц при 72 МГц */
#define PWM_MAX         900     /* промилле: 90 % максимального заполнения */

#define ENC_PPR         60u     /* импульсов на оборот */
#define RPM_MAX         6000
#define RPM_MIN         30      /* было 100 — для намотки нужно ниже */

/* --- Пересчёт тока. Шунт 0.075 Ом, ACPL-C790 x8.2, Vref 3.3 В, 12 бит.
       1 А -> 75 мВ -> 615 мВ -> 763 отсчёта. */
#define I_CNT_PER_A     763
#define CURRENT_LIMIT   1526                 /* 2.00 А — для намотки нужен момент */

/* ==========================================================================
 *  ТЯГОВЫЙ РЕЖИМ (управление моментом)
 *  Ток задаётся ручкой PA3 напрямую, контур скорости отключён.
 *  Обороты — свободная величина, их определяет баланс момента и нагрузки.
 * ========================================================================== */
#define TQ_CAP_DEF      1500u   /* потолок оборотов — низкий по умолчанию */
#define TQ_BAND_DEF     300u    /* полоса плавного сброса тока до потолка  */
#define TQ_DZ_DEF       40u     /* мёртвая зона ручки, отсчёты АЦП        */
#define TQ_RATE_DEF     15u     /* темп набора тока, отсчёты за 40 мс     */
#define TQ_FLOOR_DEF    0u      /* ток трогания, 0 = выключено            */
#define TQ_HOLD_DEF     0u      /* таймаут стоянки под током, мс; 0 = off */

/* ==========================================================================
 *  2. ТЕМП РАБОТЫ КОНТУРОВ
 *
 *  Один набор из 4 каналов ~8.7 мкс -> половина буфера (32 набора) ~0.28 мс.
 *    - контур тока и защиты      : каждый тик, ~3.6 кГц
 *    - контур скорости           : каждый 16-й тик, ~225 Гц
 *    - рампа уставки и soft-start: строго 40 мс, чтобы смысл команд
 *                                  SS/RU/RD остался прежним
 *    - телеметрия                : 40 мс, как раньше (ПК-софт не трогаем)
 * ========================================================================== */
#define SPEED_LOOP_DIV      16u
/* Настраиваемые фильтры и делитель контура (команды FR/FD/SL):
   FR<n> — сдвиг фильтра оборотов (1..4), FD<n> — сдвиг фильтра D (1..4),
   SL<n> — делитель контура скорости (1..8). */
extern volatile uint32_t rpm_filt_shift;
extern volatile uint32_t derr_filt_shift;
extern volatile uint32_t speed_loop_div;
#define RAMP_PERIOD_US      40000u
#define TELEM_PERIOD_US     40000u

/* ==========================================================================
 *  3. ЗАЩИТЫ
 * ========================================================================== */
typedef enum
{
  TRIP_NONE      = 0,
  TRIP_OC        = 1,  /* длительная перегрузка (авторестарт в режиме 0) */
  TRIP_PEAK      = 2,  /* мгновенный пик (ME1)          — защёлка */
  TRIP_THERMAL   = 3,  /* I^2t, тепловая модель (TE1)   — защёлка */
  TRIP_ENC       = 4,  /* нет отклика энкодера под ШИМ  — защёлка */
  TRIP_OVERSPEED = 5,  /* разгон выше RPM_MAX + 20 %    — защёлка */
  TRIP_TQ_HOLD   = 6,  /* стоянка под током в тяге      — защёлка */
  TRIP_ISENS     = 7,  /* датчик тока недостоверен      — защёлка */
  TRIP_ARM       = 8,  /* обрыв якоря / щёток           — защёлка */
  TRIP_COMM      = 9,  /* потеря связи при работе от ПК — защёлка */
  TRIP_WDG       = 10, /* был сброс по watchdog         — защёлка */
  TRIP_HW        = 11  /* TIM1_BKIN: аппаратный срыв по компаратору шунта */
} mc_trip_t;

#define OVERLOAD_RETRY_MS   1000u
#define START_LIMIT_TIME_MS 2000u   /* окно полного порога тока после пуска */
#define START_PWM_MAX_PCT   30u     /* потолок ШИМ на старте, % от PWM_MAX  */
#define START_PWM_TIME_MS   800u
#define THERMAL_MAX_LIMIT   10000000u
#define ENC_FAULT_MS        3000u   /* ШИМ есть, отклика нет -> обрыв датчика */
#define ENC_TIMEOUT_US      300000u /* нет фронтов -> обороты строго 0        */
#define ENC_MIN_PULSE_US    50u     /* короче — дребезг                       */
#define ENC_MIN_WIDTH_US    15u     /* импульс уже — помеха, не прорезь      */
#define ENC_USE_CENTER      1       /* 1 = по центрам прорезей, 0 = старый   */
#define BTN_DEB_MS          30u

/* ==========================================================================
 *  4. ПУБЛИЧНОЕ СОСТОЯНИЕ (читает телеметрия, правят команды USB)
 * ========================================================================== */

/* --- задание и измерение --- */
extern volatile int32_t  mc_rpm_meas;
extern volatile int32_t  mc_rpm_set;
extern volatile int32_t  mc_i_meas;
extern volatile int32_t  mc_i_ref;
extern volatile uint32_t mc_duty;
extern volatile uint32_t mc_rms;
extern volatile uint32_t mc_peak;
extern volatile uint32_t mc_pot_raw;
extern volatile uint32_t mc_inverter_on;
extern volatile mc_trip_t mc_trip;

/* --- коэффициенты контура скорости (промилле) --- */
extern volatile int32_t pid_kp, pid_ki, pid_kd;
/* ручные коэффициенты — отдельно от действующих pid_kp/ki/kd */
extern volatile int32_t kp_user, ki_user, kd_user;
/* --- коэффициенты контура тока (промилле) — НОВОЕ, команды CP/CI --- */
extern volatile int32_t ci_kp, ci_ki;

/* --- Таблица адаптивных коэффициентов по оборотам --- */
#define ZONES_N 6
extern int32_t zone_rpm[ZONES_N];
extern int32_t zone_kp[ZONES_N];
extern int32_t zone_ki[ZONES_N];
extern int32_t zone_kd[ZONES_N];
extern volatile uint8_t mc_zone_dump_req;
void mc_zones_default(void);
void mc_zones_fix(void);

/* --- режимы --- */
extern volatile uint8_t  json_mode;
extern volatile uint8_t  current_prot_mode;
extern volatile uint8_t  pid_auto;
extern volatile uint8_t  pid_ripple;
extern volatile uint8_t  torque_limit;
extern volatile uint8_t  thermal_enable;
extern volatile uint8_t  max_enable;
extern volatile uint8_t  inverter_sw;
extern volatile uint8_t  rpm_virt_sw;
extern volatile uint32_t rpm_virt;
/* рабочий диапазон оборотов (RL/RH) — сужает RPM_MIN..RPM_MAX */
extern volatile uint32_t rpm_lo, rpm_hi;

/* --- параметры --- */
extern volatile uint32_t current_limit;
extern volatile uint32_t current_limit_set;
extern volatile uint32_t cl_min_abs;
extern volatile uint32_t pause_time_ms;
extern volatile uint32_t ss_step;
extern volatile uint32_t ramp_step_up;
extern volatile uint32_t ramp_step_down;
extern volatile uint32_t line_voltage;

/* --- режим разомкнутого контура (отладка) --- */
extern volatile uint8_t  ol_mode;
extern volatile uint32_t ol_duty;
extern volatile uint8_t  ol_pot;

/* --- тяговый режим (управление моментом) --- */
extern volatile uint8_t  tq_mode;      /* 1 — момент с PA3 */
extern volatile uint32_t tq_rpm_cap;   /* TC — потолок об/мин        */
extern volatile uint32_t tq_band;      /* TB — полоса сброса, об/мин */
extern volatile uint32_t tq_dz;        /* TZ — мёртвая зона ручки    */
extern volatile uint32_t tq_rate;      /* TR — темп набора тока      */
extern volatile uint32_t tq_imax;      /* TM — свой предел тока      */
extern volatile uint32_t tq_floor;     /* TF — ток трогания          */
extern volatile uint32_t tq_hold_ms;   /* TS — таймаут стоянки       */

/* --- форсаж (наброс нагрузки) --- */
extern volatile uint32_t boost_time_ms;  /* BT — длительность форсажа, мс */

/* --- запрос сохранения во Flash (выполняется в главном цикле) --- */
extern volatile uint8_t  mc_cfg_save_req;
extern volatile uint8_t  mc_cfg_reset_req;

/* --- признак живой связи с ПК (обновляется в mc_cmd_parse) --- */
extern volatile uint32_t last_cmd_ms;

/* ==========================================================================
 *  5. API
 * ========================================================================== */

void     mc_init(void);
void     mc_tim2_overflow(void);
void     mc_enc_edge(void);
void     mc_adc_half(void);
void     mc_adc_full(void);
void     mc_task(void);
uint32_t mc_us(void);
void     mc_cfg_save(void);
void     mc_cfg_reset(void);
void     mc_trip_set_external(mc_trip_t t);   /* трип извне (watchdog) */
void     mc_hw_trip(void);                     /* TIM1_BKIN — аппаратный срыв */
uint32_t mc_led_pattern(void);
void     mc_cmd_parse(const uint8_t *buf, uint32_t len);
void     mc_trip_reset(void);
uint32_t mc_get_adc_overrun(void);
int32_t  mc_get_enc_spread_pct(void);
uint32_t mc_get_enc_duty(void);
uint32_t mc_get_enc_glitch(void);
uint32_t mc_get_enc_good(void);

extern volatile uint16_t mc_adc_buf[ADC_BUF_LEN];

#endif /* MC_H */