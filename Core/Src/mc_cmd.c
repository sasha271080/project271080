/**
  ******************************************************************************
  * @file    mc_cmd.c
  * @brief   Разбор команд USB CDC.
  *
  *  ЧТО ИЗМЕНЕНО ПО СТРУКТУРЕ:
  *
  *  1. Раньше разбор был «лапшой» из 25 независимых if-ов по Buf[0]/Buf[1],
  *     которые проверялись ВСЕ и для каждой посылки. Побочный эффект: строка
  *     "RP" попадала и в ветку 'R','P', и в ветку Buf[0]=='P' (пауза), а
  *     одиночный символ длиной 1 читал Buf[1] за границей принятых данных.
  *     Теперь — точное сравнение префикса и один выход по первому совпадению.
  *
  *  2. Cfg_Save() больше НЕ вызывается из прерывания. Команда 'w' только
  *     поднимает флаг mc_cfg_save_req; запись идёт в главном цикле и лишь
  *     при остановленном двигателе.
  *
  *  3. Добавлены: RS (снять защёлку защиты), CP/CI (коэффициенты контура
  *     тока). Все старые команды сохранены буква-в-букву — софт на ПК
  *     менять не нужно.
  ******************************************************************************
  */

#include "main.h"
#include "mc.h"
#include "mc_wifi.h"
#include <string.h>

/* ==========================================================================
 *  Разбор беззнакового числа после префикса. Раньше парсер читал до *Len
 *  без проверки, что цифра вообще есть: "KP" без числа молча ставил 0 и
 *  убивал регулятор.
 * ========================================================================== */
static uint8_t parse_u32(const uint8_t *buf, uint32_t len, uint32_t skip, int32_t *out)
{
  if (len <= skip) return 0;
  int32_t v = 0;
  uint32_t digits = 0;
  for (uint32_t i = skip; i < len; i++)
  {
    if (buf[i] >= '0' && buf[i] <= '9')
    {
      v = v * 10 + (buf[i] - '0');
      digits++;
      if (v > 1000000) return 0;          /* мусор — не применяем */
    }
    else break;
  }
  if (digits == 0) return 0;
  *out = v;
  return 1;
}

static uint8_t match(const uint8_t *buf, uint32_t len, const char *pfx, uint32_t n)
{
  if (len < n) return 0;
  return (memcmp(buf, pfx, n) == 0) ? 1u : 0u;
}

/* ==========================================================================
 *  Точка входа: вызывается из CDC_Receive_FS
 * ========================================================================== */
void mc_cmd_parse(const uint8_t *buf, uint32_t len)
{
  int32_t v;
  last_cmd_ms = HAL_GetTick();   /* признак живой связи с ПК */

  /* --- отсекаем перевод строки, если терминал его добавил --- */
  while (len > 0 && (buf[len - 1] == '\r' || buf[len - 1] == '\n')) len--;
  if (len == 0) return;

  /* ====================== ДВУХБУКВЕННЫЕ КОМАНДЫ ======================
     Проверяются ПЕРВЫМИ: иначе "RP" перехватила бы однобуквенная ветка */

  if (match(buf, len, "ON", 2)) { inverter_sw = 1; return; }
  if (match(buf, len, "OF", 2)) { inverter_sw = 2; return; }
  if (match(buf, len, "HB", 2)) { inverter_sw = 0; return; }
  if (match(buf, len, "RP", 2)) { rpm_virt_sw = 0; return; }

  /* PG — пинг от ПК. Ничего не делает: нужен только чтобы обновить
     last_cmd_ms в начале функции. Телеметрию прошивка отдаёт сама, без
     запросов, поэтому иначе признака живой связи у неё нет — TRIP_COMM
     срабатывал бы на исправном кабеле, пока человек ничего не трогает. */
  if (match(buf, len, "PG", 2)) return;

  /* RS — снять защёлку защиты (пик / I^2t / энкодер / разгон).
     НОВОЕ: раньше эти состояния снимались только сбросом питания. */
  if (match(buf, len, "RS", 2)) { mc_trip_reset(); return; }

  /* CF — полный сброс настроек к заводским: стереть Flash-конфиг.
     Выполняется в главном цикле (стирание страницы долгое). */
  if (match(buf, len, "CF", 2)) { mc_cfg_reset_req = 1; return; }

  /* WE1/WE0 — включить/выключить мост телеметрии в Wi-Fi модуль.
     По умолчанию включён (wifi_enable=1). Выключение останавливает
     и передачу кадров, и приём команд с беспроводного порта. */
  if (match(buf, len, "WE", 2) && len >= 3) { wifi_enable = (buf[2] == '1'); return; }

  if (match(buf, len, "TE", 2) && len >= 3) { thermal_enable = (buf[2] == '1'); return; }
  if (match(buf, len, "ME", 2) && len >= 3) { max_enable     = (buf[2] == '1'); return; }
  if (match(buf, len, "TL", 2) && len >= 3) { torque_limit   = (buf[2] == '1'); return; }

  /* BTxxx — длительность форсажа при набросе нагрузки, мс (0=выкл, 100..5000) */
  if (match(buf, len, "BT", 2) && parse_u32(buf, len, 2, &v))
  { if (v == 0 || (v >= 100 && v <= 5000)) boost_time_ms = (uint32_t)v; return; }

  /* SSxxx — «цифровой конденсатор»: скорость нарастания ШИМ */
  if (match(buf, len, "SS", 2) && parse_u32(buf, len, 2, &v))
  { if (v >= 1 && v <= 100) ss_step = (uint32_t)v; return; }

  /* RUxxx / RDxxx — скорость разгона и торможения уставки */
  if (match(buf, len, "RU", 2) && parse_u32(buf, len, 2, &v))
  { if (v >= 1 && v <= 100) ramp_step_up = (uint32_t)v; return; }

  if (match(buf, len, "RD", 2) && parse_u32(buf, len, 2, &v))
  { if (v >= 1 && v <= 100) ramp_step_down = (uint32_t)v; return; }

  /* UVxxx — напряжение сети для пересчёта напряжения на двигателе */
  if (match(buf, len, "UV", 2) && parse_u32(buf, len, 2, &v))
  { if (v >= 100 && v <= 300) line_voltage = (uint32_t)v; return; }

  /* RVxxx — виртуальный задатчик оборотов. Границы — рабочий диапазон
     RL..RH, а не потолок железа: задание вне него прошивка молча
     отвергнет, и человек не поймёт, почему обороты не поехали. */
  if (match(buf, len, "RV", 2) && parse_u32(buf, len, 2, &v))
  {
    if (v >= (int32_t)rpm_lo && v <= (int32_t)rpm_hi) { rpm_virt = (uint32_t)v; rpm_virt_sw = 1; }
    return;
  }

  /* RL — нижняя граница рабочего диапазона, RH — верхняя.
     Проверяем и по границам железа, и по взаимному порядку: RL не может
     оказаться выше RH, иначе задатчик даст отрицательный размах. */
  if (match(buf, len, "RL", 2) && parse_u32(buf, len, 2, &v))
  {
    if (v >= RPM_MIN && v <= (int32_t)rpm_hi - 100) { rpm_lo = (uint32_t)v; return; }
  }
  if (match(buf, len, "RH", 2) && parse_u32(buf, len, 2, &v))
  {
    if (v <= RPM_MAX && v >= (int32_t)rpm_lo + 100) { rpm_hi = (uint32_t)v; return; }
  }

  /* OL1 / OL0 — включить и выключить разомкнутый контур.
     При выходе ШИМ обнуляется: иначе остаточное задание подхватил бы
     обычный режим и мотор дёрнулся бы на пуске. */
  if (match(buf, len, "OL", 2) && len >= 3)
  {
    if (buf[2] == '1') { ol_mode = 1; ol_duty = 0; ol_pot = 0; }
    else               { ol_mode = 0; ol_duty = 0; }
    return;
  }

  /* OPxxx — задание ШИМ напрямую, в промилле (0..900, 900 = 90 %).
     Действует только в разомкнутом режиме. */
  if (match(buf, len, "OP", 2) && parse_u32(buf, len, 2, &v))
  {
    if (v >= 0 && v <= PWM_MAX) { ol_duty = (uint32_t)v; ol_pot = 0; }
    return;
  }

  /* OT1 / OT0 — отдать задание ШИМ потенциометру или вернуть терминалу */
  if (match(buf, len, "OT", 2) && len >= 3) { ol_pot = (buf[2] == '1'); return; }

  /* ================== ТЯГОВЫЙ РЕЖИМ ==================
     TQ1/TQ0 — включить/выключить управление моментом.
     Взаимоисключающ с OL: там контуров нет вообще. */
  if (match(buf, len, "TQ", 2) && len >= 3)
  {
    if (buf[2] == '1') { tq_mode = 1; ol_mode = 0; ol_duty = 0; }
    else                 tq_mode = 0;
    return;
  }

  if (match(buf, len, "TC", 2) && parse_u32(buf, len, 2, &v))
  { if (v >= RPM_MIN && v <= RPM_MAX) tq_rpm_cap = (uint32_t)v; return; }

  if (match(buf, len, "TB", 2) && parse_u32(buf, len, 2, &v))
  { if (v >= 0 && v <= 2000) tq_band = (uint32_t)v; return; }

  if (match(buf, len, "TZ", 2) && parse_u32(buf, len, 2, &v))
  { if (v >= 0 && v <= 500) tq_dz = (uint32_t)v; return; }

  if (match(buf, len, "TR", 2) && parse_u32(buf, len, 2, &v))
  { if (v >= 1 && v <= 200) tq_rate = (uint32_t)v; return; }

  if (match(buf, len, "TM", 2) && parse_u32(buf, len, 2, &v))
  { if (v >= 0 && v <= 3000) tq_imax = (uint32_t)v; return; }

  if (match(buf, len, "TF", 2) && parse_u32(buf, len, 2, &v))
  { if (v >= 0 && v <= 1000) tq_floor = (uint32_t)v; return; }

  if (match(buf, len, "TS", 2) && parse_u32(buf, len, 2, &v))
  { if (v >= 0 && v <= 600000) tq_hold_ms = (uint32_t)v; return; }

  /* CPxxx / CIxxx — контур ТОКА. */
  if (match(buf, len, "CP", 2) && parse_u32(buf, len, 2, &v))
  { if (v > 0 && v <= 5000) ci_kp = v; return; }

  if (match(buf, len, "CI", 2) && parse_u32(buf, len, 2, &v))
  { if (v >= 0 && v <= 5000) ci_ki = v; return; }

  /* CLxxxx — предел тока в отсчётах АЦП (763 = 1.00 А, шунт 0.075 Ом).
     НОВОЕ: без этой команды шунт и лимит менялись только перепрошивкой. */
  if (match(buf, len, "CL", 2) && parse_u32(buf, len, 2, &v))
  { if (v >= 50 && v <= 3000) current_limit_set = (uint32_t)v; return; }

  /* CNxxx — пол току (отсчёты АЦП, 5..current_limit_set). Абсолютный минимум,
     до которого PA3 опускает порог. В отличие от CL — не процент, а отсчёты:
     иначе пол разъезжался при каждой смене предела. */
  if (match(buf, len, "CN", 2) && parse_u32(buf, len, 2, &v))
  { if (v >= 5 && v <= (int32_t)current_limit_set) cl_min_abs = (uint32_t)v; return; }

  /* ================= ТАБЛИЦА АДАПТИВНЫХ КОЭФФИЦИЕНТОВ =================
     ZD          — выдать таблицу на ПК
     ZR          — вернуть заводские значения
     Z<n>P<v>    — Kp зоны n (n = 0..5)
     Z<n>I<v>    — Ki зоны n
     Z<n>D<v>    — Kd зоны n
     Z<n>R<v>    — обороты узла n
     Пример: Z3P280 — в зоне 3 поставить Kp = 280 */
  if (match(buf, len, "ZD", 2)) { mc_zone_dump_req = 1; return; }
  if (match(buf, len, "ZR", 2)) { mc_zones_default(); return; }

  if (buf[0] == 'Z' && len >= 4 && buf[1] >= '0' && buf[1] <= '9')
  {
    uint32_t n = (uint32_t)(buf[1] - '0');
    if (n >= ZONES_N) return;
    if (!parse_u32(buf, len, 3, &v)) return;

    switch (buf[2])
    {
      case 'P': if (v >= 0 && v <= 5000) zone_kp[n] = v; break;
      case 'I': if (v >= 0 && v <= 5000) zone_ki[n] = v; break;
      case 'D': if (v >= 0 && v <= 5000) zone_kd[n] = v; break;
      case 'R':
        if (v >= 50 && v <= RPM_MAX * 2) { zone_rpm[n] = v; mc_zones_fix(); }
        break;
      default: break;
    }
    return;
  }

  /* FR<n> — сдвиг фильтра оборотов (1..4), FD<n> — сдвиг фильтра D (1..4),
     SL<n> — делитель контура скорости (1..8). Настройка на живом моторе. */
  if (match(buf, len, "FR", 2) && parse_u32(buf, len, 2, &v))
  { if (v >= 1 && v <= 4) rpm_filt_shift = (uint32_t)v; return; }

  if (match(buf, len, "FD", 2) && parse_u32(buf, len, 2, &v))
  { if (v >= 1 && v <= 4) derr_filt_shift = (uint32_t)v; return; }

  if (match(buf, len, "SL", 2) && parse_u32(buf, len, 2, &v))
  { if (v >= 1 && v <= 8) speed_loop_div = (uint32_t)v; return; }

  /* KPxxx / KIxxx / KDxxx — контур СКОРОСТИ */
  if (buf[0] == 'K' && len >= 3 && parse_u32(buf, len, 2, &v))
  {
    if (v < 0 || v > 5000) return;
    if      (buf[1] == 'P') kp_user = v;
    else if (buf[1] == 'I') ki_user = v;
    else if (buf[1] == 'D') kd_user = v;
    else return;
    /* Ручные коэффициенты имеют смысл только при выключенной автоподстройке,
       иначе их тут же перетирали зоны. */
    pid_auto = 0;
    return;
  }

  /* Pxxx — пауза после срабатывания токовой защиты */
  if (buf[0] == 'P' && parse_u32(buf, len, 1, &v))
  { if (v >= 100 && v <= 30000) pause_time_ms = (uint32_t)v; return; }

  /* ====================== ОДНОБУКВЕННЫЕ КОМАНДЫ ====================== */
  if (len == 1)
  {
    switch (buf[0])
    {
      case 'j': json_mode = 1; break;
      case 'o': json_mode = 0; break;
      case '0': current_prot_mode = 0; break;   /* рабочий  */
      case '1': current_prot_mode = 1; break;   /* отладка  */
      case '2': current_prot_mode = 2; break;   /* тест     */
      case '3': current_prot_mode = 3; break;   /* тяговый  */
      case 'a': pid_auto = 1; break;
      case 'm': pid_auto = 0; break;
      case 'r': pid_ripple = 1; break;
      case 's': pid_ripple = 0; break;
      case 'w': mc_cfg_save_req = 1; break;     /* НЕ пишем Flash из ISR */
      default: break;
    }
  }
}