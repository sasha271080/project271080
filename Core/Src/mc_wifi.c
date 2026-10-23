/**
  ******************************************************************************
  * @file    mc_wifi.c
  * @brief   Мост телеметрии в ESP8266 (D1 mini) через USART3 + DMA.
  ******************************************************************************
  */
#include "main.h"
#include "mc.h"
#include "mc_wifi.h"
#include <string.h>

UART_HandleTypeDef huart3;
static DMA_HandleTypeDef hdma_usart3_tx;
static DMA_HandleTypeDef hdma_usart3_rx;

volatile uint8_t  wifi_enable     = 1;
volatile uint32_t wifi_tx_frames  = 0;
volatile uint32_t wifi_tx_drops   = 0;
volatile uint32_t wifi_rx_cmds    = 0;
volatile uint32_t wifi_last_rx_ms = 0;

/* Приём — кольцевой буфер, DMA крутит его по кругу без остановки.
   Так мы не теряем байты между вызовами mc_wifi_task(). */
static uint8_t  rx_ring[WIFI_RX_RING];
static uint16_t rx_tail = 0;

/* Сборка команды из потока: копим до '\n' или '\r'. */
static uint8_t  cmd_buf[64];
static uint16_t cmd_len = 0;

/* Буфер передачи: DMA читает его асинхронно, поэтому он ОТДЕЛЬНЫЙ от
   sbuf телеметрии — иначе следующий кадр затрёт ещё не ушедший. */
static char     tx_buf[1024];
static volatile uint8_t tx_busy = 0;

/* ==========================================================================
 *  ИНИЦИАЛИЗАЦИЯ
 * ========================================================================== */
void mc_wifi_init(void)
{
  __HAL_RCC_USART3_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};

  /* PB10 — TX: альтернативная функция, push-pull */
  g.Pin   = GPIO_PIN_10;
  g.Mode  = GPIO_MODE_AF_PP;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &g);

  /* PB11 — RX: вход с подтяжкой. Подтяжка важна: если модуль не
     подключён или не запитан, линия не болтается и приёмник не
     ловит шум как поток мусорных байт. */
  g.Pin  = GPIO_PIN_11;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &g);

  huart3.Instance          = USART3;
  huart3.Init.BaudRate     = WIFI_BAUD;
  huart3.Init.WordLength   = UART_WORDLENGTH_8B;
  huart3.Init.StopBits     = UART_STOPBITS_1;
  huart3.Init.Parity       = UART_PARITY_NONE;
  huart3.Init.Mode         = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();

  /* --- DMA передачи: USART3_TX = DMA1_Channel2 --- */
  hdma_usart3_tx.Instance                 = DMA1_Channel2;
  hdma_usart3_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_usart3_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_usart3_tx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart3_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_usart3_tx.Init.Mode                = DMA_NORMAL;
  hdma_usart3_tx.Init.Priority            = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK) Error_Handler();
  __HAL_LINKDMA(&huart3, hdmatx, hdma_usart3_tx);

  /* --- DMA приёма: USART3_RX = DMA1_Channel3, кольцевой режим --- */
  hdma_usart3_rx.Instance                 = DMA1_Channel3;
  hdma_usart3_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_usart3_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_usart3_rx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart3_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_usart3_rx.Init.Mode                = DMA_CIRCULAR;
  hdma_usart3_rx.Init.Priority            = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK) Error_Handler();
  __HAL_LINKDMA(&huart3, hdmarx, hdma_usart3_rx);

  /* ПРИОРИТЕТЫ. Wi-Fi — самый терпеливый потребитель в системе:
       0 — энкодер и TIM1_BRK (аварийный срыв)
       1 — TIM2 overflow
       2 — DMA АЦП
       3 — USB
       3 — USART3 и его DMA  <- здесь
     Телеметрия может опоздать на миллисекунду, измерение скорости — нет. */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  HAL_NVIC_SetPriority(USART3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);

  /* Приём стартуем один раз и навсегда — кольцо крутится само */
  HAL_UART_Receive_DMA(&huart3, rx_ring, WIFI_RX_RING);
  rx_tail = 0;
}

/* ==========================================================================
 *  ПЕРЕДАЧА
 * ========================================================================== */
uint8_t mc_wifi_send(const char *buf, uint16_t len)
{
  if (!wifi_enable) return 0;              /* мост выключен — «не отправлено» */
  if (len == 0 || len > sizeof(tx_buf)) return 0;

  /* Предыдущий кадр ещё не ушёл — ПРОПУСКАЕМ текущий.
     Именно пропускаем, а не ждём: ждать в главном цикле нельзя,
     там крутится управление двигателем. Счётчик drops покажет,
     что канал не справляется. */
  if (tx_busy) { wifi_tx_drops++; return 0; }

  memcpy(tx_buf, buf, len);
  tx_busy = 1;

  if (HAL_UART_Transmit_DMA(&huart3, (uint8_t *)tx_buf, len) != HAL_OK)
  {
    tx_busy = 0;
    wifi_tx_drops++;
    return 0;
  }
  wifi_tx_frames++;
  return 1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3) tx_busy = 0;
}

/* ==========================================================================
 *  ПРИЁМ
 *
 *  Разбираем кольцо в главном цикле. Команды те же, что по USB, —
 *  mc_cmd_parse() общий, поэтому ПК и телефон управляют одинаково.
 * ========================================================================== */
static void rx_process(void)
{
  /* Голова кольца = сколько DMA уже положил */
  uint16_t head = (uint16_t)(WIFI_RX_RING - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx));

  while (rx_tail != head)
  {
    uint8_t c = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1u) % WIFI_RX_RING);

    if (c == '\r' || c == '\n')
    {
      if (cmd_len > 0)
      {
        mc_cmd_parse(cmd_buf, cmd_len);
        wifi_rx_cmds++;
        wifi_last_rx_ms = HAL_GetTick();
        cmd_len = 0;
      }
    }
    else if (cmd_len < sizeof(cmd_buf))
    {
      cmd_buf[cmd_len++] = c;
    }
    else
    {
      cmd_len = 0;     /* мусор длиннее буфера — сбрасываем */
    }
  }
}

/* ==========================================================================
 *  ВОССТАНОВЛЕНИЕ ПОСЛЕ ОШИБКИ
 *
 *  Помеха на линии поднимает флаг ORE/FE/NE, и приёмник ВСТАЁТ: DMA
 *  перестаёт получать байты, связь молча умирает до перезагрузки.
 *  Рядом силовой ключ на 30 кГц, так что это не теория. Перезапускаем.
 * ========================================================================== */
static void rx_recover(void)
{
  uint32_t sr = USART3->SR;
  if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))
  {
    (void)USART3->SR;
    (void)USART3->DR;                       /* чтение снимает флаги */
    HAL_UART_DMAStop(&huart3);
    HAL_UART_Receive_DMA(&huart3, rx_ring, WIFI_RX_RING);
    rx_tail = 0;
    cmd_len = 0;
  }
}

void mc_wifi_task(void)
{
  if (!wifi_enable) return;
  rx_recover();
  rx_process();
}

uint8_t mc_wifi_online(void)
{
  return (wifi_last_rx_ms != 0u &&
          (HAL_GetTick() - wifi_last_rx_ms) < 5000u) ? 1u : 0u;
}