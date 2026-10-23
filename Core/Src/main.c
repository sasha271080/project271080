/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Контроллер коллекторного двигателя постоянного тока
  *
  *  РАСПИНОВКА ИТОГОВОЙ ПЛАТЫ (аппаратная защита TIM1_BKIN):
  *    PA0/PA1  ADC1_IN0/IN1  датчик тока ACPL-C790 (шунт 0.075 Ом, x8.2)
  *    PA2      ADC1_IN2      переменный резистор — обороты
  *    PA3      ADC1_IN3      переменный резистор — ток (момент)
  *    PB5      EXTI5         энкодер 60 имп/об, внешний pull-up 10k
  *    PA8      TIM1_CH1      ШИМ 30 кГц  (было PB4/TIM3)
  *    PA6      TIM1_BKIN     аварийный вход от компаратора, активный LOW
  *                            (частичный ремап TIM1: BKIN с PB12 → PA6)
  *    PB4      GPIO          кнопка ВКЛ/ВЫКЛ (переехала с PB12)
  *    PC13     GPIO          светодиод
  *    PA11/12  USB CDC       телеметрия и команды
  *
  *  ВСЯ ЛОГИКА ВЫНЕСЕНА В mc.c/mc.h — каскадная структура
  *  «скорость -> ток -> ШИМ». Здесь остались только инициализация
  *  периферии (без правок) и главный цикл.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mc.h"
#include "mc_wifi.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
static void MX_IWDG_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* --- Колбэки периферии: только переадресация в ядро --------------------- */

/* Энкодер. Расчёт оборотов здесь больше НЕ делается: в прерывании только
   накопление импульсов и времени, чтобы прерывание было коротким. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_5) mc_enc_edge();
}

/* Половина буфера АЦП готова. НОВОЕ: обработка по половинам снимает гонку
   с циклическим DMA — раньше DMA писал в буфер, пока его же читали. */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  (void)hadc;
  mc_adc_half();
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  (void)hadc;
  mc_adc_full();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  /* Инициализация периферии — без изменений относительно исходного проекта */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  /* Ядро: TIM2, кнопка, приоритеты прерываний, загрузка настроек,
     старт АЦП+DMA и ШИМ с нулевым заполнением */
  mc_init();

  /* Сторожевой таймер ставится ПОСЛЕ инициализации: устройство держит
     силовой ключ, поэтому зависание процессора должно приводить к сбросу,
     а не к залипшему ШИМ. LSI 40 кГц / 64, reload 1000 -> таймаут ~1.6 с. */
  MX_IWDG_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN 3 */

    /* Всё управление: измерение, контуры, защиты, телеметрия */
    mc_task();

    /* Мост Wi-Fi: разбор команд из модуля и восстановление после помех.
       Выйдет из строя модуль — здесь просто ничего не произойдёт,
       на управление двигателем не влияет. */
    mc_wifi_task();

    /* Запись настроек во Flash — ТОЛЬКО здесь. Раньше команда 'w' вызывала
       стирание страницы прямо из прерывания USB: 20..40 мс с остановом
       конвейера, и всё это время ШИМ держал последнее заполнение. */
    if (mc_cfg_save_req)
    {
      mc_cfg_save_req = 0;
      mc_cfg_save();
    }

    /* Индикация: код неисправности читается по частоте вспышек */
    {
      uint32_t per = mc_led_pattern();
      uint32_t on;
      if (per == 0u)         on = 0u;
      else if (per == 1000u) on = ((HAL_GetTick() % 2000u) < 50u);  /* «жив» */
      else                   on = ((HAL_GetTick() / per) & 1u);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }

    /* Обновление сторожевого таймера через регистры (без HAL-драйвера) */
    IWDG->KR = 0xAAAA;

    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function — ШИМ 30 кГц + аппаратная защита BKIN
  *        Пары: PA8=TIM1_CH1 (выход на ключ), PA6=TIM1_BKIN (вход компаратора,
  *        частичный ремап TIM1)
  */
static void MX_TIM1_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreak = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2400;              /* 30 кГц при 72 МГц */
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;  /* при break ключ ЗАКРЫТ */
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* Аппаратная защита: компаратор на PA6 (TIM1_BKIN, частичный ремап). Активный
     НОЛЬ — при срабатывании компаратора PA6 уходит в 0 и ШИМ гаснет.
     Примечание: обрыв провода к компаратору ПОДТЯЖКА не увидит (PULLUP даёт
     «всё хорошо»), потеря провода не детектируется — это не баг, а свойство шата.
     AutomaticOutput DISABLE — защёлка: после срыва выход сам не вернётся,
     пока mc_trip_reset() явно не восстановит MOE.
     OSSI=ENABLE + OCIdleState=RESET — при срыве PA8 удерживается в нуле,
     а не уходит в Hi-Z (иначе драйвер может держать ключ приоткрытым). */
  sBreak.OffStateRunMode  = TIM_OSSR_DISABLE;
  sBreak.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreak.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreak.DeadTime         = 0;
  sBreak.BreakState       = TIM_BREAK_ENABLE;
  sBreak.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
  sBreak.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreak) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim1);
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* PC13 — встроенный светодиод Blue Pill (0 = горит) */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); /* погашен */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief IWDG init — сторожевой таймер, таймаут ~1.6 с
  *        IWDG настраивается через регистры (HAL-драйвера нет в сборке проекта).
  */
static void MX_IWDG_Init(void)
{
  /* LSI 40 кГц / 64 = 625 Гц → reload 1000 → ~1.6 с */
  IWDG->KR = 0x5555;       /* разрешить запись в IWDG_PR и IWDG_RLR */
  IWDG->PR = 4;            /* предделитель 64 (4 = 10.4 кГц... точнее: PR=4 → /64) */
  IWDG->RLR = 1000;        /* счёт до сброса */
  IWDG->KR = 0xCCCC;       /* запуск IWDG */
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* СНАЧАЛА снимаем ШИМ, потом встаём. Для TIM1 обязательно снять MOE,
     иначе выход останется активным даже после обнуления CCR. */
  TIM1->CCR1 = 0;
  TIM1->BDTR &= ~TIM_BDTR_MOE;   /* гасим выход ключа аппаратно */
  TIM1->CR1  &= ~TIM_CR1_CEN;
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */