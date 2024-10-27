/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint8_t seconds_units;
    uint8_t seconds_tens;
    uint8_t minutes_units;
    uint8_t minutes_tens;
    uint8_t hours_units;
    uint8_t hours_tens;
} TimeStruct;

typedef struct {
    uint8_t seconds_units;
    uint8_t seconds_tens;
    uint8_t minutes_units;
    uint8_t minutes_tens;
    uint8_t hours_units;
    uint8_t hours_tens;
} AlarmTimeStruct;

typedef enum {
    MODE_CONFIG_CLOCK,
    MODE_CONFIG_ALARM,
    MODE_RUNNING,
} ConfigMode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Clock_Frequency 32768;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
TimeStruct currentTime = {0, 0, 0, 0, 0, 0}; // Inicializa currentTime
AlarmTimeStruct alarmTime = {0, 0, 0, 0, 0, 0};
ConfigMode currentMode = MODE_RUNNING;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void incrementSeconds_units();
void incrementSeconds_decs();
void incrementMinutes_units();
void incrementMinutes_decs();
void incrementHours_units();
void incrementHours_decs();
void incrementAlarmMinutes_units();
void incrementAlarmMinutes_decs();
void incrementAlarmHours_units();
void incrementAlarmHours_decs();
void saveAlarm();
void updateRelogio();
void tocarAlarme();
void readButtons();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void incrementSeconds_units() {
    currentTime.seconds_units = (currentTime.seconds_units + 1) % 10;
}

void incrementSeconds_tens() {
    currentTime.seconds_tens = (currentTime.seconds_tens + 1) % 6;
}

void incrementMinutes_units() {
    currentTime.minutes_units = (currentTime.minutes_units + 1) % 10;
}

void incrementMinutes_tens() {
    currentTime.minutes_tens = (currentTime.minutes_tens + 1) % 6;
}

void incrementHours_units() {
	if (currentTime.hours_tens == 2) {
		currentTime.hours_units = (currentTime.hours_units + 1) % 4;  // Limite de 24 horas
	} else {
		currentTime.hours_units = (currentTime.hours_units + 1) % 10;
	}
}

void incrementHours_tens() {
    currentTime.hours_tens = (currentTime.hours_tens + 1) % 3;  // Limite de 24 horas
}

void incrementAlarmMinutes_units() {
    alarmTime.minutes_units = (alarmTime.minutes_units + 1) % 10;
}

void incrementAlarmMinutes_tens() {
    alarmTime.minutes_tens = (alarmTime.minutes_tens + 1) % 6;
}

void incrementAlarmHours_units() {
    if (alarmTime.hours_tens == 2) {
        alarmTime.hours_units = (alarmTime.hours_units + 1) % 4;  // Limite de 24 horas
    } else {
        alarmTime.hours_units = (alarmTime.hours_units + 1) % 10;
    }
}

void incrementAlarmHours_tens() {
    alarmTime.hours_tens = (alarmTime.hours_tens + 1) % 3;  // Limite de 24 horas
}


void saveAlarm() {
    // Implementar a lógica para salvar o alarme
}

void updateRelogio(){
  // Implementar a lógica para atualizar o relógio com o RTC (RUNNING MODE)

  // Implementar a lógica para atualizar o relógio sem atualizar o RTC (CONFIG MODE)
}

void tocarAlarme() {
	if (currentTime.hours_tens == alarmTime.hours_tens && currentTime.hours_units == alarmTime.hours_units &&
	    currentTime.minutes_tens == alarmTime.minutes_tens && currentTime.minutes_units == alarmTime.minutes_units) {
	    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	    HAL_Delay(1000);  // Alarm sound duration
	    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
	}

}

void readButtons() {
    uint8_t Config_Set = 0;

    if (HAL_GPIO_ReadPin(GPIOA, SEL_MODE_Pin) == GPIO_PIN_SET) {
        Config_Set = (Config_Set + 1) % 2;
        currentMode = Config_Set ? MODE_CONFIG_ALARM : MODE_CONFIG_CLOCK;
    }

    if (currentMode == MODE_CONFIG_CLOCK) {
        uint8_t confirm = 0;

        while (!confirm) {
            if (HAL_GPIO_ReadPin(SEC_UNID_GPIO_Port, SEC_UNID_Pin) == GPIO_PIN_SET) { incrementSeconds_units(); }
            if (HAL_GPIO_ReadPin(SEC_DEZ_GPIO_Port, SEC_DEZ_Pin) == GPIO_PIN_SET) { incrementSeconds_tens(); }
            if (HAL_GPIO_ReadPin(MIN_UNID_GPIO_Port, MIN_UNID_Pin) == GPIO_PIN_SET) { incrementMinutes_units(); }
            if (HAL_GPIO_ReadPin(MIN_DEZ_GPIO_Port, MIN_DEZ_Pin) == GPIO_PIN_SET) { incrementMinutes_tens(); }
            if (HAL_GPIO_ReadPin(HOUR_UNID_GPIO_Port, HOUR_UNID_Pin) == GPIO_PIN_SET) { incrementHours_units(); }
            if (HAL_GPIO_ReadPin(HOUR_DEZ_GPIO_Port, HOUR_DEZ_Pin) == GPIO_PIN_SET) { incrementHours_tens(); }

            if (HAL_GPIO_ReadPin(GPIOA, CONFIRM_Pin) == GPIO_PIN_SET) {
                confirm = 1;
            }
        }
    } else if (currentMode == MODE_CONFIG_ALARM) {
        uint8_t confirm = 0;

        while (!confirm) {
            if (HAL_GPIO_ReadPin(MIN_UNID_GPIO_Port, MIN_UNID_Pin) == GPIO_PIN_SET) { incrementAlarmMinutes_units(); }
            if (HAL_GPIO_ReadPin(MIN_DEZ_GPIO_Port, MIN_DEZ_Pin) == GPIO_PIN_SET) { incrementAlarmMinutes_tens(); }
            if (HAL_GPIO_ReadPin(HOUR_UNID_GPIO_Port, HOUR_UNID_Pin) == GPIO_PIN_SET) { incrementAlarmHours_units(); }
            if (HAL_GPIO_ReadPin(HOUR_DEZ_GPIO_Port, HOUR_DEZ_Pin) == GPIO_PIN_SET) { incrementAlarmHours_tens(); }

            if (HAL_GPIO_ReadPin(CONFIRM_GPIO_Port, CONFIRM_Pin) == GPIO_PIN_SET) {
                confirm = 1;
                saveAlarm();
            }
        }
    }

    currentMode = MODE_RUNNING;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  readButtons();

	  if (currentMode == MODE_RUNNING) {
		  updateRelogio();
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 7;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 3;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DATA_Pin|SCLK_Pin|XLAT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLANK_GPIO_Port, BLANK_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : DATA_Pin SCLK_Pin XLAT_Pin BLANK_Pin */
  GPIO_InitStruct.Pin = DATA_Pin|SCLK_Pin|XLAT_Pin|BLANK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SEC_UNID_Pin SEC_DEZ_Pin MIN_UNID_Pin MIN_DEZ_Pin
                           HOUR_UNID_Pin HOUR_DEZ_Pin SEL_MODE_Pin CONFIRM_Pin */
  GPIO_InitStruct.Pin = SEC_UNID_Pin|SEC_DEZ_Pin|MIN_UNID_Pin|MIN_DEZ_Pin
                          |HOUR_UNID_Pin|HOUR_DEZ_Pin|SEL_MODE_Pin|CONFIRM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ON_OFF_Pin */
  GPIO_InitStruct.Pin = ON_OFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ON_OFF_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
