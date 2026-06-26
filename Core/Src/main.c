#include "main.h"
#include <stdio.h>
#include <string.h>

/* USER CODE BEGIN PTD */
typedef enum {
    STATE_NORMAL,
    STATE_INTERMEDIATE_WARNING,
    STATE_WARNING_ACKNOWLEDGED,
    STATE_HIGH_FAULT
} SystemState_t;
/* USER CODE END PTD */


/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
SystemState_t currentState = STATE_NORMAL;

/* Timing Trackers */
uint32_t ackTimerStart = 0;
uint32_t lastToggleTime = 0;
uint32_t lastPrintTime = 0;

/* Output tracking variables */
uint8_t toggleState = 0;
uint8_t ackButtonPressed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
uint32_t ADC_Read(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    // Increased timeout for safer multisample reads
    HAL_ADC_PollForConversion(&hadc1, 100);

    uint32_t value = HAL_ADC_GetValue(&hadc1);

    HAL_ADC_Stop(&hadc1);

    return value;
}

void Buzzer_Start(uint16_t freq)
{
    uint32_t arr = (72000000 / freq) - 1;

    __HAL_TIM_SET_AUTORELOAD(&htim4, arr);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, arr/2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

void Buzzer_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    // Explicit low fallback for passive piezo drivers on PB8
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
}

void Motor_Monitor(void)
{
    char buf[600];
    uint32_t currentTime = HAL_GetTick();

    /* 1. Sensor Acquisition & Mapping */
    uint8_t temp = ADC_Read(ADC_CHANNEL_0) * 100 / 4095;
    uint8_t vib  = ADC_Read(ADC_CHANNEL_1) * 100 / 4095;
    uint8_t curr = ADC_Read(ADC_CHANNEL_2) * 100 / 4095;
    uint8_t health = 100 - ((temp + vib + curr) / 3);

    /* 2. Flag Assertions */
    uint8_t temp_HF = (temp >= 84);
    uint8_t vib_HF  = (vib >= 84);
    uint8_t curr_HF = (curr >= 84);
    uint8_t highFaultCount = temp_HF + vib_HF + curr_HF;

    uint8_t temp_IW = (temp >= 70 && temp <= 83);
    uint8_t vib_IW  = (vib >= 70 && vib <= 83);
    uint8_t curr_IW = (curr >= 70 && curr <= 83);

    /*uint8_t anyHighFault = (temp_HF || vib_HF || curr_HF);*/
    uint8_t anyIntermediateWarning = (temp_IW || vib_IW || curr_IW);

    /* 3. Button Evaluation (PB0 - Active Low Configuration with Pull-up) */
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET) {
        if (!ackButtonPressed) {
            ackButtonPressed = 1; // Debounce latch flag
            if (currentState == STATE_INTERMEDIATE_WARNING) {
                currentState = STATE_WARNING_ACKNOWLEDGED;
                ackTimerStart = currentTime;

                // Instantly silence indicators on acknowledgment
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
                Buzzer_Stop();
            }
        }
    } else {
        ackButtonPressed = 0;
    }


    if(highFaultCount >= 2)
    {
        currentState = STATE_HIGH_FAULT;
    }
    else if(highFaultCount == 1)
    {
        currentState = STATE_HIGH_FAULT;
    }
    else if(anyIntermediateWarning)
    {
        if(currentState == STATE_WARNING_ACKNOWLEDGED)
        {
        	if(currentTime >= (ackTimerStart + 30000))
        	{
        	    currentState = STATE_INTERMEDIATE_WARNING;
        	}
        }
        else
        {
            currentState = STATE_INTERMEDIATE_WARNING;
        }
    }
    else
    {
        currentState = STATE_NORMAL;
    }

    /* 5. State Execution Logic */
    switch (currentState)
    {
        case STATE_NORMAL:
        	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
        	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);

        	Buzzer_Stop();

            break;

        case STATE_INTERMEDIATE_WARNING:
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET); // Blue OFF
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // Red OFF

            // Synchronized Non-blocking Blinking Pattern (500ms Duty Interval)
            if (currentTime - lastToggleTime >= 500) {
                lastToggleTime = currentTime;
                toggleState = !toggleState;

                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, toggleState ? GPIO_PIN_SET : GPIO_PIN_RESET);
                if (toggleState) {
                    Buzzer_Start(1200); // Distinct Intermediate warning beep pitch
                } else {
                    Buzzer_Stop();
                }
            }

            // Print intermediate warning block once every 4 seconds to limit UART spam
            if (currentTime - lastPrintTime >= 4000) {
                lastPrintTime = currentTime;
                char tempMsg[64] = "";
                char vibMsg[64] = "";
                char currMsg[64] = "";

                if (temp_IW) strcpy(tempMsg, "- TEMP: INTERMEDIATE OVERHEATING (CHECK FAN)\r\n");
                if (vib_IW)  strcpy(vibMsg,  "- VIB: INTERMEDIATE VIBRATION (INSPECT BEARING)\r\n");
                if (curr_IW) strcpy(currMsg, "- CURR: INTERMEDIATE OVERLOAD (REDUCE LOAD)\r\n");

                sprintf(buf,
                    "\r\n============================\r\n"
                    "INTERMEDIATE WARNING ACTIVE\r\n"
                    "============================\r\n"
                    "TEMP   : %d %%\r\n"
                    "VIB    : %d %%\r\n"
                    "CURR   : %d %%\r\n"
                    "HEALTH : %d %%\r\n\r\n"
                    "WARNING DETAILS:\r\n%s%s%s\r\n"
                    "MAINTENANCE REQUIRED\r\n"
                    "============================\r\n",
                    temp, vib, curr, health, tempMsg, vibMsg, currMsg);
                HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 200);
            }
            break;

        case STATE_WARNING_ACKNOWLEDGED:
            // Warnings remain muted during acknowledgment period
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
            Buzzer_Stop();
            break;

        case STATE_HIGH_FAULT:
        	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
        	if(highFaultCount >= 2)
        	{
        	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
        	    Buzzer_Start(4000);
        	}
        	else
        	{
        	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
        	    Buzzer_Start(1800);
        	}

            // Red LED Blinking (Fast 200ms Interval)
            if (currentTime - lastToggleTime >= 200) {
                lastToggleTime = currentTime;
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            }

            // Print High Fault updates every 2 seconds
            if (currentTime - lastPrintTime >= 2000) {
                lastPrintTime = currentTime;
                char fMsg[128] = "";
                if (temp_HF) strcat(fMsg, "- CRITICAL TEMPERATURE FAULT!\r\n");
                if (vib_HF)  strcat(fMsg, "- CRITICAL VIBRATION EXCESS!\r\n");
                if (curr_HF) strcat(fMsg, "- CRITICAL MOTOR OVERLOAD!\r\n");

                sprintf(buf,
                    "\r\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n"
                    "       CRITICAL HIGH FAULT\r\n"
                    "!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n"
                    "TEMP   : %d %%\r\n"
                    "VIB    : %d %%\r\n"
                    "CURR   : %d %%\r\n"
                    "HEALTH : %d %%\r\n\r\n"
                    "EMERGENCY FAULT LOG:\r\n%s\r\n"
                    "ACTION REQUIRED IMMEDIATELY!\r\n"
                    "!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n",
                    temp, vib, curr, health, fMsg);
                HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 200);
            }
            break;
    }
}

int main(void)
{


  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();



  /* Configure the system clock */
  SystemClock_Config();



  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
  /* USER CODE END 2 */



	  while (1)
	  {
	      Motor_Monitor();
	      HAL_Delay(50);
	  }


}


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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }


}


static void MX_TIM4_Init(void)
{



  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};


  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim4);

}


static void MX_USART1_UART_Init(void)
{


  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}


static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Pin|LED_2_Pin|led_3_green_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin LED_2_Pin led_3_green_Pin */
  GPIO_InitStruct.Pin = LED_Pin|LED_2_Pin|led_3_green_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RESEY_BUTTON_Pin */
  GPIO_InitStruct.Pin = RESEY_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RESEY_BUTTON_GPIO_Port, &GPIO_InitStruct);


}


void Error_Handler(void)
{

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
