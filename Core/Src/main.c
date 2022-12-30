/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include<stdbool.h>
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
I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t reg_offset = 0;
volatile uint8_t reg_data[8] = {
    0x00, // Input port 0
    0x00, // Input port 1
    0b00000000, // Output port 0, ON F2, F4, F6, F8 by default
    0x00, // Output port 1
    0x00, // Polarity Inversion port 0
    0x00, // Polarity Inversion port 1
    0x00, // Configuration port 0
    0x00  // Configuration port 1
};
uint8_t i2c_state = 0;
uint32_t last_i2c_req_time = 0;

HAL_StatusTypeDef I2C_Slave_isr(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources) {
  if (hi2c->Instance->ISR & (I2C_ISR_ADDR)) {
    /* Clear ADDR flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);

    last_i2c_req_time = HAL_GetTick();
  }

  if (hi2c->Instance->ISR & (I2C_ISR_RXNE)) {
    uint8_t data = hi2c->Instance->RXDR;
    if (i2c_state == 0) {
      reg_offset = data;
      i2c_state = 1;
    } else {
      reg_data[reg_offset++] = hi2c->Instance->RXDR;
    }
  }

  if (hi2c->Instance->ISR & (I2C_ISR_TXIS)) {
    hi2c->Instance->TXDR = reg_data[reg_offset++];
  }

  if (reg_offset >= sizeof(reg_data)) {
    reg_offset = 0;
  }

  // ---------
  GPIOA->ODR = (GPIOA->ODR & 0xFF00) | reg_data[2];
  GPIOB->ODR = (GPIOB->ODR & 0xFF00) | reg_data[3];
  // ---------

  if (hi2c->Instance->ISR & (I2C_ISR_STOPF)) {
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

    /* Clear ADDR flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);

    // flush the transmit data register I2C_TXDR
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_TXE);

    i2c_state = 0;
  }

  return HAL_OK;
}

static void I2C_Slave_init() {
  // ====|| I2C Slave setup with interrupt ||====

  /* Enable Address Acknowledge */
  hi2c1.Instance->CR2 &= ~I2C_CR2_NACK;

  // Set isr function
  hi2c1.XferISR = I2C_Slave_isr;

  // Enable I2C interrup
  __HAL_I2C_ENABLE_IT(&hi2c1, I2C_IT_ADDRI | I2C_IT_STOPI | I2C_IT_TXI | I2C_IT_RXI);

  // ====|| END ||====

  i2c_state = 0;
}

bool i2c_error_detect = false;

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  i2c_error_detect = true;
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
  MX_I2C1_Init();
  MX_IWDG_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  I2C_Slave_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*
    if ((HAL_GetTick() > last_i2c_req_time) && ((HAL_GetTick() - last_i2c_req_time) > (30 * 1000))) { // Host not send over 30 s
      // so host down ?
      reg_data[2] = 0b00000000; // Output port 0, ON F2, F4, F6, F8 by default
      reg_data[3] = 0b00000000; // Output port 1, OFF All
    }*/
    GPIOA->ODR = (GPIOA->ODR & 0xFF00) | reg_data[2];
    GPIOB->ODR = (GPIOB->ODR & 0xFF00) | reg_data[3];
    if (i2c_error_detect) {
      i2c_error_detect = false;

      HAL_I2C_DeInit(&hi2c1);

      // Init again
      MX_I2C1_Init();
      I2C_Slave_init();
    }
    HAL_IWDG_Refresh(&hiwdg);
    HAL_Delay(50);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* I2C1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_IRQn);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 66;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_8;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 399;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, F1_Pin|F2_Pin|F3_Pin|F4_Pin
                          |F5_Pin|F6_Pin|F7_Pin|F8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PUMP_Pin|HEATER_Pin|LIGHT_Pin|GPIO_PIN_3
                          |GPIO_PIN_4|NET_Pin|RS485_SEL1_Pin|RS485_SEL0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : F1_Pin F2_Pin F3_Pin F4_Pin
                           F5_Pin F6_Pin F7_Pin F8_Pin */
  GPIO_InitStruct.Pin = F1_Pin|F2_Pin|F3_Pin|F4_Pin
                          |F5_Pin|F6_Pin|F7_Pin|F8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PUMP_Pin HEATER_Pin LIGHT_Pin PB3
                           PB4 NET_Pin RS485_SEL1_Pin RS485_SEL0_Pin */
  GPIO_InitStruct.Pin = PUMP_Pin|HEATER_Pin|LIGHT_Pin|GPIO_PIN_3
                          |GPIO_PIN_4|NET_Pin|RS485_SEL1_Pin|RS485_SEL0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
