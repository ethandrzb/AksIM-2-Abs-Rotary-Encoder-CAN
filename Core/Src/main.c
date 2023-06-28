/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
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
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// CAN
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxDataCAN[8];
uint8_t RxDataCAN[8];

uint32_t TxMailbox;

// UART buffer
uint8_t buffer[50];

// Encoder
#define ENCODER_POS_LENGTH 18
#define RX_DATA_LENGTH 7
uint8_t RxData[RX_DATA_LENGTH];
uint8_t printBits = 0;
uint32_t encPosition = 0;
uint8_t encStatus = 0;
uint8_t encCRC = 0;
uint32_t CRCMessage;

uint8_t CRCErrorCount = 255;

uint8_t CRCTable[64] = { 0x00, 0x03, 0x06, 0x05, 0x0C, 0x0F, 0x0A, 0x09, 
                            0x18, 0x1B, 0x1E, 0x1D, 0x14, 0x17, 0x12, 0x11, 
                            0x30, 0x33, 0x36, 0x35, 0x3C, 0x3F, 0x3A, 0x39, 
                            0x28, 0x2B, 0x2E, 0x2D, 0x24, 0x27, 0x22, 0x21, 
                            0x23, 0x20, 0x25, 0x26, 0x2F, 0x2C, 0x29, 0x2A, 
                            0x3B, 0x38, 0x3D, 0x3E, 0x37, 0x34, 0x31, 0x32, 
                            0x13, 0x10, 0x15, 0x16, 0x1F, 0x1C, 0x19, 0x1A, 
                            0x0B, 0x08, 0x0D, 0x0E, 0x07, 0x04, 0x01, 0x02
                            };

uint8_t calculateCRC(uint32_t data)
{
    uint8_t CRCValue = 0;
    uint32_t top = 0;

    // Unrolled version from RLS application note
    top = (data >> 30) & 0x00000003;
    CRCValue = (data >> 24) & 0x0000003F;
    top = CRCValue ^ CRCTable[top];
    CRCValue = (data >> 18) & 0x0000003F;
    top = CRCValue ^ CRCTable[top];
    CRCValue = (data >> 12) & 0x0000003F;
    top = CRCValue ^ CRCTable[top];
    CRCValue = (data >> 6) & 0x0000003F;
    top = CRCValue ^ CRCTable[top];
    CRCValue = data & 0x0000003F;
    top = CRCValue ^ CRCTable[top];
    CRCValue = CRCTable[top];

    return CRCTable[top];
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  // Position data is stored in RxData indices 3-5, MSB first
  // The >> 6 (or - 6 for the <<'s) is to compensate for the result being aligned to bit 23 instead of bit 17
  // The + 1 accommodates for the data being shifted in the individual packets
  // This is an 18-bit encoder, so it should output values in [0, 2^18 - 1]
  encPosition = ((uint32_t) RxData[3]) << (16 + 1 - 6);
  encPosition += ((uint32_t) RxData[4]) << (8 + 1 - 6);
  // LSBs of position are stored in the upper three bits of RxData[5], so we mask out everything else
  encPosition += (RxData[5] & 0xE0) >> 5;

  encStatus = (RxData[5] & 0x18) >> 3;

  encCRC = (RxData[5] & 0x07) << 3;
  encCRC += (RxData[6] & 0xE0) >> 5;

  CRCMessage = (uint32_t)(encStatus) << 28;
  CRCMessage += encPosition;
  CRCMessage <<= 2;

  // Split encoder position into discrete bytes
  TxDataCAN[0] = (encPosition >> 16) & 0x0003;
  TxDataCAN[1] = (encPosition >> 8) & 0x00FF;
  TxDataCAN[2] = encPosition & 0x00FF;

  // CRC check passed?
  if(calculateCRC(CRCMessage) == encCRC)
  {
    TxDataCAN[0] |= 0x10;
  }
  else
  {
    TxDataCAN[0] &= ~(0x10);
    CRCErrorCount++;
  }

  TxDataCAN[3] = CRCErrorCount;

  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxDataCAN, &TxMailbox);

  sprintf(buffer, "%0X %d 0x%02X 0x%0X %s %d\n", encPosition, encStatus, encCRC, calculateCRC(CRCMessage), (calculateCRC(CRCMessage) == encCRC) ? "CRC MATCHED" : "CRC MISMATCH", CRCErrorCount);
  if(HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), 100) != HAL_OK)
  {
    Error_Handler();
  }
  if(printBits)
    {
      for(int i = 0; i < RX_DATA_LENGTH; i++)
      {
        // Print each bit of message received
        uint8_t mask = 0x80;
        for(int j = 0; j < 8; j++)
        {
          if(RxData[i] & mask)
          {
            if(HAL_UART_Transmit(&huart2, "1", 2, 100) != HAL_OK)
            {
              Error_Handler();
            }
          }
          else
          {
            if(HAL_UART_Transmit(&huart2, "0", 2, 100) != HAL_OK)
            {
              Error_Handler();
            }
          }

          // Space into groups of 4 bits
          if(j % 4 == 3)
          {
            if(HAL_UART_Transmit(&huart2, " ", 2, 100) != HAL_OK)
            {
              Error_Handler();
            }
          }

          mask = mask >> 1;
        }

        if(HAL_UART_Transmit(&huart2, " | ", 4, 100) != HAL_OK)
        {
          Error_Handler();
        }
      }

      if(HAL_UART_Transmit(&huart2, "\n", 2, 100) != HAL_OK)
      {
        Error_Handler();
      }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxDataCAN);
  if(RxHeader.DLC == 4)
  {
    uint32_t positionRx = 0;
    uint8_t crcPassed = 0;

    // Upper nibble contains CRC flag, so we mask it out
    positionRx = ((uint32_t) (RxDataCAN[0]) & 0x0F) << 16;
    positionRx += ((uint32_t) RxDataCAN[1]) << 8;
    positionRx += RxDataCAN[2];

    crcPassed = RxDataCAN[0] & 0x10;

    // sprintf(buffer, "Message from other MCU: %d %d %d\n", RxDataCAN[0], RxDataCAN[1], RxDataCAN[2]);
    sprintf(buffer, "%d degrees (%d) %s (%d total)\n", (positionRx * 360) / 262144, positionRx, crcPassed ? "Data OK" : "Data invalid!", RxDataCAN[3]);
    if(HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), 100) != HAL_OK)
    {
      Error_Handler();
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == &htim2)
  {
      HAL_SPI_Receive_IT(&hspi1, &RxData[0], RX_DATA_LENGTH);
  }
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  // HAL_TIM_Base_Start_IT(&htim2);

  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  // 2 bytes
  TxHeader.DLC = 4;
  TxHeader.IDE = CAN_ID_STD;  // Standard CAN ID
  TxHeader.RTR = CAN_RTR_DATA;  // Specify that the packet contains data
  TxHeader.StdId = 0x103; // ID of transmitter
  // TxHeader.StdId = 0x446; // ID of transmitter
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 18;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef canFilterConfig;

  canFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  canFilterConfig.FilterBank = 18;
  canFilterConfig.FilterFIFOAssignment = CAN_FilterFIFO0;
  canFilterConfig.FilterIdHigh = 0x446 << 5;
  // canFilterConfig.FilterIdHigh = 0x103 << 5;
  canFilterConfig.FilterIdLow = 0;
  canFilterConfig.FilterMaskIdHigh = 0x446 << 5;
  // canFilterConfig.FilterMaskIdHigh = 0x103 << 5;
  canFilterConfig.FilterMaskIdLow = 0x0000;
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilterConfig.SlaveStartFilterBank = 20;

  HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig);
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 200-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

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

