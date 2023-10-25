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





#define REG_BATT 0xFF
const uint8_t OV7670_reg[][2] = {
	 {0x12, 0x14},   // QVGA, RGB
	  {0x8C, 0x00},   // RGB444 Disable
	  {0x40, 0x10 + 0xc0},   // RGB565, 00 - FF
	  {0x3A, 0x04 + 8},   // UYVY (why?)
	  {0x3D, 0x80 + 0x00},   // gamma enable, UV auto adjust, UYVY
	  {0xB0, 0x84}, // important

	  /* clock related */
	  {0x0C, 0x04},  // DCW enable
	  {0x3E, 0x19},  // manual scaling, pclk/=2
	  {0x70, 0x4A},  // scaling_xsc
	  {0x71, 0x35},  // scaling_ysc
	  {0x72, 0x11}, // down sample by 2
	  {0x73, 0xf1}, // DSP clock /= 2

	  /* windowing (empirically decided...) */
	  {0x17, 0x16},   // HSTART
	  {0x18, 0x04},   // HSTOP
	  {0x32, 0x80},   // HREF
	  {0x19, 0x03},   // VSTART =  14 ( = 3 * 4 + 2)
	  {0x1a, 0x7b},   // VSTOP  = 494 ( = 123 * 4 + 2)
	  {0x03, 0x0a},   // VREF (VSTART_LOW = 2, VSTOP_LOW = 2)

	  /* color matrix coefficient */
	#if 0
	  {0x4f, 0xb3},
	  {0x50, 0xb3},
	  {0x51, 0x00},
	  {0x52, 0x3d},
	  {0x53, 0xa7},
	  {0x54, 0xe4},
	  {0x58, 0x9e},
	#else
	  {0x4f, 0x80},
	  {0x50, 0x80},
	  {0x51, 0x00},
	  {0x52, 0x22},
	  {0x53, 0x5e},
	  {0x54, 0x80},
	  {0x58, 0x9e},
	#endif

	  /* 3a */
	//  {0x13, 0x84},
	//  {0x14, 0x0a},   // AGC Ceiling = 2x
	//  {0x5F, 0x2f},   // AWB B Gain Range (empirically decided)
	//                  // without this bright scene becomes yellow (purple). might be because of color matrix
	//  {0x60, 0x98},   // AWB R Gain Range (empirically decided)
	//  {0x61, 0x70},   // AWB G Gain Range (empirically decided)
	  {0x41, 0x38},   // edge enhancement, de-noise, AWG gain enabled


	  /* gamma curve */
	#if 1
	  {0x7b, 16},
	  {0x7c, 30},
	  {0x7d, 53},
	  {0x7e, 90},
	  {0x7f, 105},
	  {0x80, 118},
	  {0x81, 130},
	  {0x82, 140},
	  {0x83, 150},
	  {0x84, 160},
	  {0x85, 180},
	  {0x86, 195},
	  {0x87, 215},
	  {0x88, 230},
	  {0x89, 244},
	  {0x7a, 16},
	#else
	  /* gamma = 1 */
	  {0x7b, 4},
	  {0x7c, 8},
	  {0x7d, 16},
	  {0x7e, 32},
	  {0x7f, 40},
	  {0x80, 48},
	  {0x81, 56},
	  {0x82, 64},
	  {0x83, 72},
	  {0x84, 80},
	  {0x85, 96},
	  {0x86, 112},
	  {0x87, 144},
	  {0x88, 176},
	  {0x89, 208},
	  {0x7a, 64},
	#endif

	  /* fps */
	//  {0x6B, 0x4a}, //PLL  x4
	  {0x11, 0x00}, // pre-scalar = 1/1

	  /* others */
	  {0x1E, 0x31}, //mirror flip
	//  {0x42, 0x08}, // color bar

	{REG_BATT, REG_BATT},
};


static const uint8_t DevAdrr_T = 0x42;
static const uint8_t DevAdrr_R = 0x43;

#define OV7670_QVGA_WIDTH  320   // 65000
#define OV7670_QVGA_HEIGHT 240
#define CAMERA_FRAME_BUFFER_SIZE 38400  // (320 * 240 / 2 = 38400)
#define NEXT_ADDRESS 9600  // 38400/4


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
DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DCMI_Init(void);
static void MX_I2C2_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */


// My functions are here
int ov7670_read(uint8_t regAddr, uint8_t *data)
{
    if(HAL_I2C_Master_Transmit(&hi2c2, DevAdrr_T, &regAddr, 1, 100) == HAL_OK){
    	if(HAL_I2C_Master_Receive(&hi2c2, DevAdrr_R, data, 1, 100) == HAL_OK)
    		return 1;
    	else {
    		HAL_UART_Transmit(&huart4,(uint8_t*) "ov7670_read - You can transmit, but you cannot receive \r\n", sizeof("ov7670_read - You can transmit, but you cannot receive \r\n"), 20);
    		return 0;
    	}
    } else {
    	HAL_UART_Transmit(&huart4,(uint8_t*) "ov7670_read - You cannot even transmit. \r\n", sizeof("ov7670_read - You cannot even transmit. \r\n"), 20);
    	return 0;
    }

}

int ov7670_write(uint8_t regAddr, uint8_t data)
{
  if(HAL_I2C_Mem_Write(&hi2c2, DevAdrr_T, regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) == HAL_OK)
	  return 1;
  else{
	  HAL_UART_Transmit(&huart4,(uint8_t*) "ov7670_write has FAILED \r\n", sizeof("ov7670_write has FAILED \r\n"), 20);
	  return 0;
  };
}


int ov7670_init()
{
  if(ov7670_write(0x12, 0x80)){   // Resets the camera
	  HAL_Delay(50);
	  HAL_UART_Transmit(&huart4,(uint8_t*) "ov7670_init - RESET Operation of the Camera is completed. \r\n", sizeof("ov7670_init - RESET Operation of the Camera is completed. \r\n"), 20);
	  return 1;
  } else{
	  HAL_UART_Transmit(&huart4, (uint8_t*) "ov7670_init - RESET operation is FAILED. \r\n", sizeof("ov7670_init - RESET operation is FAILED. \r\n"), 20);
	  return 0;
  }
}




int ov7670_config()
{
  char aTxBuffer[1] = {0xAA};
  if(HAL_DCMI_Stop(&hdcmi) != HAL_OK){     // ?????????????????
	  HAL_UART_Transmit(&huart4,(uint8_t*) "ov7670_config - HAL_DCMI_Stop FAILED. \r\n", sizeof("ov7670_config - HAL_DCMI_Stop FAILED. \r\n"), 20);
	  return 0;
  } else if(ov7670_write(0x12, 0x80) != 1)  {  // RESET
	  HAL_UART_Transmit(&huart4,(uint8_t*) "ov7670_config - RESET FAILED. \r\n", sizeof("ov7670_config - RESET FAILED. \r\n"), 20);
	  return 0;
  } else{
	  ;
  }

  for(int i = 0; OV7670_reg[i][0] != REG_BATT; i++) {
	HAL_Delay(10);
    if(ov7670_write(OV7670_reg[i][0], OV7670_reg[i][1]) != 1){
    	HAL_Delay(1);
    	HAL_UART_Transmit(&huart4,(uint8_t*) "ov7670_config has FAILED when i = ", sizeof("ov7670_config has FAILED when i = "), 20);
    	HAL_UART_Transmit(&huart4, (uint8_t*)aTxBuffer, sprintf(aTxBuffer, "%d", i), 20);
    	HAL_UART_Transmit(&huart4, (uint8_t*) "\r\n", 2, 20);
    	return 0;
    }
  }
  return 1;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_DCMI_Init();
  MX_I2C2_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  // Arrange the RESET and PWDN pins
  HAL_GPIO_WritePin(DCMI_RST_GPIO_Port, DCMI_RST_Pin, SET);
  HAL_GPIO_WritePin(DCMI_PWD_GPIO_Port, DCMI_PWD_Pin, RESET);
  HAL_Delay(10);
  // Read the address of the I2C Channel
  if(HAL_I2C_IsDeviceReady(&hi2c2, DevAdrr_T, 3, 100) != HAL_OK){
	  HAL_UART_Transmit(&huart4,(uint8_t*) "FAIL - Transmit address is NOT ready \r\n", sizeof("FAIL - Transmit address is not READY \r\n"), 20);
	  return 0;
  }

  if(HAL_I2C_IsDeviceReady(&hi2c2, DevAdrr_R, 3, 100) != HAL_OK){
	  HAL_UART_Transmit(&huart4,(uint8_t*) "FAIL - Receive address is NOT ready \r\n", sizeof("FAIL - Receive address is not READY \r\n"), 20);
	  return 0;
  }


	// Initialization
	ov7670_init();

	// Camera Configuration
	if(ov7670_config())
		HAL_UART_Transmit(&huart4, (uint8_t*) "Configuration is completed with success \r\n", sizeof("Configuration is completed with success \r\n"), 20);

	HAL_Delay(500);

	uint32_t cameraBuffer[CAMERA_FRAME_BUFFER_SIZE] = {0};
  	uint16_t start_bit = 0xBEBE;
  	uint16_t stop_bit = 0xDEDE;

  while (1)
  {

	   memset(cameraBuffer, 0xCE, CAMERA_FRAME_BUFFER_SIZE*4);

	   if(HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, cameraBuffer, CAMERA_FRAME_BUFFER_SIZE) != HAL_OK){
		   HAL_UART_Transmit(&huart4,(uint8_t *) "DCMI_Start_Failed \r\n", sizeof("DCMI_Start_Failed \r\n"), 20);
			return 0;
		}

		HAL_Delay(500);

		HAL_DCMI_Suspend(&hdcmi);
		HAL_Delay(10);
		HAL_DCMI_Stop(&hdcmi);

		HAL_Delay(10);


		HAL_UART_Transmit(&huart4,(uint8_t *) &start_bit, 2, 20);
		HAL_Delay(1);
		HAL_UART_Transmit(&huart4, cameraBuffer, CAMERA_FRAME_BUFFER_SIZE, HAL_MAX_DELAY);
	    HAL_Delay(5);
	    HAL_UART_Transmit(&huart4, cameraBuffer+NEXT_ADDRESS, CAMERA_FRAME_BUFFER_SIZE, HAL_MAX_DELAY);
	    HAL_Delay(5);
	    HAL_UART_Transmit(&huart4, cameraBuffer+2*NEXT_ADDRESS, CAMERA_FRAME_BUFFER_SIZE, HAL_MAX_DELAY);
	    HAL_Delay(5);
	    HAL_UART_Transmit(&huart4, cameraBuffer+3*NEXT_ADDRESS, CAMERA_FRAME_BUFFER_SIZE, HAL_MAX_DELAY);
	    HAL_Delay(5);
	    HAL_UART_Transmit(&huart4,(uint8_t *) &stop_bit, 2, 20);
	    HAL_Delay(1);


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_5);
}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_FALLING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DCMI_RST_Pin|DCMI_PWD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DCMI_RST_Pin DCMI_PWD_Pin */
  GPIO_InitStruct.Pin = DCMI_RST_Pin|DCMI_PWD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
