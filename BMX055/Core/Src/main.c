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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMX055.h"
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

/* USER CODE BEGIN PV */
uint8_t buffer[100] = "Hello World";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
BMX055_t data_BMX055;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t debug = 0;
	uint8_t size = 0;
	uint8_t readData;
	uint8_t writeData;
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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

//  SearchDevice(&hi2c1);

  uint16_t init_tries= 0;
  while(BMX055_Init(&hi2c1) != 0 ){
	  BMX055_Init(&hi2c1);
	  init_tries++;
//	  while(1);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_Delay(100);

	  //debug = BMX055_Init(&hi2c1);
//	  debug = HAL_I2C_Mem_Read(&hi2c1, BMX055_ACC_SLAVE_ADDRESS_DEFAULT<<1, BMX055_WHO_AM_I_REG, 1, &readData, 1, 500);
//	  size = sprintf((char *)buffer, "WHO IAM 0x%X Shall be 0x%X\n\r", (int)readData, (int)BMX055_ACC_DEVICE);
//	  CDC_Transmit_FS(buffer, size);
//	  debug = 0;
//
//	  debug = HAL_I2C_Mem_Read(&hi2c1, BMX055_GYRO_SLAVE_ADDRESS_DEFAULT<<1, BMX055_WHO_AM_I_REG, 1, &readData, 1, 500);
//	  size = sprintf((char *)buffer, "WHO IAM 0x%X Shall be 0x%X\n\r", (int)readData, (int)0x0F);
//	  CDC_Transmit_FS(buffer, size);
//	  debug = 0;
//
//	  /* Wakeup Magnetometer */
//	  writeData = 0x01;
//	  HAL_I2C_Mem_Write(&hi2c1, BMX055_MAG_SLAVE_ADDRESS_DEFAULT<<1, BMX055_MAG_POW_CTL_REG, 1, &writeData, 1, 500);
//	  debug = HAL_I2C_Mem_Read(&hi2c1, BMX055_MAG_SLAVE_ADDRESS_DEFAULT<<1, BMX055_WHO_AM_I_MAG_REG, 1, &readData, 1, 500);
//	  size = sprintf((char *)buffer, "WHO IAM 0x%X Shall be 0x%X\n\r", (int)readData, (int)0x32);
//	  CDC_Transmit_FS(buffer, size);
//	  debug= 0;

	  int16_t rawAcc[3];
	  int16_t rawGyro[3];
	  int16_t rawMag[4];
	  float temp;
//	  readAccelData(rawAcc, &hi2c1);
	  size = sprintf((char *)buffer, "Acc: %d %d %d ", (int)rawAcc[0], (int)rawAcc[1], (int)rawAcc[2]);
//	  CDC_Transmit_FS(buffer, size);
	  HAL_Delay(2);

	  /* Read all sensors acc,gyro,mag */
	  BMX055_readAllSensors(&hi2c1, &data_BMX055);
	  size = sprintf((char *)buffer, "/*%.2f,%.2f,%.2f,",data_BMX055.AccelX, data_BMX055.AccelY, data_BMX055.AccelZ);
	  CDC_Transmit_FS(buffer, size);
	  HAL_Delay(2);

	  /* Get temperature from sensor BMX055 */
	  readTemp_BMX055(&temp, &hi2c1);
	  size = sprintf((char *)buffer, "%.2f,",temp);
	  CDC_Transmit_FS(buffer, size);
	  HAL_Delay(2);

//	  readGyroData(rawGyro, &hi2c1);
	  size = sprintf((char *)buffer, "%d,%d,%d,",rawGyro[0], rawGyro[1], rawGyro[2]);
//	  CDC_Transmit_FS(buffer, size);
	  HAL_Delay(2);

	  size = sprintf((char *)buffer, "%.2f,%.2f,%.2f,",data_BMX055.GyroX, data_BMX055.GyroY, data_BMX055.GyroZ);
	  CDC_Transmit_FS(buffer, size);
	  HAL_Delay(2);

	  readMagData(rawMag, &hi2c1);
	  size = sprintf((char *)buffer, "%d,%d,%d,%d*/\n\r",rawMag[0], rawMag[1], rawMag[2],rawMag[3]);
	  CDC_Transmit_FS(buffer, size);
	  HAL_Delay(2);





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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
