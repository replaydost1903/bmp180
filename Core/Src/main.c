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
#include "bmp180.h"
#include "math.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int a=0,b=1;
long p;
long ilk_altitude_degeri;
long real_altitude,altitude;
float Temperature;
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
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  bmp180_read_calibration_data(&hi2c2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_StatusTypeDef value;
	  value = HAL_I2C_IsDeviceReady(&hi2c2, 0xEE, 10, 1000);
	  (void)value;

	  /*Read Calibration Data  (22 tane 8 bitlik data)*/


//	  /*Read uncompensated temperature value*/
//	  uint8_t data=0x2E;
//	  HAL_I2C_Mem_Write(&hi2c2, 0xEE, 0xF4, 1, &data, 1, 100);
//	  HAL_Delay(5);
//
//	  uint8_t uncompensated_temperature_value[2];
//	  HAL_I2C_Mem_Read(&hi2c2, 0xEF, 0xF6, 1, uncompensated_temperature_value,2, 100);
//	  HAL_Delay(5);
//	  long UT = ((uncompensated_temperature_value[0]<<8)+uncompensated_temperature_value[1]);
//
//
//	  /*ultra high resolution oss=3*/
//	  /*Read uncompensated pressure value*/
//	  int oss=3;
//	  uint8_t data_new=0x34+(oss<<6);
//	  uint8_t read_data[3];
//	  HAL_I2C_Mem_Write(&hi2c2, 0xEE, 0xF4, 1, &data_new, 1, 100);
//	  HAL_Delay(5);
//	  HAL_I2C_Mem_Read(&hi2c2, 0xEF, 0xF6, 1, read_data,3, 100);
//	  long UP = ((read_data[0]<<16)+(read_data[1]<<8)+read_data[2])>>(8-oss);
//
//
//	  /*calculate true temperature*/
//	  long X1=((UT-AC6) * (AC5/(pow(2,15))));
//	  long X2=MC*pow(2,11)/(X1+MD);
//	  long B5 = X1+X2;
//	  long T=(B5+8)/pow(2,4);
//	  Temperature = (float)T/10.0;
//
//
//	  /*calculate pressure temperature*/
//	  long B6 = B5-4000;
//	  X1 = (B2 * (B6*B6/(pow(2,12))))/(pow(2,11));
//	  X2 = AC2*B6/(pow(2,11));
//	  long X3 = X1+X2;
//	  long B3 = (((AC1*4+X3)<<oss)+2)/4;
//	  X1 = AC3*B6/pow(2,13);
//	  X2 = (B1 * (B6*B6/(pow(2,12))))/(pow(2,16));
//	  X3 = ((X1+X2)+2)/pow(2,2);
//	  unsigned long B4 = AC4*(unsigned long)(X3+32768)/(pow(2,15));
//	  unsigned long B7 = ((unsigned long)UP-B3)*(50000>>oss);
//	  if(B7<0x80000000)
//	  {
//		  p = (B7*2)/B4;
//	  }
//	  else
//	  {
//		  p = (B7/B4)*2;
//	  }
//	  X1 = (p/(pow(2,8)))*(p/(pow(2,8)));
//	  X1 = (X1*3038)/(pow(2,16));
//	  X2 = (-7357*p)/(pow(2,16));
//	  p=p+(X1+X2+3791)/pow(2,4);
//	  altitude = 44330*(1-(pow(((float)p/(float)101325), 0.19029495718)));
//
//
//	  if(b)
//	  {
//		  ilk_altitude_degeri = altitude;
//		  b=0;
//
//	  }
//	  else if(~b)
//	  {
//		  real_altitude = altitude - ilk_altitude_degeri;
//		  if(real_altitude<0)
//		  {
//			  real_altitude=0;
//		  }
//	  }
	  HAL_Delay(150);
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c2.Init.ClockSpeed = 400000;
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
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
