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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "bme680.h"
#include <stdio.h>
#include <string.h>

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

/* USER CODE BEGIN PV */

char message[50];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void user_delay_ms(uint32_t period) {
	HAL_Delay(period);
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
  int8_t rslt = 0;
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Read(&hi2c2, (uint16_t)(dev_id<<1), reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)reg_data, len, 5);

  if (status != HAL_OK)
    rslt = -1;

  return rslt;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
	int8_t rslt = 0;
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Write(&hi2c2, (uint16_t)(dev_id<<1), reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)reg_data, len, 5);

  if (status != HAL_OK)
    rslt = -1;

  return rslt;
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
  MX_I2C2_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(2000);

  for (uint8_t i = 0; i < 128; i++) {
	  if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(i<<1), 3, 5) == HAL_OK) {
      sprintf(message, "Sensor device on address 0x%2x is ready.\r\n", i);
      HAL_UART_Transmit(&hlpuart1, (uint8_t*)message, sizeof(message), 100);
      memset(message, 0, sizeof(message));
	  }
  }

  /* Initializing sensor */
  struct bme680_dev sensor;
  sensor.dev_id = BME680_I2C_ADDR_SECONDARY;
  sensor.intf = BME680_I2C_INTF;
  sensor.read = user_i2c_read;
  sensor.write = user_i2c_write;
  sensor.delay_ms = user_delay_ms;
  sensor.amb_temp = 25;

  int8_t rslt = BME680_OK;
  rslt = bme680_init(&sensor);

  /* Configuring sensor */
  sensor.tph_sett.os_hum = BME680_OS_2X;
  sensor.tph_sett.os_pres = BME680_OS_4X;
  sensor.tph_sett.os_temp = BME680_OS_8X;
  sensor.tph_sett.filter = BME680_FILTER_SIZE_3;
  sensor.gas_sett.run_gas = BME680_DISABLE_GAS_MEAS;
  sensor.gas_sett.heatr_temp = 0;
  sensor.gas_sett.heatr_dur = 0;
  sensor.power_mode = BME680_FORCED_MODE;
 
  uint8_t set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL; // | BME680_GAS_SENSOR_SEL;

  rslt = bme680_set_sensor_settings(set_required_settings, &sensor);
  rslt = bme680_set_sensor_mode(&sensor);

  uint16_t meas_period;
  bme680_get_profile_dur(&meas_period, &sensor);

  struct bme680_field_data data;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    rslt = bme680_get_sensor_data(&data, &sensor);

    //sprintf(message, "rslt: %d, T: %d C, P: %d hPa, H: %d rH\r\n", rslt, data.temperature / 100, data.pressure / 100, data.humidity / 1000);
    
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)message, sizeof(message), 100);
    HAL_Delay(5000);

    rslt = bme680_set_sensor_mode(&sensor);

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
