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
#include "gpio.h"
#include "i2c.h"
#include "stm32wlxx_hal.h"
#include "stm32wlxx_hal_def.h"
#include "stm32wlxx_hal_uart.h"
#include "subghz.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "bme680.h"
#include "radio_driver.h"
#include "stm32wlxx_nucleo.h"
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum { STATE_NULL, STATE_MASTER, STATE_SLAVE } state_t;

typedef enum { SSTATE_NULL, SSTATE_RX, SSTATE_TX } substate_t;

typedef struct {
  state_t state;
  substate_t subState;
  uint32_t rxTimeout;
  uint32_t rxMargin;
  uint32_t randomDelay;
  char rxBuffer[RX_BUFFER_SIZE];
  uint8_t rxSize;
} pingPongFSM_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define RF_FREQUENCY 868000000 /* Hz */
#define TX_OUTPUT_POWER 14     /* dBm */
#define LORA_BANDWIDTH 0       /* Hz */
#define LORA_SPREADING_FACTOR 10
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8 /* Same for Tx and Rx */
#define LORA_SYMBOL_TIMEOUT 5  /* Symbols */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

char message[50];
void (*volatile eventReceptor)(pingPongFSM_t *const fsm);
PacketParams_t packetParams; // TODO: this is lazy
const RadioLoRaBandwidths_t Bandwidths[] = {LORA_BW_125, LORA_BW_250,
                                            LORA_BW_500};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void radioInit(void);
void RadioOnDioIrq(RadioIrqMasks_t radioIrq);
void eventTxDone(pingPongFSM_t *const fsm);
void eventRxDone(pingPongFSM_t *const fsm);
void eventTxTimeout(pingPongFSM_t *const fsm);
void eventRxTimeout(pingPongFSM_t *const fsm);
void eventRxError(pingPongFSM_t *const fsm);
void enterMasterRx(pingPongFSM_t *const fsm);
void enterSlaveRx(pingPongFSM_t *const fsm);
void enterMasterTx(pingPongFSM_t *const fsm, char *message, size_t size);
void enterSlaveTx(pingPongFSM_t *const fsm, char *message, size_t size);
void transitionRxDone(pingPongFSM_t *const fsm);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef TRANSMITTER

void user_delay_ms(uint32_t period) { HAL_Delay(period); }

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data,
                     uint16_t len) {
  int8_t rslt = 0;
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Read(&hi2c2, (uint16_t)(dev_id << 1), reg_addr,
                            I2C_MEMADD_SIZE_8BIT, (uint8_t *)reg_data, len, 5);

  if (status != HAL_OK)
    rslt = -1;

  return rslt;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data,
                      uint16_t len) {
  int8_t rslt = 0;
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Write(&hi2c2, (uint16_t)(dev_id << 1), reg_addr,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t *)reg_data, len, 5);

  if (status != HAL_OK)
    rslt = -1;

  return rslt;
}

#endif

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

  pingPongFSM_t fsm;
  char uartBuff[100];

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  MX_SUBGHZ_Init();
  /* USER CODE BEGIN 2 */

  strcpy(uartBuff,
         "\n\r02226 Networked Embedded Systems\r\nDTU\r\n---------------\r\n");
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)uartBuff, strlen(uartBuff),
                    HAL_MAX_DELAY);
  sprintf(uartBuff, "LORA_MODULATION\r\nLORA_BW=%d Hz\r\nLORA_SF=%d\r\n",
          (1 << LORA_BANDWIDTH) * 125, LORA_SPREADING_FACTOR);
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)uartBuff, strlen(uartBuff),
                    HAL_MAX_DELAY);
  radioInit();

  uint32_t rnd = 0;
  SUBGRF_SetDioIrqParams(IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE,
                         IRQ_RADIO_NONE);
  rnd = SUBGRF_GetRandom();

  fsm.state = STATE_NULL;
  fsm.subState = SSTATE_NULL;
  fsm.rxTimeout = 3000;        // 3000 ms
  fsm.rxMargin = 200;          // 200 ms
  fsm.randomDelay = rnd >> 22; // [0, 1023] ms
  sprintf(uartBuff, "rand=%lu\r\n", fsm.randomDelay);
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)uartBuff, strlen(uartBuff),
                    HAL_MAX_DELAY);

  HAL_Delay(fsm.randomDelay);
  SUBGRF_SetDioIrqParams(IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
                         IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
                         IRQ_RADIO_NONE, IRQ_RADIO_NONE);
  SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
  SUBGRF_SetRx(fsm.rxTimeout << 6);
  fsm.state = STATE_MASTER;
  fsm.subState = SSTATE_RX;

  HAL_Delay(2000);
  BSP_LED_Off(LED_BLUE);
  BSP_LED_On(LED_RED);

#ifdef TRANSMITTER

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

  if (rslt != 0) {
    // Initialization failed
  }

  /* Configuring sensor */
  sensor.tph_sett.os_hum = BME680_OS_2X;
  sensor.tph_sett.os_pres = BME680_OS_4X;
  sensor.tph_sett.os_temp = BME680_OS_8X;
  sensor.tph_sett.filter = BME680_FILTER_SIZE_3;
  sensor.gas_sett.run_gas = BME680_DISABLE_GAS_MEAS;
  sensor.gas_sett.heatr_temp = 0;
  sensor.gas_sett.heatr_dur = 0;
  sensor.power_mode = BME680_FORCED_MODE;

  uint8_t set_required_settings = BME680_OST_SEL | BME680_OSP_SEL |
                                  BME680_OSH_SEL | BME680_FILTER_SEL |
                                  BME680_GAS_SENSOR_SEL;

  rslt = bme680_set_sensor_settings(set_required_settings, &sensor);
  rslt = bme680_set_sensor_mode(&sensor);

  uint16_t meas_period;
  bme680_get_profile_dur(&meas_period, &sensor);

  struct bme680_field_data data;

#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

    eventReceptor = NULL;
    while (eventReceptor == NULL)
      ;
    eventReceptor(&fsm);

#ifdef TRANSMITTER

    rslt = bme680_get_sensor_data(&data, &sensor);

    sprintf(message, ",%d,%d,%d", data.temperature, data.pressure,
            data.humidity);

    rslt = bme680_set_sensor_mode(&sensor);

#endif

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
   */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3 | RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief  Initialize the Sub-GHz radio and dependent hardware.
 * @retval None
 */
void radioInit(void) {
  // Initialize the hardware (SPI bus, TCXO control, RF switch)
  SUBGRF_Init(RadioOnDioIrq);

  // Use DCDC converter if `DCDC_ENABLE` is defined in radio_conf.h
  // "By default, the SMPS clock detection is disabled and must be enabled
  // before enabling the SMPS." (6.1 in RM0453)
  SUBGRF_WriteRegister(SUBGHZ_SMPSC0R, (SUBGRF_ReadRegister(SUBGHZ_SMPSC0R) |
                                        SMPS_CLK_DET_ENABLE));
  SUBGRF_SetRegulatorMode();

  // Use the whole 256-byte buffer for both TX and RX
  SUBGRF_SetBufferBaseAddress(0x00, 0x00);

  SUBGRF_SetRfFrequency(RF_FREQUENCY);
  SUBGRF_SetRfTxPower(TX_OUTPUT_POWER);
  SUBGRF_SetStopRxTimerOnPreambleDetect(false);

  SUBGRF_SetPacketType(PACKET_TYPE_LORA);

  SUBGRF_WriteRegister(REG_LR_SYNCWORD,
                       (LORA_MAC_PRIVATE_SYNCWORD >> 8) & 0xFF);
  SUBGRF_WriteRegister(REG_LR_SYNCWORD + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF);

  ModulationParams_t modulationParams;
  modulationParams.PacketType = PACKET_TYPE_LORA;
  modulationParams.Params.LoRa.Bandwidth = Bandwidths[LORA_BANDWIDTH];
  modulationParams.Params.LoRa.CodingRate =
      (RadioLoRaCodingRates_t)LORA_CODINGRATE;
  modulationParams.Params.LoRa.LowDatarateOptimize = 0x00;
  modulationParams.Params.LoRa.SpreadingFactor =
      (RadioLoRaSpreadingFactors_t)LORA_SPREADING_FACTOR;
  SUBGRF_SetModulationParams(&modulationParams);

  packetParams.PacketType = PACKET_TYPE_LORA;
  packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
  packetParams.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH;
  packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
  packetParams.Params.LoRa.PayloadLength = 0xFF;
  packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
  SUBGRF_SetPacketParams(&packetParams);

  // SUBGRF_SetLoRaSymbNumTimeout(LORA_SYMBOL_TIMEOUT);

  // WORKAROUND - Optimizing the Inverted IQ Operation, see DS_SX1261-2_V1.2
  // datasheet chapter 15.4 RegIqPolaritySetup @address 0x0736
  SUBGRF_WriteRegister(0x0736, SUBGRF_ReadRegister(0x0736) | (1 << 2));
}

/**
 * @brief  Receive data trough SUBGHZSPI peripheral
 * @param  radioIrq  interrupt pending status information
 * @retval None
 */
void RadioOnDioIrq(RadioIrqMasks_t radioIrq) {
  switch (radioIrq) {
  case IRQ_TX_DONE:
    eventReceptor = eventTxDone;
    break;
  case IRQ_RX_DONE:
    eventReceptor = eventRxDone;
    break;
  case IRQ_RX_TX_TIMEOUT:
    if (SUBGRF_GetOperatingMode() == MODE_TX) {
      eventReceptor = eventTxTimeout;
    } else if (SUBGRF_GetOperatingMode() == MODE_RX) {
      eventReceptor = eventRxTimeout;
    }
    break;
  case IRQ_CRC_ERROR:
    eventReceptor = eventRxError;
    break;
  default:
    break;
  }
}

/**
 * @brief  Process the TX Done event
 * @param  fsm pointer to FSM context
 * @retval None
 */
void eventTxDone(pingPongFSM_t *const fsm) {
  switch (fsm->state) {
  case STATE_MASTER:
    switch (fsm->subState) {
    case SSTATE_TX:
      enterMasterRx(fsm);
      fsm->subState = SSTATE_RX;
      break;
    default:
      break;
    }
    break;
  case STATE_SLAVE:
    switch (fsm->subState) {
    case SSTATE_TX:
      enterSlaveRx(fsm);
      fsm->subState = SSTATE_RX;
      break;
    default:
      break;
    }
    break;
  default:
    break;
  }
}

void uartLogEvent(pingPongFSM_t *const fsm) {
  PacketStatus_t packetStatus;
  SUBGRF_GetPacketStatus(&packetStatus);
  char uart_buffer[50];
  strncpy(uart_buffer, fsm->rxBuffer, fsm->rxSize);
  uart_buffer[fsm->rxSize] = '\0';
  sprintf(uart_buffer + fsm->rxSize, ",%d,%d\r\n",
          packetStatus.Params.LoRa.RssiPkt, packetStatus.Params.LoRa.SnrPkt);
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)uart_buffer, strlen(uart_buffer),
                    HAL_MAX_DELAY);
  BSP_LED_Off(LED_RED);
  BSP_LED_Toggle(LED_BLUE);
}

/**
 * @brief  Process the RX Done event
 * @param  fsm pointer to FSM context
 * @retval None
 */
void eventRxDone(pingPongFSM_t *const fsm) {
  switch (fsm->state) {
  case STATE_MASTER:
    switch (fsm->subState) {
    case SSTATE_RX:
      transitionRxDone(fsm);
      if (strncmp(fsm->rxBuffer, "iOuC", 4) == 0) {
        uartLogEvent(fsm);
        enterMasterTx(fsm, message, strlen(message));
        fsm->subState = SSTATE_TX;
      } else if (strncmp(fsm->rxBuffer, "nX05", 4) == 0) {
        uartLogEvent(fsm);
        enterSlaveRx(fsm);
        fsm->state = STATE_SLAVE;
      } else {
        enterMasterRx(fsm);
      }
      break;
    default:
      break;
    }
    break;
  case STATE_SLAVE:
    switch (fsm->subState) {
    case SSTATE_RX:
      transitionRxDone(fsm);
      if (strncmp(fsm->rxBuffer, "nX05", 4) == 0) {
        uartLogEvent(fsm);
        enterSlaveTx(fsm, message, strlen(message));
        fsm->subState = SSTATE_TX;
      } else {
        enterMasterRx(fsm);
        fsm->state = STATE_MASTER;
      }
      break;
    default:
      break;
    }
    break;
  default:
    break;
  }
}

/**
 * @brief  Process the TX Timeout event
 * @param  fsm pointer to FSM context
 * @retval None
 */
void eventTxTimeout(pingPongFSM_t *const fsm) {
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Event TX Timeout,,,,\r\n", 22,
                    HAL_MAX_DELAY);
  BSP_LED_Off(LED_BLUE);
  BSP_LED_On(LED_RED);
  switch (fsm->state) {
  case STATE_MASTER:
    switch (fsm->subState) {
    case SSTATE_TX:
      enterMasterRx(fsm);
      fsm->subState = SSTATE_RX;
      break;
    default:
      break;
    }
    break;
  case STATE_SLAVE:
    switch (fsm->subState) {
    case SSTATE_TX:
      enterSlaveRx(fsm);
      fsm->subState = SSTATE_RX;
      break;
    default:
      break;
    }
    break;
  default:
    break;
  }
}

/**
 * @brief  Process the RX Timeout event
 * @param  fsm pointer to FSM context
 * @retval None
 */
void eventRxTimeout(pingPongFSM_t *const fsm) {
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Event RX Timeout,,,,\r\n", 22,
                    HAL_MAX_DELAY);
  BSP_LED_Off(LED_BLUE);
  BSP_LED_On(LED_RED);
  switch (fsm->state) {
  case STATE_MASTER:
    switch (fsm->subState) {
    case SSTATE_RX:
      HAL_Delay(fsm->randomDelay);
      enterMasterTx(fsm, message, strlen(message));
      fsm->subState = SSTATE_TX;
      break;
    default:
      break;
    }
    break;
  case STATE_SLAVE:
    switch (fsm->subState) {
    case SSTATE_RX:
      enterSlaveRx(fsm);
      break;
    default:
      break;
    }
    break;
  default:
    break;
  }
}

/**
 * @brief  Process the RX Error event
 * @param  fsm pointer to FSM context
 * @retval None
 */
void eventRxError(pingPongFSM_t *const fsm) {
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Event RX Error,,,,\r\n", 20,
                    HAL_MAX_DELAY);
  BSP_LED_Off(LED_BLUE);
  BSP_LED_On(LED_RED);
  switch (fsm->state) {
  case STATE_MASTER:
    switch (fsm->subState) {
    case SSTATE_RX:
      HAL_Delay(fsm->randomDelay);
      enterMasterTx(fsm, message, strlen(message));
      fsm->subState = SSTATE_TX;
      break;
    default:
      break;
    }
    break;
  case STATE_SLAVE:
    switch (fsm->subState) {
    case SSTATE_RX:
      enterSlaveRx(fsm);
      break;
    default:
      break;
    }
    break;
  default:
    break;
  }
}

/**
 * @brief  Entry actions for the RX sub-state of the Master state
 * @param  fsm pointer to FSM context
 * @retval None
 */
void enterMasterRx(pingPongFSM_t *const fsm) {
  SUBGRF_SetDioIrqParams(
      IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
      IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
      IRQ_RADIO_NONE, IRQ_RADIO_NONE);
  SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
  packetParams.Params.LoRa.PayloadLength = 0xFF;
  SUBGRF_SetPacketParams(&packetParams);
  SUBGRF_SetRx(fsm->rxTimeout << 6);
}

/**
 * @brief  Entry actions for the RX sub-state of the Slave state
 * @param  fsm pointer to FSM context
 * @retval None
 */
void enterSlaveRx(pingPongFSM_t *const fsm) {
  SUBGRF_SetDioIrqParams(
      IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
      IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
      IRQ_RADIO_NONE, IRQ_RADIO_NONE);
  SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
  packetParams.Params.LoRa.PayloadLength = 0xFF;
  SUBGRF_SetPacketParams(&packetParams);
  SUBGRF_SetRx(fsm->rxTimeout << 6);
}

/**
 * @brief  Entry actions for the TX sub-state of the Master state
 * @param  fsm pointer to FSM context
 * @retval None
 */
void enterMasterTx(pingPongFSM_t *const fsm, char *message, size_t size) {
  HAL_Delay(fsm->rxMargin);
  char payload[50] = "nX05";
  strcat(payload, message);
  SUBGRF_SetDioIrqParams(IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                         IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT, IRQ_RADIO_NONE,
                         IRQ_RADIO_NONE);
  SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX);
  // Workaround 5.1 in DS.SX1261-2.W.APP (before each packet transmission)
  SUBGRF_WriteRegister(0x0889, (SUBGRF_ReadRegister(0x0889) | 0x04));
  packetParams.Params.LoRa.PayloadLength = strlen(payload);
  SUBGRF_SetPacketParams(&packetParams);
  SUBGRF_SendPayload((uint8_t *)payload, strlen(payload), 0);
}

/**
 * @brief  Entry actions for the TX sub-state of the Slave state
 * @param  fsm pointer to FSM context
 * @retval None
 */
void enterSlaveTx(pingPongFSM_t *const fsm, char *message, size_t size) {
  HAL_Delay(fsm->rxMargin);
  char payload[50] = "iOuC";
  strcat(payload, message);
  SUBGRF_SetDioIrqParams(IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                         IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT, IRQ_RADIO_NONE,
                         IRQ_RADIO_NONE);
  SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX);
  // Workaround 5.1 in DS.SX1261-2.W.APP (before each packet transmission)
  SUBGRF_WriteRegister(0x0889, (SUBGRF_ReadRegister(0x0889) | 0x04));
  packetParams.Params.LoRa.PayloadLength = strlen(payload);
  SUBGRF_SetPacketParams(&packetParams);
  SUBGRF_SendPayload((uint8_t *)payload, strlen(payload), 0);
}

/**
 * @brief  Transition actions executed on every RX Done event (helper function)
 * @param  fsm pointer to FSM context
 * @retval None
 */
void transitionRxDone(pingPongFSM_t *const fsm) {
  PacketStatus_t packetStatus;
  // Workaround 15.3 in DS.SX1261-2.W.APP (because following RX w/ timeout
  // sequence)
  SUBGRF_WriteRegister(0x0920, 0x00);
  SUBGRF_WriteRegister(0x0944, (SUBGRF_ReadRegister(0x0944) | 0x02));

  SUBGRF_GetPayload((uint8_t *)fsm->rxBuffer, &fsm->rxSize, 0xFF);
  SUBGRF_GetPacketStatus(&packetStatus);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
