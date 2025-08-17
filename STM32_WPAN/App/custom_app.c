/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpr121.h"
#include "apds9960.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* LiquidMonitoringService */
  uint8_t               Height_Notification_Status;
  uint8_t               Color_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */
  Height_MeasVal_t      MeasurementHeight;
  Color_MeasVal_t       MeasurementColor;
  Height_CalVal_t       CalibrationHight;
  uint8_t               TimerMeasurement_Id;
  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MEASUREMENT_INTERVAL (1000000/CFG_TS_TICK_VAL)  /**< 1s */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[512];
uint8_t NotifyCharData[512];
uint16_t Connection_Handle;
/* USER CODE BEGIN PV */
extern I2C_HandleTypeDef hi2c1;
APDS9960_t apds;
MPR121_t mpr;

uint16_t redValue;
uint16_t greenValue;
uint16_t blueValue;

Calibration_Data_t g_heightCalibration = {1.0f, 0.0f};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* LiquidMonitoringService */
static void Custom_Height_Update_Char(void);
static void Custom_Height_Send_Notification(void);
static void Custom_Color_Update_Char(void);
static void Custom_Color_Send_Notification(void);

/* USER CODE BEGIN PFP */
static void MeasurementTask(void);

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* LiquidMonitoringService */
    case CUSTOM_STM_HEIGHT_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_HEIGHT_READ_EVT */

      /* USER CODE END CUSTOM_STM_HEIGHT_READ_EVT */
      break;

    case CUSTOM_STM_HEIGHT_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_HEIGHT_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_HEIGHT_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_HEIGHT_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_HEIGHT_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_HEIGHT_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_COLOR_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_COLOR_READ_EVT */

      /* USER CODE END CUSTOM_STM_COLOR_READ_EVT */
      break;

    case CUSTOM_STM_COLOR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_COLOR_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_COLOR_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_COLOR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_COLOR_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_COLOR_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_HEIGHTCAL_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_HEIGHTCAL_WRITE_NO_RESP_EVT */

      /* USER CODE END CUSTOM_STM_HEIGHTCAL_WRITE_NO_RESP_EVT */
      break;

    case CUSTOM_STM_HEIGHTCAL_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_HEIGHTCAL_WRITE_EVT */

      /* USER CODE END CUSTOM_STM_HEIGHTCAL_WRITE_EVT */
      break;

    case CUSTOM_STM_NOTIFICATION_COMPLETE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */

      /* USER CODE END CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /**
       * It could be the enable notification is received twice without the disable notification in between
       */
      HW_TS_Stop(Custom_App_Context.TimerMeasurement_Id);
      HW_TS_Start(Custom_App_Context.TimerMeasurement_Id, MEASUREMENT_INTERVAL);
      // HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(TCS_LED_GPIO_Port, TCS_LED_Pin, GPIO_PIN_SET);
      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */
      HW_TS_Stop(Custom_App_Context.TimerMeasurement_Id);
      // HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(TCS_LED_GPIO_Port, TCS_LED_Pin, GPIO_PIN_RESET);
      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */
  MPR121_Init(&mpr, &hi2c1, MPR121_TOUCH_THRESHOLD_DEFAULT, MPR121_RELEASE_THRESHOLD_DEFAULT);
  APDS9960_Init(&apds, &hi2c1);
  APDS9960_EnableLightSensor(&apds, 0);
  UTIL_SEQ_RegTask(1 << CFG_TASK_MEASUREMENT_ID, UTIL_SEQ_RFU, MeasurementTask);
  Custom_App_Context.MeasurementHeight.raw = 0.0;
  Custom_App_Context.MeasurementHeight.mm = 0.0;
  Custom_App_Context.MeasurementColor.red = 0;
  Custom_App_Context.MeasurementColor.green = 0;
  Custom_App_Context.MeasurementColor.blue = 0;
  Custom_App_Context.CalibrationHight.h1 = 0.0;
  Custom_App_Context.CalibrationHight.r1 = 0.0;
  Custom_App_Context.CalibrationHight.h2 = 0.0;
  Custom_App_Context.CalibrationHight.r2 = 0.0;

  if (!Custom_App_CalibrationData_Load(&g_heightCalibration)) {
    Custom_App_Calibration_Defaults(&g_heightCalibration);
    Custom_App_CalibrationData_Save(&g_heightCalibration);
  }

  Custom_STM_App_Update_Char(CUSTOM_STM_HEIGHT, (uint8_t *) &Custom_App_Context.MeasurementHeight.raw);
  Custom_STM_App_Update_Char(CUSTOM_STM_COLOR, (uint8_t *) &Custom_App_Context.MeasurementColor.red);
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(Custom_App_Context.TimerMeasurement_Id), hw_ts_Repeated, MeasurementTask);
  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */
void Custom_App_StartCalibration(uint8_t *data, uint16_t length)
{
  if (length != 16)
    return;

  memcpy(&Custom_App_Context.CalibrationHight.h1, &data[0],  sizeof(float));
  memcpy(&Custom_App_Context.CalibrationHight.r1, &data[4],  sizeof(float));
  memcpy(&Custom_App_Context.CalibrationHight.h2, &data[8],  sizeof(float));
  memcpy(&Custom_App_Context.CalibrationHight.r2, &data[12], sizeof(float));

  // Calculate calibration
  Calibration_Data_t cal;
  cal.scale  = (Custom_App_Context.CalibrationHight.h2 - Custom_App_Context.CalibrationHight.h1)
			       / (Custom_App_Context.CalibrationHight.r2 - Custom_App_Context.CalibrationHight.r1);
  cal.offset = Custom_App_Context.CalibrationHight.h1 - (cal.scale * Custom_App_Context.CalibrationHight.r1);
  cal.crc = Custom_App_CalculateCRC32(&cal, sizeof(Calibration_Data_t) - sizeof(uint32_t));
  
  g_heightCalibration = cal;

  Custom_App_CalibrationData_Save(&cal);
}

uint32_t Custom_App_CalculateCRC32(const void *data, size_t length)
{
    uint32_t crc = CALIBRATION_CRC_INIT;
    const uint8_t *p = (const uint8_t*)data;

    while (length--) {
        crc ^= *p++;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc >> 1) ^ (0xEDB88320U & -(crc & 1));
    }
    return ~crc;
}

uint8_t Custom_App_CalibrationData_IsValid(const Calibration_Data_t *cal)
{
  uint32_t crc_calc = Custom_App_CalculateCRC32(cal, sizeof(Calibration_Data_t) - sizeof(uint32_t));
  return (crc_calc == cal->crc);
}

void Custom_App_CalibrationData_Save(const Calibration_Data_t *cal)
{
  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef erase;
  uint32_t pageError;
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Page = 200;
  erase.NbPages = 1;

  if (HAL_FLASHEx_Erase(&erase, &pageError) != HAL_OK) {
	  HAL_FLASH_Lock();
	  return;
  }

  // Program first doubleword (bytes 0-7: scale and offset)
  uint64_t *src = (uint64_t*)cal;
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, CALIBRATION_FLASH_ADDRESS, src[0]) != HAL_OK) {
    HAL_FLASH_Lock();
    return;
  }

  // Program second doubleword (bytes 8-15): crc in lower 32 bits, 0xFFFFFFFF in upper 32 bits
  uint64_t last_doubleword = ((uint64_t)0xFFFFFFFF << 32) | (uint64_t)cal->crc;
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, CALIBRATION_FLASH_ADDRESS + 8, last_doubleword) != HAL_OK) {
    HAL_FLASH_Lock();
    return;
  }

  HAL_FLASH_Lock();
}

uint8_t Custom_App_CalibrationData_Load(Calibration_Data_t *out)
{
  memcpy(out, (void *)CALIBRATION_FLASH_ADDRESS, sizeof(Calibration_Data_t));
  if (Custom_App_CalibrationData_IsValid(out))
  {
    return 1;
  }
  
  out->scale = 1.0f;
  out->offset = 0.0f;
  out->crc = Custom_App_CalculateCRC32(out, sizeof(Calibration_Data_t) - sizeof(uint32_t));
  return 0;
}

void Custom_App_Calibration_Defaults(Calibration_Data_t *out)
{
  out->scale  = 1.0f;
  out->offset = 0.0f;
  out->crc = Custom_App_CalculateCRC32(out, sizeof(Calibration_Data_t) - sizeof(uint32_t));
}

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* LiquidMonitoringService */
__USED void Custom_Height_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Height_UC_1*/

  /* USER CODE END Height_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_HEIGHT, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Height_UC_Last*/

  /* USER CODE END Height_UC_Last*/
  return;
}

void Custom_Height_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Height_NS_1*/

  /* USER CODE END Height_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_HEIGHT, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Height_NS_Last*/

  /* USER CODE END Height_NS_Last*/

  return;
}

__USED void Custom_Color_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Color_UC_1*/

  /* USER CODE END Color_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_COLOR, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Color_UC_Last*/

  /* USER CODE END Color_UC_Last*/
  return;
}

void Custom_Color_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Color_NS_1*/

  /* USER CODE END Color_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_COLOR, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Color_NS_Last*/

  /* USER CODE END Color_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
static void MeasurementTask(void)
{
  HAL_StatusTypeDef status;
  float rawHeight = (float) MPR121_BaseLineData(&mpr, 3);
  status = APDS9960_ReadRGBLight(&apds, &redValue, &greenValue, &blueValue);
  if (status != HAL_OK) {
    return;
  }

  Custom_App_Context.MeasurementHeight.raw = rawHeight;
  Custom_App_Context.MeasurementHeight.mm = (rawHeight * g_heightCalibration.scale) + g_heightCalibration.offset;
  Custom_App_Context.MeasurementColor.red = redValue;
  Custom_App_Context.MeasurementColor.green = greenValue;
  Custom_App_Context.MeasurementColor.blue = blueValue;

  Custom_STM_App_Update_Char(CUSTOM_STM_HEIGHT, (uint8_t *) &Custom_App_Context.MeasurementHeight.raw);
	Custom_STM_App_Update_Char(CUSTOM_STM_COLOR,
			(uint8_t*) &Custom_App_Context.MeasurementColor.red);
}
/* USER CODE END FD_LOCAL_FUNCTIONS*/
