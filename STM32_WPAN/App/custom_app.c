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

uint16_t capValues[12];

uint16_t redValue;
uint16_t greenValue;
uint16_t blueValue;
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
      HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */
      HW_TS_Stop(Custom_App_Context.TimerMeasurement_Id);
      HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
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
  Custom_App_Context.MeasurementHeight.value = 0;
  Custom_App_Context.MeasurementColor.red = 0;
  Custom_App_Context.MeasurementColor.green = 0;
  Custom_App_Context.MeasurementColor.blue = 0;
  Custom_STM_App_Update_Char(CUSTOM_STM_HEIGHT, (uint8_t *) &Custom_App_Context.MeasurementHeight.value);
  Custom_STM_App_Update_Char(CUSTOM_STM_COLOR, (uint8_t *) &Custom_App_Context.MeasurementColor.red);
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(Custom_App_Context.TimerMeasurement_Id), hw_ts_Repeated, MeasurementTask);
  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

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
  capValues[3] = MPR121_BaseLineData(&mpr, 3);
  status = APDS9960_ReadRGBLight(&apds, &redValue, &greenValue, &blueValue);
  if (status != HAL_OK) {
    return;
  }
  Custom_App_Context.MeasurementHeight.value = capValues[3];
  Custom_App_Context.MeasurementColor.red = redValue;
  Custom_App_Context.MeasurementColor.green = greenValue;
  Custom_App_Context.MeasurementColor.blue = blueValue;

  Custom_STM_App_Update_Char(CUSTOM_STM_HEIGHT, (uint8_t *) &Custom_App_Context.MeasurementHeight.value);
  Custom_STM_App_Update_Char(CUSTOM_STM_COLOR, (uint8_t *) &Custom_App_Context.MeasurementColor.red);
}
/* USER CODE END FD_LOCAL_FUNCTIONS*/
