/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.h
  * @author  MCD Application Team
  * @brief   Header for custom_app.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CUSTOM_APP_H
#define CUSTOM_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  CUSTOM_CONN_HANDLE_EVT,
  CUSTOM_DISCON_HANDLE_EVT,
} Custom_App_Opcode_Notification_evt_t;

typedef struct
{
  Custom_App_Opcode_Notification_evt_t     Custom_Evt_Opcode;
  uint16_t                                 ConnectionHandle;
} Custom_App_ConnHandle_Not_evt_t;
/* USER CODE BEGIN ET */
typedef struct {
  float raw;
  float mm;
} Height_MeasVal_t;

typedef struct {
  uint16_t red;
  uint16_t green;
  uint16_t blue;
} Color_MeasVal_t;

typedef struct {
  float h1; // Known reference height 1 (mm)
  float r1; // Raw value 1
  float h2; // Known reference height 2 (mm)
  float r2; // Raw value 2
} Height_CalVal_t;

typedef struct {
	float scale;  // mm per raw count
	float offset; // mm
	uint32_t crc; // CRC32 of scale + offset
} Calibration_Data_t;

#define CALIBRATION_FLASH_ADDRESS  ((uint32_t)0x080C8000U)  // Page 200 in 1MB flash (STM32WB55CGU6)
#define CALIBRATION_CRC_INIT   0xFFFFFFFFU

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ---------------------------------------------*/
void Custom_APP_Init(void);
void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification);
/* USER CODE BEGIN EF */
void Custom_App_StartCalibration(uint8_t *data, uint16_t length);

uint32_t Custom_App_CalculateCRC32(const void *data, size_t length);
uint8_t Custom_App_CalibrationData_IsValid(const Calibration_Data_t *cal);
void Custom_App_CalibrationData_Save(const Calibration_Data_t *cal);
uint8_t Custom_App_CalibrationData_Load(Calibration_Data_t *out);
void Custom_App_Calibration_Defaults(Calibration_Data_t *out);
/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_APP_H */
