/*
 * mpr121.h
 *
 *  Created on: Mar 14, 2025
 *      Author: kf1375
 */

#ifndef MPR121_H
#define MPR121_H

#include "stm32wbxx_hal.h"
#include "stdint.h"

#define MPR121_ADDR 0x5A                   ///< default I2C address
#define MPR121_TOUCH_THRESHOLD_DEFAULT 12  ///< default touch threshold value
#define MPR121_RELEASE_THRESHOLD_DEFAULT 6 ///< default relese threshold value

/*!
 *  Device register map
 */
enum {
  MPR121_TOUCHSTATUS_L = 0x00,
  MPR121_TOUCHSTATUS_H = 0x01,
  MPR121_FILTDATA_0L = 0x04,
  MPR121_FILTDATA_0H = 0x05,
  MPR121_BASELINE_0 = 0x1E,
  MPR121_MHDR = 0x2B,
  MPR121_NHDR = 0x2C,
  MPR121_NCLR = 0x2D,
  MPR121_FDLR = 0x2E,
  MPR121_MHDF = 0x2F,
  MPR121_NHDF = 0x30,
  MPR121_NCLF = 0x31,
  MPR121_FDLF = 0x32,
  MPR121_NHDT = 0x33,
  MPR121_NCLT = 0x34,
  MPR121_FDLT = 0x35,

  MPR121_TOUCHTH_0 = 0x41,
  MPR121_RELEASETH_0 = 0x42,
  MPR121_DEBOUNCE = 0x5B,
  MPR121_CONFIG1 = 0x5C,
  MPR121_CONFIG2 = 0x5D,
  MPR121_CHARGECURR_0 = 0x5F,
  MPR121_CHARGETIME_1 = 0x6C,
  MPR121_ECR = 0x5E,
  MPR121_AUTOCONFIG0 = 0x7B,
  MPR121_AUTOCONFIG1 = 0x7C,
  MPR121_UPLIMIT = 0x7D,
  MPR121_LOWLIMIT = 0x7E,
  MPR121_TARGETLIMIT = 0x7F,

  MPR121_GPIODIR = 0x76,
  MPR121_GPIOEN = 0x77,
  MPR121_GPIOSET = 0x78,
  MPR121_GPIOCLR = 0x79,
  MPR121_GPIOTOGGLE = 0x7A,

  MPR121_SOFTRESET = 0x80,
};

typedef struct {
  I2C_HandleTypeDef *hi2c; // Pointer to I2C handle
} MPR121_t;

HAL_StatusTypeDef MPR121_Init(MPR121_t *dev, I2C_HandleTypeDef *hi2c, uint8_t touchThreshold, uint8_t releaseThreshold);
HAL_StatusTypeDef MPR121_SetThresholds(MPR121_t *dev, uint8_t touch, uint8_t release);
uint16_t MPR121_FilteredData(MPR121_t *dev, uint8_t channel);
uint16_t MPR121_BaseLineData(MPR121_t *dev, uint8_t channel);
uint16_t MPR121_Touched(MPR121_t *dev);

#endif  // MPR121_H
