/*
 * apds9960.c
 *
 *  Created on: Mar 14, 2025
 *      Author: kf1375
 */

#include "apds9960.h"

/* Gesture processing */
static void APDS9960_ResetGestureParameters(APDS9960_t *dev);
static HAL_StatusTypeDef APDS9960_ProcessGestureData(APDS9960_t *dev);
static HAL_StatusTypeDef APDS9960_DecodeGesture(APDS9960_t *dev);

/* Proximity Interrupt Threshold */
static uint8_t APDS9960_GetProxIntLowThresh(APDS9960_t *dev);
static HAL_StatusTypeDef APDS9960_SetProxIntLowThresh(APDS9960_t *dev, uint8_t threshold);
static uint8_t APDS9960_GetProxIntHighThresh(APDS9960_t *dev);
static HAL_StatusTypeDef APDS9960_SetProxIntHighThresh(APDS9960_t *dev, uint8_t threshold);

/* LED Boost Control */
static uint8_t APDS9960_GetLEDBoost(APDS9960_t *dev);
static HAL_StatusTypeDef APDS9960_SetLEDBoost(APDS9960_t *dev, uint8_t boost);

/* Proximity photodiode select */
static uint8_t APDS9960_GetProxGainCompEnable(APDS9960_t *dev);
static HAL_StatusTypeDef APDS9960_SetProxGainCompEnable(APDS9960_t *dev, uint8_t enable);
static uint8_t APDS9960_GetProxPhotoMask(APDS9960_t *dev);
static HAL_StatusTypeDef APDS9960_SetProxPhotoMask(APDS9960_t *dev, uint8_t mask);

/* Gesture threshold control */
static uint8_t APDS9960_GetGestureEnterThresh(APDS9960_t *dev);
static HAL_StatusTypeDef APDS9960_SetGestureEnterThresh(APDS9960_t *dev, uint8_t threshold);
static uint8_t APDS9960_GetGestureExitThresh(APDS9960_t *dev);
static HAL_StatusTypeDef APDS9960_SetGestureExitThresh(APDS9960_t *dev, uint8_t threshold);

/* Gesture LED, gain, and time control */
static uint8_t APDS9960_GetGestureWaitTime(APDS9960_t *dev);
static HAL_StatusTypeDef APDS9960_SetGestureWaitTime(APDS9960_t *dev, uint8_t time);

/* Gesture mode */
static uint8_t APDS9960_GetGestureMode(APDS9960_t *dev);
static HAL_StatusTypeDef APDS9960_SetGestureMode(APDS9960_t *dev, uint8_t mode);

/* Raw I2C Commands */
static HAL_StatusTypeDef APDS9960_WriteByte(APDS9960_t *dev, uint8_t val);
static HAL_StatusTypeDef APDS9960_WriteRegister(APDS9960_t *dev, uint8_t reg, uint8_t value);
static HAL_StatusTypeDef APDS9960_ReadRegister(APDS9960_t *dev, uint8_t reg, uint8_t *data);

/*******************************************************************************
 * Public methods for controlling the APDS-9960
 ******************************************************************************/

/**
 * @brief Configures I2C communications and initializes registers to defaults
 * @param dev Pointer to APDS9960 device structure
 * @return HAL_OK if initialized successfully. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_Init(APDS9960_t *dev, I2C_HandleTypeDef *hi2c)
{
  uint8_t id;

  dev->gesture_ud_delta = 0;
  dev->gesture_lr_delta = 0;

  dev->gesture_ud_count = 0;
  dev->gesture_lr_count = 0;

  dev->gesture_near_count = 0;
  dev->gesture_far_count = 0;

  dev->gesture_state = 0;
  dev->gesture_motion = DIR_NONE;

  dev->hi2c = hi2c;

  /* Read ID register and check against known values for APDS-9960 */
  if (APDS9960_ReadRegister(dev, APDS9960_ID, &id) != HAL_OK) {
    return HAL_ERROR;
  }

  if (!(id == APDS9960_ID_1 || id == APDS9960_ID_2)) {
    return HAL_ERROR;
  }

  /* Set ENABLE register to 0 (disable all features) */
  if (APDS9960_SetMode(dev, ALL, OFF) != HAL_OK) {
    return HAL_ERROR;
  }

  /* Set default values for ambient light and proximity registers */
  if (APDS9960_WriteRegister(dev, APDS9960_ATIME, DEFAULT_ATIME) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_WriteRegister(dev, APDS9960_WTIME, DEFAULT_WTIME) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_WriteRegister(dev, APDS9960_PPULSE, DEFAULT_PROX_PPULSE) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_WriteRegister(dev, APDS9960_POFFSET_UR, DEFAULT_POFFSET_UR) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_WriteRegister(dev, APDS9960_POFFSET_DL, DEFAULT_POFFSET_DL) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_WriteRegister(dev, APDS9960_CONFIG1, DEFAULT_CONFIG1) != HAL_OK) {
    return HAL_ERROR;
  }

  if (APDS9960_SetLedDrive(dev, DEFAULT_LDRIVE) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_SetProximityGain(dev, DEFAULT_PGAIN) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_SetAmbientLightGain(dev, DEFAULT_AGAIN) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_SetProxIntLowThresh(dev, DEFAULT_PILT) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_SetProxIntHighThresh(dev, DEFAULT_PIHT) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_SetLightIntLowThreshold(dev, DEFAULT_AILT) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_SetLightIntHighThreshold(dev, DEFAULT_AIHT) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_WriteRegister(dev, APDS9960_PERS, DEFAULT_PERS) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_WriteRegister(dev, APDS9960_CONFIG2, DEFAULT_CONFIG2) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_WriteRegister(dev, APDS9960_CONFIG3, DEFAULT_CONFIG3) != HAL_OK) {
    return HAL_ERROR;
  }

  /* Set default values for gesture sense registers */
  if (APDS9960_SetGestureEnterThresh(dev, DEFAULT_GPENTH) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_SetGestureExitThresh(dev, DEFAULT_GEXTH) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_WriteRegister(dev, APDS9960_GCONF1, DEFAULT_GCONF1) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_SetGestureGain(dev, DEFAULT_GGAIN) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_SetGestureLedDrive(dev, DEFAULT_GLDRIVE) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_SetGestureWaitTime(dev, DEFAULT_GWTIME) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_WriteRegister(dev, APDS9960_GOFFSET_U, DEFAULT_GOFFSET) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_WriteRegister(dev, APDS9960_GOFFSET_D, DEFAULT_GOFFSET) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_WriteRegister(dev, APDS9960_GOFFSET_L, DEFAULT_GOFFSET) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_WriteRegister(dev, APDS9960_GOFFSET_R, DEFAULT_GOFFSET) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_WriteRegister(dev, APDS9960_GPULSE, DEFAULT_GPULSE) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_WriteRegister(dev, APDS9960_GCONF3, DEFAULT_GCONF3) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_SetGestureIntEnable(dev, DEFAULT_GIEN) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief Reads and returns the contents of the STATUS register
 * @param dev Pointer to APDS9960 device structure
 * @return Contents of the STATUS register. 0xFF if error.
 */
uint8_t APDS9960_GetStatus(APDS9960_t *dev)
{
  uint8_t status_value;
  if (!APDS9960_ReadRegister(dev, APDS9960_STATUS, &status_value) != HAL_OK) {
    return APDS9960_ERROR;
  }

  return status_value;
}

/**
 * @brief Reads and returns the contents of the ENABLE register
 * @param dev Pointer to APDS9960 device structure
 * @return Contents of the ENABLE register. 0xFF if error.
 */
uint8_t APDS9960_GetMode(APDS9960_t *dev)
{
  uint8_t enable_value;
  if (APDS9960_ReadRegister(dev, APDS9960_ENABLE, &enable_value) != HAL_OK) {
    return APDS9960_ERROR;
  }

  return enable_value;
}

/**
 * @brief Enables or disables a feature in the APDS-9960
 * @param dev Pointer to APDS9960 device structure
 * @param mode which feature to enable
 * @param enable ON (1) or OFF (0)
 * @return HAL_OK if operation success. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_SetMode(APDS9960_t *dev, uint8_t mode, uint8_t enable)
{
  uint8_t reg_val;

  reg_val = APDS9960_GetMode(dev);
  if (reg_val == APDS9960_ERROR) {
    return HAL_ERROR;
  }

  /* Change bit(s) in ENABLE register */
  enable = enable & 0x01;
  if (mode >= 0 && mode <= 6) {
    if (enable) {
      reg_val |= (1 << mode);
    } else {
      reg_val &= ~(1 << mode);
    }
  } else if (mode == ALL) {
    if (enable) {
      reg_val = 0x7F;
    } else {
      reg_val = 0x00;
    }
  }

  if (APDS9960_WriteRegister(dev, APDS9960_ENABLE, reg_val) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief Starts the light (R/G/B/Ambient) sensor on the APDS-9960
 * @param dev Pointer to APDS9960 device structure
 * @param interrupts true to enable hardware interrupt on high or low light
 * @return HAL_OK if sensor enabled correctly. HAL_ERROR on error.
 */
HAL_StatusTypeDef APDS9960_EnableLightSensor(APDS9960_t *dev, uint8_t interrupts)
{
  if (APDS9960_SetAmbientLightGain(dev, DEFAULT_AGAIN) != HAL_OK) {
    return HAL_ERROR;
  }

  if (interrupts) {
    if (APDS9960_SetAmbientLightIntEnable(dev, 1) != HAL_OK) {
      return HAL_ERROR;
    }
  } else {
    if (APDS9960_SetAmbientLightIntEnable(dev, 0) != HAL_OK) {
      return HAL_ERROR;
    }
  }

  if (APDS9960_EnablePower(dev) != HAL_OK) {
    return HAL_ERROR;
  }

  if (APDS9960_SetMode(dev, AMBIENT_LIGHT, 1) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief Ends the light sensor on the APDS-9960
 * @param dev Pointer to APDS9960 device structure
 * @return HAL_OK if sensor disabled correctly. HAL_ERROR on error.
 */
HAL_StatusTypeDef APDS9960_DisableLightSensor(APDS9960_t *dev)
{
  if (APDS9960_SetAmbientLightIntEnable(dev, 0) != HAL_OK) {
    return HAL_ERROR;
  }

  if (APDS9960_SetMode(dev, AMBIENT_LIGHT, 0) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief Starts the proximity sensor on the APDS-9960
 * @param dev Pointer to APDS9960 device structure
 * @param interrupts true to enable hardware external interrupt on proximity
 * @return HAL_OK if sensor enabled correctly. HAL_ERROR on error.
 */
HAL_StatusTypeDef APDS9960_EnableProximitySensor(APDS9960_t *dev, uint8_t interrupts)
{
  if (APDS9960_SetProximityGain(dev, DEFAULT_PGAIN) != HAL_OK) {
    return HAL_ERROR;
  }

  if (APDS9960_SetLedDrive(dev, DEFAULT_LDRIVE) != HAL_OK) {
    return HAL_ERROR;
  }

  if (interrupts) {
    if (APDS9960_SetProximityIntEnable(dev, 1) != HAL_OK) {
      return HAL_ERROR;
    }
  } else {
    if (APDS9960_SetProximityIntEnable(dev, 0) != HAL_OK) {
      return HAL_ERROR;
    }
  }

  if (APDS9960_EnablePower(dev) != HAL_OK) {
    return HAL_ERROR;
  }

  if (APDS9960_SetMode(dev, PROXIMITY, 1) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief Ends the proximity sensor on the APDS-9960
 * @param dev Pointer to APDS9960 device structure
 * @return HAL_OK if sensor disabled correctly. HAL_ERROR on error.
 */
HAL_StatusTypeDef APDS9960_DisableProximitySensor(APDS9960_t *dev)
{

  if (APDS9960_SetProximityIntEnable(dev, 0) != HAL_OK) {
    return HAL_ERROR;
  }
  if (APDS9960_SetMode(dev, PROXIMITY, 0) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief Starts the gesture recognition engine on the APDS-9960
 * @param dev Pointer to APDS9960 device structure
 * @param interrupts true to enable hardware external interrupt on gesture
 * @return HAL_OK if engine enabled correctly. HAL_ERROR on error.
 */
HAL_StatusTypeDef APDS9960_EnableGestureSensor(APDS9960_t *dev, uint8_t interrupts)
{
  /* Enable gesture mode
   Set ENABLE to 0 (power off)
   Set WTIME to 0xFF
   Set AUX to LED_BOOST_300
   Enable PON, WEN, PEN, GEN in ENABLE
   */
  APDS9960_ResetGestureParameters(dev);
  if (APDS9960_WriteRegister(dev, APDS9960_WTIME, 0xFF) != HAL_OK) {
    return HAL_ERROR;
  }

  if (APDS9960_WriteRegister(dev, APDS9960_PPULSE, DEFAULT_GESTURE_PPULSE) != HAL_OK) {
    return HAL_ERROR;
  }

  if (APDS9960_SetLEDBoost(dev, LED_BOOST_300) != HAL_OK) {
    return HAL_ERROR;
  }

  if (interrupts) {
    if (APDS9960_SetGestureIntEnable(dev, 1) != HAL_OK) {
      return HAL_ERROR;
    }
  } else {
    if (APDS9960_SetGestureIntEnable(dev, 1) != HAL_OK) {
      return HAL_ERROR;
    }
  }

  if (APDS9960_SetGestureMode(dev, 1) != HAL_OK) {
    return HAL_ERROR;
  }

  if (APDS9960_EnablePower(dev) != HAL_OK) {
    return HAL_ERROR;
  }

  if (APDS9960_SetMode(dev, WAIT, 1) != HAL_OK) {
    return HAL_ERROR;
  }

  if (APDS9960_SetMode(dev, PROXIMITY, 1) != HAL_OK) {
    return HAL_ERROR;
  }

  if (APDS9960_SetMode(dev, GESTURE, 1) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief Ends the gesture recognition engine on the APDS-9960
 * @param dev Pointer to APDS9960 device structure
 * @return HAL_OK if engine disabled correctly. HAL_ERROR on error.
 */
HAL_StatusTypeDef APDS9960_DisableGestureSensor(APDS9960_t *dev)
{
  APDS9960_ResetGestureParameters(dev);
  if (APDS9960_SetGestureIntEnable(dev, 0) != HAL_OK) {
    return HAL_ERROR;
  }

  if (APDS9960_SetGestureMode(dev, 0) != HAL_OK) {
    return HAL_ERROR;
  }

  if (APDS9960_SetMode(dev, GESTURE, 0) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief Determines if there is a gesture available for reading
 * @param dev Pointer to APDS9960 device structure
 * @return True if gesture available. False otherwise.
 */
uint8_t APDS9960_IsGestureAvailable(APDS9960_t *dev)
{
  uint8_t value;
  /* Read value from GSTATUS register */
  if (APDS9960_ReadRegister(dev, APDS9960_GSTATUS, &value) != HAL_OK) {
    return 0;
  }

  /* Shift and mask out GVALID bit */
  value &= APDS9960_GVALID;

  /* Return true/false based on GVALID bit */
  if (value == 1) {
    return 1;
  } else {
    return 0;
  }
}

/**
 * @brief Processes a gesture event and returns best guessed gesture
 * @param dev Pointer to APDS9960 device structure
 * @return Number corresponding to gesture. -1 on error.
 */
int APDS9960_ReadGesture(APDS9960_t *dev)
{
  return 0;
}

/**
 * Turn the APDS-9960 on
 * @param dev Pointer to APDS9960 device structure
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_EnablePower(APDS9960_t *dev)
{
  return APDS9960_SetMode(dev, POWER, 1);
}

/**
 * Turn the APDS-9960 off
 * @param[in] dev Pointer to APDS9960 device structure
 * @return True if operation successful. False otherwise.
 */
HAL_StatusTypeDef APDS9960_DisablePower(APDS9960_t *dev)
{
  return APDS9960_SetMode(dev, POWER, 0);
}

/*******************************************************************************
 * Ambient light and color sensor controls
 ******************************************************************************/

/**
 * @brief Reads the ambient (clear) light level as a 16-bit value
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[out] val value of the light sensor.
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_ReadAmbientLight(APDS9960_t *dev, uint16_t *val)
{
  uint8_t val_byte;
  *val = 0;

  /* Read value from clear channel, low byte register */
  if (APDS9960_ReadRegister(dev, APDS9960_CDATAL, &val_byte) != HAL_OK) {
    return HAL_ERROR;
  }
  *val = val_byte;

  /* Read value from clear channel, high byte register */
  if (APDS9960_ReadRegister(dev, APDS9960_CDATAL, &val_byte) != HAL_OK) {
    return HAL_ERROR;
  }
  *val = *val + ((uint16_t) val_byte << 8);

  return HAL_OK;
}

/**
 * @brief Reads the red light level as a 16-bit value
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[out] val value of the light sensor.
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_ReadRedLight(APDS9960_t *dev, uint16_t *val)
{
  uint8_t val_byte;
  *val = 0;

  /* Read value from clear channel, low byte register */
  if (APDS9960_ReadRegister(dev, APDS9960_RDATAL, &val_byte) != HAL_OK) {
    return HAL_ERROR;
  }
  *val = val_byte;

  /* Read value from clear channel, high byte register */
  if (APDS9960_ReadRegister(dev, APDS9960_RDATAL, &val_byte) != HAL_OK) {
    return HAL_ERROR;
  }
  *val = *val + ((uint16_t) val_byte << 8);

  return HAL_OK;
}

/**
 * @brief Reads the green light level as a 16-bit value
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[out] val value of the light sensor.
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_ReadGreenLight(APDS9960_t *dev, uint16_t *val)
{
  uint8_t val_byte;
  *val = 0;

  /* Read value from clear channel, low byte register */
  if (APDS9960_ReadRegister(dev, APDS9960_GDATAL, &val_byte) != HAL_OK) {
    return HAL_ERROR;
  }
  *val = val_byte;

  /* Read value from clear channel, high byte register */
  if (APDS9960_ReadRegister(dev, APDS9960_GDATAL, &val_byte) != HAL_OK) {
    return HAL_ERROR;
  }
  *val = *val + ((uint16_t) val_byte << 8);

  return HAL_OK;
}

/**
 * @brief Reads the blue light level as a 16-bit value
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[out] val value of the light sensor.
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_ReadBlueLight(APDS9960_t *dev, uint16_t *val)
{
  uint8_t val_byte;
  *val = 0;

  /* Read value from clear channel, low byte register */
  if (APDS9960_ReadRegister(dev, APDS9960_BDATAL, &val_byte) != HAL_OK) {
    return HAL_ERROR;
  }
  *val = val_byte;

  /* Read value from clear channel, high byte register */
  if (APDS9960_ReadRegister(dev, APDS9960_BDATAL, &val_byte) != HAL_OK) {
    return HAL_ERROR;
  }
  *val = *val + ((uint16_t) val_byte << 8);

  return HAL_OK;
}

/**
 * @brief Reads the RGB light level as a 16-bit value
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[out] redVal value of the red light sensor.
 * @param[out] greenVal value of the green light sensor.
 * @param[out] blueVal value of the blue light sensor.
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_ReadRGBLight(APDS9960_t *dev, uint16_t *redVal, uint16_t *greenVal, uint16_t *blueVal)
{
  HAL_StatusTypeDef status;
  status = APDS9960_ReadRedLight(dev, redVal);
  if (status != HAL_OK) {
    return HAL_ERROR;
  }
  status = APDS9960_ReadGreenLight(dev, greenVal);
  if (status != HAL_OK) {
    return HAL_ERROR;
  }
  status = APDS9960_ReadBlueLight(dev, blueVal);
  if (status != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}
/*******************************************************************************
 * Proximity sensor controls
 ******************************************************************************/

/**
 * @brief Reads the proximity level as an 8-bit value
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[out] val value of the proximity sensor.
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_ReadProximity(APDS9960_t *dev, uint8_t *val)
{
  *val = 0;

  /* Read value from proximity data register */
  if (APDS9960_ReadRegister(dev, APDS9960_PDATA, val) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/*******************************************************************************
 * High-level gesture controls
 ******************************************************************************/

/**
 * @brief Resets all the parameters in the gesture data member
 * @param[in] dev Pointer to APDS9960 device structure
 */
void APDS9960_ResetGestureParameters(APDS9960_t *dev)
{
  dev->gesture_data.index = 0;
  dev->gesture_data.total_gestures = 0;

  dev->gesture_ud_delta = 0;
  dev->gesture_lr_delta = 0;

  dev->gesture_ud_count = 0;
  dev->gesture_lr_count = 0;

  dev->gesture_near_count = 0;
  dev->gesture_far_count = 0;

  dev->gesture_state = 0;
  dev->gesture_motion = DIR_NONE;
}

/**
 * @brief Processes the raw gesture data to determine swipe direction
 * @param[in] dev Pointer to APDS9960 device structure
 * @return HAL_OK if near or far state seen. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_ProcessGestureData(APDS9960_t *dev)
{
  return HAL_ERROR;
}

/**
 * @brief Determines swipe direction or near/far state
 * @param[in] dev Pointer to APDS9960 device structure
 * @return HAL_OK if near/far event. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_DecodeGesture(APDS9960_t *dev)
{
  return HAL_ERROR;
}

/*******************************************************************************
 * Getters and setters for register values
 ******************************************************************************/

/**
 * @brief Returns the lower threshold for proximity detection
 * @param[in] dev Pointer to APDS9960 device structure
 * @return lower threshold
 */
uint8_t APDS9960_GetProxIntLowThresh(APDS9960_t *dev)
{
  uint8_t val;

  /* Read value from PILT register */
  if (APDS9960_ReadRegister(dev, APDS9960_PILT, &val) != HAL_OK) {
    val = 0;
  }

  return val;
}

/**
 * @brief Sets the lower threshold for proximity detection
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[in] threshold the lower proximity threshold
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_SetProxIntLowThresh(APDS9960_t *dev, uint8_t threshold)
{
  if (APDS9960_WriteRegister(dev, APDS9960_PILT, threshold) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief Returns the high threshold for proximity detection
 * @param[in] dev Pointer to APDS9960 device structure
 * @return high threshold
 */
uint8_t APDS9960_GetProxIntHighThresh(APDS9960_t *dev)
{
  uint8_t val;

  /* Read value from PIHT register */
  if (APDS9960_ReadRegister(dev, APDS9960_PIHT, &val) != HAL_OK) {
    val = 0;
  }

  return val;
}

/**
 * @brief Sets the high threshold for proximity detection
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[in] threshold the high proximity threshold
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_SetProxIntHighThresh(APDS9960_t *dev, uint8_t threshold)
{
  if (APDS9960_WriteRegister(dev, APDS9960_PIHT, threshold) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief Get the current LED boost value
 *
 * Value  Boost Current
 *   0        100%
 *   1        150%
 *   2        200%
 *   3        300%
 *
 * @param[in] dev Pointer to APDS9960 device structure
 * @return The LED boost value. 0xFF on failure.
 */
uint8_t APDS9960_GetLEDBoost(APDS9960_t *dev)
{
  uint8_t val;

  /* Read value from CONFIG2 register */
  if (APDS9960_ReadRegister(dev, APDS9960_CONFIG2, &val) != HAL_OK) {
    return APDS9960_ERROR;
  }

  /* Shift and mask out LED_BOOST bits */
  val = (val >> 4) & 0b00000011;

  return val;
}

/**
 * @brief Sets the LED current boost value
 *
 * Value  Boost Current
 *   0        100%
 *   1        150%
 *   2        200%
 *   3        300%
 *
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[in] drive the value (0-3) for current boost (100-300%)
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_SetLEDBoost(APDS9960_t *dev, uint8_t boost)
{
  uint8_t val;

  /* Read value from CONFIG2 register */
  if (APDS9960_ReadRegister(dev, APDS9960_CONFIG2, &val) != HAL_OK) {
    return HAL_ERROR;
  }

  /* Set bits in register to given value */
  boost &= 0b00000011;
  boost = boost << 4;
  val &= 0b11001111;
  val |= boost;

  /* Write register value back into CONFIG2 register */
  if (APDS9960_WriteRegister(dev, APDS9960_CONFIG2, val) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief Gets proximity gain compensation enable
 * @param[in] dev Pointer to APDS9960 device structure
 * @return 1 if compensation is enabled. 0 if not. 0xFF on error.
 */
uint8_t APDS9960_GetProxGainCompEnable(APDS9960_t *dev)
{
  uint8_t val;

  /* Read value from CONFIG3 register */
  if (APDS9960_ReadRegister(dev, APDS9960_CONFIG3, &val) != HAL_OK) {
    return APDS9960_ERROR;
  }

  /* Shift and mask out PCMP bits */
  val = (val >> 5) & 0b00000001;

  return val;
}

/**
 * @brief Sets the proximity gain compensation enable
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[in] enable 1 to enable compensation. 0 to disable compensation.
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_SetProxGainCompEnable(APDS9960_t *dev, uint8_t enable)
{
  uint8_t val;

  /* Read value from from register */
  if (APDS9960_ReadRegister(dev, APDS9960_CONFIG3, &val) != HAL_OK) {
    return HAL_ERROR;
  }

  /* Set bits in register to given value */
  enable &= 0b00000011;
  enable = enable << 4;
  val &= 0b11001111;
  val |= enable;

  /* Write register value back into CONFIG3 register */
  if (APDS9960_WriteRegister(dev, APDS9960_CONFIG3, val) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief Gets the current mask for enabled/disabled proximity photodiodes
 *
 * 1 = disabled, 0 = enabled
 * Bit    Photodiode
 *  3       UP
 *  2       DOWN
 *  1       LEFT
 *  0       RIGHT
 *
 * @param[in] dev Pointer to APDS9960 device structure
 * @return Current proximity mask for photodiodes. 0xFF on error.
 */
uint8_t APDS9960_GetProxPhotoMask(APDS9960_t *dev)
{
  uint8_t val;

  /* Read value from CONFIG3 register */
  if (APDS9960_ReadRegister(dev, APDS9960_CONFIG3, &val) != HAL_OK) {
    return APDS9960_ERROR;
  }

  /* Mask out photodiode enable mask bits */
  val &= 0b00001111;

  return val;
}

/**
 * @brief Sets the mask for enabling/disabling proximity photodiodes
 *
 * 1 = disabled, 0 = enabled
 * Bit    Photodiode
 *  3       UP
 *  2       DOWN
 *  1       LEFT
 *  0       RIGHT
 *
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[in] mask 4-bit mask value
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_SetProxPhotoMask(APDS9960_t *dev, uint8_t mask)
{
  uint8_t val;

  /* Read value from from register */
  if (APDS9960_ReadRegister(dev, APDS9960_CONFIG3, &val) != HAL_OK) {
    return HAL_ERROR;
  }

  /* Set bits in register to given value */
  mask &= 0b00001111;
  val &= 0b11110000;
  val |= mask;

  /* Write register value back into CONFIG3 register */
  if (APDS9960_WriteRegister(dev, APDS9960_CONFIG3, val) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief Gets the entry proximity threshold for gesture sensing
 * @param[in] dev Pointer to APDS9960 device structure
 * @return Current entry proximity threshold.
 */
uint8_t APDS9960_GetGestureEnterThresh(APDS9960_t *dev)
{
  uint8_t val;

  /* Read value from GPENTH register */
  if (APDS9960_ReadRegister(dev, APDS9960_GPENTH, &val) != HAL_OK) {
    val = 0;
  }

  return val;
}

/**
 * @brief Sets the entry proximity threshold for gesture sensing
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[in] threshold proximity value needed to start gesture mode
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_SetGestureEnterThresh(APDS9960_t *dev, uint8_t threshold)
{
  return APDS9960_WriteRegister(dev, APDS9960_GPENTH, threshold);
}

/**
 * @brief Gets the exit proximity threshold for gesture sensing
 * @param[in] dev Pointer to APDS9960 device structure
 * @return Current exit proximity threshold.
 */
uint8_t APDS9960_GetGestureExitThresh(APDS9960_t *dev)
{
  uint8_t val;

  /* Read value from GEXTH register */
  if (APDS9960_ReadRegister(dev, APDS9960_GEXTH, &val) != HAL_OK) {
    val = 0;
  }

  return val;
}

/**
 * @brief Sets the exit proximity threshold for gesture sensing
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[in] threshold proximity value needed to end gesture mode
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_SetGestureExitThresh(APDS9960_t *dev, uint8_t threshold)
{
  return APDS9960_WriteRegister(dev, APDS9960_GEXTH, threshold);
}

/**
 * @brief Returns LED drive strength for proximity and ALS
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @param[in] dev Pointer to APDS9960 device structure
 * @return the value of the LED drive strength. 0xFF on failure.
 */
uint8_t APDS9960_GetLedDrive(APDS9960_t *dev)
{
  uint8_t val;

  /* Read value from CONTROL register */
  if (APDS9960_ReadRegister(dev, APDS9960_CONTROL, &val) != HAL_OK) {
    return APDS9960_ERROR;
  }

  /* Shift and mask out LED drive bits */
  val = (val >> 6) & 0b00000011;

  return val;
}

/**
 * @brief Sets the LED drive strength for proximity and ALS
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[in] drive the value (0-3) for the LED drive strength
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_SetLedDrive(APDS9960_t *dev, uint8_t drive)
{
  uint8_t val;

  /* Read value from CONTROL register */
  if (APDS9960_ReadRegister(dev, APDS9960_CONTROL, &val) != HAL_OK) {
    return HAL_ERROR;
  }

  /* Set bits in register to given value */
  drive &= 0b00000011;
  drive = drive << 6;
  val &= 0b00111111;
  val |= drive;

  /* Write register value back into CONTROL register */
  if (APDS9960_WriteRegister(dev, APDS9960_CONTROL, val) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief Gets the drive current of the LED during gesture mode
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @param[in] dev Pointer to APDS9960 device structure
 * @return the LED drive current value. 0xFF on error.
 */
uint8_t APDS9960_GetGestureLedDrive(APDS9960_t *dev)
{
  uint8_t val;

  /* Read value from GCONF2 register */
  if (APDS9960_ReadRegister(dev, APDS9960_GCONF2, &val) != HAL_OK) {
    return APDS9960_ERROR;
  }

  /* Shift and mask out GLDRIVE bits */
  val = (val >> 3) & 0b00000011;

  return val;
}

/**
 * @brief Sets the LED drive current during gesture mode
 *
 * Value    LED Current
 *   0        100 mA
 *   1         50 mA
 *   2         25 mA
 *   3         12.5 mA
 *
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[in] drive the value for the LED drive current
 * @return True if operation successful. False otherwise.
 */
HAL_StatusTypeDef APDS9960_SetGestureLedDrive(APDS9960_t *dev, uint8_t drive)
{
  uint8_t val;

  /* Read value from GCONF2 register */
  if (APDS9960_ReadRegister(dev, APDS9960_GCONF2, &val) != HAL_OK) {
    return HAL_ERROR;
  }

  /* Set bits in register to given value */
  drive &= 0b00000011;
  drive = drive << 3;
  val &= 0b11100111;
  val |= drive;

  /* Write register value back into CONTROL register */
  if (APDS9960_WriteRegister(dev, APDS9960_GCONF2, val) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief Returns receiver gain for the ambient light sensor (ALS)
 *
 * Value    Gain
 *   0        1x
 *   1        4x
 *   2       16x
 *   3       64x
 *
 * @param[in] dev Pointer to APDS9960 device structure
 * @return the value of the ALS gain. 0xFF on failure.
 */
uint8_t APDS9960_GetAmbientLightGain(APDS9960_t *dev)
{
  uint8_t val;

  /* Read value from CONTROL register */
  if (APDS9960_ReadRegister(dev, APDS9960_CONTROL, &val) != HAL_OK) {
    return APDS9960_ERROR;
  }

  /* Shift and mask out ADRIVE bits */
  val &= 0b00000011;

  return val;
}

/**
 * @brief Sets the receiver gain for the ambient light sensor (ALS)
 *
 * Value    Gain
 *   0        1x
 *   1        4x
 *   2       16x
 *   3       64x
 *
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[in] gain the value (0-3) for the gain
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_SetAmbientLightGain(APDS9960_t *dev, uint8_t gain)
{
  uint8_t val;

  /* Read value from CONTROL register */
  if (APDS9960_ReadRegister(dev, APDS9960_CONTROL, &val) != HAL_OK) {
    return HAL_ERROR;
  }

  /* Set bits in register to given value */
  gain &= 0b00000011;
  val &= 0b11111100;
  val |= gain;

  /* Write register value back into CONTROL register */
  if (APDS9960_WriteRegister(dev, APDS9960_CONTROL, val) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief Returns receiver gain for proximity detection
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @param[in] dev Pointer to APDS9960 device structure
 * @return the value of the proximity gain. 0xFF on failure.
 */
uint8_t APDS9960_GetProximityGain(APDS9960_t *dev)
{
  uint8_t val;

  /* Read value from CONTROL register */
  if (APDS9960_ReadRegister(dev, APDS9960_CONTROL, &val) != HAL_OK) {
    return APDS9960_ERROR;
  }

  /* Shift and mask out PDRIVE bits */
  val = (val >> 2) & 0b00000011;

  return val;
}

/**
 * @brief Sets the receiver gain for proximity detection
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[in] gain the value (0-3) for the gain
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_SetProximityGain(APDS9960_t *dev, uint8_t gain)
{
  uint8_t val;

  /* Read value from CONTROL register */
  if (APDS9960_ReadRegister(dev, APDS9960_CONTROL, &val) != HAL_OK) {
    return HAL_ERROR;
  }

  /* Set bits in register to given value */
  gain &= 0b00000011;
  gain = gain << 2;
  val &= 0b11110011;
  val |= gain;

  /* Write register value back into CONTROL register */
  if (APDS9960_WriteRegister(dev, APDS9960_CONTROL, val) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief Gets the gain of the photodiode during gesture mode
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @param[in] dev Pointer to APDS9960 device structure
 * @return the current photodiode gain. 0xFF on error.
 */
uint8_t APDS9960_GetGestureGain(APDS9960_t *dev)
{
  uint8_t val;

  /* Read value from GCONF2 register */
  if (APDS9960_ReadRegister(dev, APDS9960_GCONF2, &val) != HAL_OK) {
    return APDS9960_ERROR;
  }

  /* Shift and mask out GGAIN bits */
  val = (val >> 5) & 0b00000011;

  return val;
}

/**
 * @brief Sets the gain of the photodiode during gesture mode
 *
 * Value    Gain
 *   0       1x
 *   1       2x
 *   2       4x
 *   3       8x
 *
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[in] gain the value for the photodiode gain
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_SetGestureGain(APDS9960_t *dev, uint8_t gain)
{
  uint8_t val;

  /* Read value from GCONF2 register */
  if (APDS9960_ReadRegister(dev, APDS9960_GCONF2, &val) != HAL_OK) {
    return HAL_ERROR;
  }

  /* Set bits in register to given value */
  gain &= 0b00000011;
  gain = gain << 5;
  val &= 0b10011111;
  val |= gain;

  /* Write register value back into GCONF2 register */
  if (APDS9960_WriteRegister(dev, APDS9960_GCONF2, val) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief Gets the time in low power mode between gesture detections
 *
 * Value    Wait time
 *   0          0 ms
 *   1          2.8 ms
 *   2          5.6 ms
 *   3          8.4 ms
 *   4         14.0 ms
 *   5         22.4 ms
 *   6         30.8 ms
 *   7         39.2 ms
 *
 * @param[in] dev Pointer to APDS9960 device structure
 * @return the current wait time between gestures. 0xFF on error.
 */
uint8_t APDS9960_GetGestureWaitTime(APDS9960_t *dev)
{
  uint8_t val;

  /* Read value from GCONF2 register */
  if (APDS9960_ReadRegister(dev, APDS9960_GCONF2, &val) != HAL_OK) {
    return APDS9960_ERROR;
  }

  /* Mask out GWTIME bits */
  val &= 0b00000111;

  return val;
}

/**
 * @brief Sets the time in low power mode between gesture detections
 *
 * Value    Wait time
 *   0          0 ms
 *   1          2.8 ms
 *   2          5.6 ms
 *   3          8.4 ms
 *   4         14.0 ms
 *   5         22.4 ms
 *   6         30.8 ms
 *   7         39.2 ms
 *
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[in] time the value for the wait time
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_SetGestureWaitTime(APDS9960_t *dev, uint8_t time)
{
  uint8_t val;

  /* Read value from GCONF2 register */
  if (APDS9960_ReadRegister(dev, APDS9960_GCONF2, &val) != HAL_OK) {
    return HAL_ERROR;
  }

  /* Set bits in register to given value */
  time &= 0b00000111;
  val &= 0b11111000;
  val |= time;

  /* Write register value back into GCONF2 register */
  if (APDS9960_WriteRegister(dev, APDS9960_GCONF2, val) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief Gets the low threshold for ambient light interrupts
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_GetLightIntLowThreshold(APDS9960_t *dev, uint16_t *threshold)
{
  uint8_t val_byte;
  *threshold = 0;

  /* Read value from ambient light low threshold, low byte register */
  if (APDS9960_ReadRegister(dev, APDS9960_AILTL, &val_byte) != HAL_OK) {
    return HAL_ERROR;
  }
  *threshold = val_byte;

  /* Read value from ambient light low threshold, high byte register */
  if (APDS9960_ReadRegister(dev, APDS9960_AILTH, &val_byte) != HAL_OK) {
    return HAL_ERROR;
  }
  *threshold = *threshold + ((uint16_t) val_byte << 8);

  return HAL_OK;
}

/**
 * @brief Sets the low threshold for ambient light interrupts
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[in] threshold low threshold value for interrupt to trigger
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_SetLightIntLowThreshold(APDS9960_t *dev, uint16_t threshold)
{
  uint8_t val_low;
  uint8_t val_high;

  /* Break 16-bit threshold into 2 8-bit values */
  val_low = threshold & 0x00FF;
  val_high = (threshold & 0xFF00) >> 8;

  /* Write low byte */
  if (APDS9960_WriteRegister(dev, APDS9960_AILTL, val_low) != HAL_OK) {
    return HAL_ERROR;
  }

  /* Write high byte */
  if (APDS9960_WriteRegister(dev, APDS9960_AILTH, val_high) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief Gets the high threshold for ambient light interrupts
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_GetLightIntHighThreshold(APDS9960_t *dev, uint16_t *threshold)
{
  uint8_t val_byte;
  *threshold = 0;

  /* Read value from ambient light low threshold, low byte register */
  if (APDS9960_ReadRegister(dev, APDS9960_AIHTL, &val_byte) != HAL_OK) {
    return HAL_ERROR;
  }
  *threshold = val_byte;

  /* Read value from ambient light low threshold, high byte register */
  if (APDS9960_ReadRegister(dev, APDS9960_AIHTH, &val_byte) != HAL_OK) {
    return HAL_ERROR;
  }
  *threshold = *threshold + ((uint16_t) val_byte << 8);

  return HAL_OK;
}

/**
 * @brief Sets the high threshold for ambient light interrupts
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[in] threshold high threshold value for interrupt to trigger
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_SetLightIntHighThreshold(APDS9960_t *dev, uint16_t threshold)
{
  uint8_t val_low;
  uint8_t val_high;

  /* Break 16-bit threshold into 2 8-bit values */
  val_low = threshold & 0x00FF;
  val_high = (threshold & 0xFF00) >> 8;

  /* Write low byte */
  if (APDS9960_WriteRegister(dev, APDS9960_AIHTL, val_low) != HAL_OK) {
    return HAL_ERROR;
  }

  /* Write high byte */
  if (APDS9960_WriteRegister(dev, APDS9960_AIHTH, val_high) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief Gets the low threshold for proximity interrupts
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_GetProximityIntLowThreshold(APDS9960_t *dev, uint8_t *threshold)
{
  return APDS9960_ReadRegister(dev, APDS9960_PILT, threshold);
}

/**
 * @brief Sets the low threshold for proximity interrupts
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[in] threshold low threshold value for interrupt to trigger
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_SetProximityIntLowThreshold(APDS9960_t *dev, uint8_t threshold)
{
  return APDS9960_WriteRegister(dev, APDS9960_PILT, threshold);
}

/**
 * @brief Gets the high threshold for proximity interrupts
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[out] threshold current low threshold stored on the APDS-9960
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_GetProximityIntHighThreshold(APDS9960_t *dev, uint8_t *threshold)
{
  return APDS9960_ReadRegister(dev, APDS9960_PIHT, threshold);
}

/**
 * @brief Sets the high threshold for proximity interrupts
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[in] threshold high threshold value for interrupt to trigger
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_SetProximityIntHighThreshold(APDS9960_t *dev, uint8_t threshold)
{
  return APDS9960_WriteRegister(dev, APDS9960_PIHT, threshold);
}

/**
 * @brief Gets if ambient light interrupts are enabled or not
 * @param[in] dev Pointer to APDS9960 device structure
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t APDS9960_GetAmbientLightIntEnable(APDS9960_t *dev)
{
  uint8_t val;

  /* Read value from ENABLE register */
  if (APDS9960_ReadRegister(dev, APDS9960_ENABLE, &val) != HAL_OK) {
    return APDS9960_ERROR;
  }

  /* Shift and mask out AIEN bit */
  val = (val >> 4) & 0b00000001;

  return val;
}

/**
 * @brief Turns ambient light interrupts on or off
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_SetAmbientLightIntEnable(APDS9960_t *dev, uint8_t enable)
{
  uint8_t val;

  /* Read value from ENABLE register */
  if (APDS9960_ReadRegister(dev, APDS9960_ENABLE, &val) != HAL_OK) {
    return HAL_ERROR;
  }

  /* Set bits in register to given value */
  enable &= 0b00000001;
  enable = enable << 4;
  val &= 0b11101111;
  val |= enable;

  /* Write register value back into ENABLE register */
  if (APDS9960_WriteRegister(dev, APDS9960_ENABLE, val) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief Gets if proximity interrupts are enabled or not
 * @param[in] dev Pointer to APDS9960 device structure
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t APDS9960_GetProximityIntEnable(APDS9960_t *dev)
{
  uint8_t val;

  /* Read value from ENABLE register */
  if (APDS9960_ReadRegister(dev, APDS9960_ENABLE, &val) != HAL_OK) {
    return APDS9960_ERROR;
  }

  /* Shift and mask out PIEN bit */
  val = (val >> 5) & 0b00000001;

  return val;
}

/**
 * @brief Turns proximity interrupts on or off
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_SetProximityIntEnable(APDS9960_t *dev, uint8_t enable)
{
  uint8_t val;

  /* Read value from ENABLE register */
  if (APDS9960_ReadRegister(dev, APDS9960_ENABLE, &val) != HAL_OK) {
    return HAL_ERROR;
  }

  /* Set bits in register to given value */
  enable &= 0b00000001;
  enable = enable << 5;
  val &= 0b11011111;
  val |= enable;

  /* Write register value back into ENABLE register */
  if (APDS9960_WriteRegister(dev, APDS9960_ENABLE, val) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief Gets if gesture interrupts are enabled or not
 * @param[in] dev Pointer to APDS9960 device structure
 * @return 1 if interrupts are enabled, 0 if not. 0xFF on error.
 */
uint8_t APDS9960_GetGestureIntEnable(APDS9960_t *dev)
{
  uint8_t val;

  /* Read value from GCONF4 register */
  if (APDS9960_ReadRegister(dev, APDS9960_GCONF4, &val) != HAL_OK) {
    return APDS9960_ERROR;
  }

  /* Shift and mask out GIEN bit */
  val = (val >> 1) & 0b00000001;

  return val;
}

/**
 * @brief Turns gesture-related interrupts on or off
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[in] enable 1 to enable interrupts, 0 to turn them off
 * @return True if operation successful. False otherwise.
 */
HAL_StatusTypeDef APDS9960_SetGestureIntEnable(APDS9960_t *dev, uint8_t enable)
{
  uint8_t val;

  /* Read value from GCONF4 register */
  if (APDS9960_ReadRegister(dev, APDS9960_GCONF4, &val) != HAL_OK) {
    return HAL_ERROR;
  }

  /* Set bits in register to given value */
  enable &= 0b00000001;
  enable = enable << 1;
  val &= 0b11111101;
  val |= enable;

  /* Write register value back into ENABLE register */
  if (APDS9960_WriteRegister(dev, APDS9960_GCONF4, val) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief Clears the ambient light interrupt
 * @param[in] dev Pointer to APDS9960 device structure
 * @return HAL_OK if operation completed successfully. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_ClearAmbientLightInt(APDS9960_t *dev)
{
  uint8_t throwaway;
  return APDS9960_ReadRegister(dev, APDS9960_AICLEAR, &throwaway);
}

/**
 * @brief Clears the proximity interrupt
 * @param[in] dev Pointer to APDS9960 device structure
 * @return HAL_OK if operation completed successfully. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_ClearProximityInt(APDS9960_t *dev)
{
  uint8_t throwaway;
  return APDS9960_ReadRegister(dev, APDS9960_PICLEAR, &throwaway);
}

/**
 * @brief Tells if the gesture state machine is currently running
 * @param[in] dev Pointer to APDS9960 device structure
 * @return 1 if gesture state machine is running, 0 if not. 0xFF on error.
 */
uint8_t APDS9960_GetGestureMode(APDS9960_t *dev)
{
  uint8_t val;

  /* Read value from GCONF4 register */
  if (APDS9960_ReadRegister(dev, APDS9960_GCONF4, &val) != HAL_OK) {
    return APDS9960_ERROR;
  }

  /* Mask out GMODE bit */
  val &= 0b00000001;

  return val;
}

/**
 * @brief Tells the state machine to either enter or exit gesture state machine
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[in] mode 1 to enter gesture state machine, 0 to exit.
 * @return HAL_OK if operation completed successfully. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_SetGestureMode(APDS9960_t *dev, uint8_t mode)
{
  uint8_t val;

  /* Read value from GCONF4 register */
  if (APDS9960_ReadRegister(dev, APDS9960_GCONF4, &val) != HAL_OK) {
    return HAL_ERROR;
  }

  /* Set bits in register to given value */
  mode &= 0b00000001;
  val &= 0b11111110;
  val |= mode;

  /* Write register value back into ENABLE register */
  if (APDS9960_WriteRegister(dev, APDS9960_GCONF4, val) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/*******************************************************************************
 * Raw I2C Reads and Writes
 ******************************************************************************/

/**
 * @brief Writes a single byte to the I2C device (no register)
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[in] val the 1-byte value to write to the I2C device
 * @return HAL_OK if successful write operation. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_WriteByte(APDS9960_t *dev, uint8_t val)
{
  return HAL_I2C_Master_Transmit(dev->hi2c, (APDS9960_I2C_ADDR << 1), &val, 1,
  HAL_MAX_DELAY);
}

/**
 * @brief Writes a single byte to the I2C device and specified register
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[in] reg the register in the I2C device to write to
 * @param[in] val the 1-byte value to write to the I2C device
 * @return HAL_OK if successful write operation. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_WriteRegister(APDS9960_t *dev, uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(dev->hi2c, (APDS9960_I2C_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}

/**
 * @brief Reads a single byte from the I2C device and specified register
 * @param[in] dev Pointer to APDS9960 device structure
 * @param[in] reg the register to read from
 * @param[out] the value returned from the register
 * @return HAL_OK if successful read operation. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef APDS9960_ReadRegister(APDS9960_t *dev, uint8_t reg, uint8_t *data)
{
  return HAL_I2C_Mem_Read(dev->hi2c, (APDS9960_I2C_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}
