/*
 * mpr121.c
 *
 *  Created on: Mar 14, 2025
 *      Author: kf1375
 */

#include "mpr121.h"

static HAL_StatusTypeDef MPR121_WriteRegister(MPR121_t *dev, uint8_t reg, uint8_t value);
static HAL_StatusTypeDef MPR121_ReadRegister8(MPR121_t *dev, uint8_t reg, uint8_t *data);
static HAL_StatusTypeDef MPR121_ReadRegister16(MPR121_t *dev, uint8_t reg, uint16_t *data);

/*******************************************************************************
 * Public methods for controlling the APDS-9960
 ******************************************************************************/

/**
 * @brief Configures I2C communications and initializes registers to defaults
 * @param dev Pointer to MPR121 device structure
 * @return HAL_OK if initialized successfully. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef MPR121_Init(MPR121_t *dev, I2C_HandleTypeDef *hi2c, uint8_t touchThreshold, uint8_t releaseThreshold)
{
  dev->hi2c = hi2c;
  // soft reset
  if (MPR121_WriteRegister(dev, MPR121_SOFTRESET, 0x63) != HAL_OK) {
    return HAL_ERROR;
  }
  HAL_Delay(1);
  if (MPR121_WriteRegister(dev, MPR121_ECR, 0x0) != HAL_OK) {
    return HAL_ERROR;
  }

  uint8_t config2;
  if (MPR121_ReadRegister8(dev, MPR121_CONFIG2, &config2) != HAL_OK) {
    return HAL_ERROR;
  }
  if (config2 != 0x24) {
    return HAL_ERROR;
  }

  if (MPR121_SetThresholds(dev, touchThreshold, releaseThreshold) != HAL_OK) {
    return HAL_ERROR;
  }

  if (MPR121_WriteRegister(dev, MPR121_MHDR, 0x01) != HAL_OK) {
    return HAL_ERROR;
  }
  if (MPR121_WriteRegister(dev, MPR121_NHDR, 0x01) != HAL_OK) {
    return HAL_ERROR;
  }
  if (MPR121_WriteRegister(dev, MPR121_NCLR, 0x0E) != HAL_OK) {
    return HAL_ERROR;
  }
  if (MPR121_WriteRegister(dev, MPR121_FDLR, 0x00) != HAL_OK) {
    return HAL_ERROR;
  }
  if (MPR121_WriteRegister(dev, MPR121_MHDF, 0x01) != HAL_OK) {
    return HAL_ERROR;
  }
  if (MPR121_WriteRegister(dev, MPR121_NHDF, 0x05) != HAL_OK) {
    return HAL_ERROR;
  }
  if (MPR121_WriteRegister(dev, MPR121_NCLF, 0x01) != HAL_OK) {
    return HAL_ERROR;
  }
  if (MPR121_WriteRegister(dev, MPR121_FDLF, 0x00) != HAL_OK) {
    return HAL_ERROR;
  }
  if (MPR121_WriteRegister(dev, MPR121_NHDT, 0x00) != HAL_OK) {
    return HAL_ERROR;
  }
  if (MPR121_WriteRegister(dev, MPR121_NCLT, 0x00) != HAL_OK) {
    return HAL_ERROR;
  }
  if (MPR121_WriteRegister(dev, MPR121_FDLT, 0x00) != HAL_OK) {
    return HAL_ERROR;
  }
  if (MPR121_WriteRegister(dev, MPR121_DEBOUNCE, 0) != HAL_OK) {
    return HAL_ERROR;
  }
  if (MPR121_WriteRegister(dev, MPR121_CONFIG1, 0x10) != HAL_OK) { // default, 16uA charge current
    return HAL_ERROR;
  }
  if (MPR121_WriteRegister(dev, MPR121_CONFIG2, 0x20) != HAL_OK) { // 0.5uS encoding, 1ms period
    return HAL_ERROR;
  }

  uint8_t ecr_setting = 0b10000000 + 12; // 5 bits for baseline tracking & proximity disabled + X amount of electrodes running (12)
  if (MPR121_WriteRegister(dev, MPR121_ECR, ecr_setting) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
 * @brief Reads Set the touch and release thresholds for all 13 channels
 * @param[in] dev Pointer to MPR121 device structure
 * @param[in] touch Touch threshold
 * @param[in] release Release threshold
 * @return HAL_OK if operation successful. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef MPR121_SetThresholds(MPR121_t *dev, uint8_t touch, uint8_t release)
{
  // set all thresholds (the same)
  for (uint8_t i = 0; i < 12; i++) {
    if (MPR121_WriteRegister(dev, MPR121_TOUCHTH_0 + 2 * i, touch) != HAL_OK) {
      return HAL_ERROR;
    }
    if (MPR121_WriteRegister(dev, MPR121_RELEASETH_0 + 2 * i, release) != HAL_OK) {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}

/**
 * @brief Read the filtered data from channel
 * @param[in] dev Pointer to MPR121 device structure
 * @param[in] channel The channel to read
 * @return The filtered reading as a 10 bit unsigned value
 */
uint16_t MPR121_FilteredData(MPR121_t *dev, uint8_t channel)
{
  if (channel > 12) {
    return 0;
  }

  uint16_t filtered_data;
  if (MPR121_ReadRegister16(dev, MPR121_FILTDATA_0L + channel * 2, &filtered_data) != HAL_OK) {
    return 0;
  }

  return filtered_data;
}

/**
 * @brief Read the baseline value for the channel.
 * @param[in] dev Pointer to MPR121 device structure
 * @param[in] channel The channel to read
 * @return The baseline data that was read
 */
uint16_t MPR121_BaseLineData(MPR121_t *dev, uint8_t channel)
{
  if (channel > 12) {
    return 0;
  }

  uint16_t baseline_data;
  if (MPR121_ReadRegister8(dev, MPR121_BASELINE_0 + channel, (uint8_t *) &baseline_data) != HAL_OK) {
    return 0;
  }

  return (baseline_data << 2);
}

/**
 * @brief Read the touch status of all 13 channels as bit values in a 12 bit integer.
 * @param[in] dev Pointer to MPR121 device structure
 * @returns A 12 bit integer with each bit corresponding to the touch status
 *          of a sensor. For example, if bit 0 is set then channel 0 of the
 *          device is currently deemed to be touched.
 */
uint16_t MPR121_Touched(MPR121_t *dev)
{
  uint16_t touched;
  if (MPR121_ReadRegister16(dev, MPR121_TOUCHSTATUS_L, &touched) != HAL_OK) {
    return 0;
  }

  return (touched & 0x0FFF);
}

/*******************************************************************************
 * Raw I2C Reads and Writes
 ******************************************************************************/

/**
 * @brief Writes a single byte to the I2C device and specified register
 * @param[in] dev Pointer to MPR121 device structure
 * @param[in] reg the register in the I2C device to write to
 * @param[in] val the 1-byte value to write to the I2C device
 * @return HAL_OK if successful write operation. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef MPR121_WriteRegister(MPR121_t *dev, uint8_t reg, uint8_t value)
{
  // MPR121 must be put in Stop Mode to write to most registers
  uint8_t stop_required = 1;

  // First get the current set value of the MPR121_ECR register
  uint8_t ecr_reg_backup;
  if (MPR121_ReadRegister8(dev, MPR121_ECR, &ecr_reg_backup) != HAL_OK) {
    return HAL_ERROR;
  }
  if ((reg == MPR121_ECR) || ((0x73 <= reg) && (reg <= 0x7A))) {
    stop_required = 0;
  }
  if (stop_required) {
    // clear this register to set stop mode
    uint8_t stop_command = 0x00;
    if (HAL_I2C_Mem_Write(dev->hi2c, (MPR121_ADDR << 1), MPR121_ECR, I2C_MEMADD_SIZE_8BIT, &stop_command, 1, 1000) != HAL_OK) {
      return HAL_ERROR;
    }
  }

  if (HAL_I2C_Mem_Write(dev->hi2c, (MPR121_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 1000) != HAL_OK) {
    return HAL_ERROR;
  }

  // Write back the previous ECR settings
  if (stop_required) {
    if (HAL_I2C_Mem_Write(dev->hi2c, (MPR121_ADDR << 1), MPR121_ECR, I2C_MEMADD_SIZE_8BIT, &ecr_reg_backup, 1, 1000) != HAL_OK) {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}

/**
 * @brief Reads a single byte from the I2C device and specified register
 * @param[in] dev Pointer to MPR121 device structure
 * @param[in] reg the register to read from
 * @param[out] data the value returned from the register
 * @return HAL_OK if successful read operation. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef MPR121_ReadRegister8(MPR121_t *dev, uint8_t reg, uint8_t *data)
{
  return HAL_I2C_Mem_Read(dev->hi2c, (MPR121_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

/**
 * @brief Reads 2 bytes from the I2C device and specified register
 * @param[in] dev Pointer to MPR121 device structure
 * @param[in] reg the register to read from
 * @param[out] data the value returned from the register
 * @return HAL_OK if successful read operation. HAL_ERROR otherwise.
 */
HAL_StatusTypeDef MPR121_ReadRegister16(MPR121_t *dev, uint8_t reg, uint16_t *data)
{
  uint8_t buffer[2];

  if (HAL_I2C_Mem_Read(dev->hi2c, (MPR121_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, buffer, 2, HAL_MAX_DELAY) != HAL_OK) {
    return HAL_ERROR;
  }
  *data = (buffer[1] << 8) | buffer[0];

  return HAL_OK;
}
