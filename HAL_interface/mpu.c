/*
 * mpu.c
 *
 *  Created on: Jun 9, 2025
 *      Author: rauna
 */

#include "main.h"
#include "mpu.h"

extern I2C_HandleTypeDef hi2c1;

void mpu_init(void){
	 //wake up sensor
  uint8_t wake = 0;
  HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x6B, 1, &wake, 1, 100);  // Wake up MPU6050
  HAL_Delay(10);



  //check presence of device
  HAL_StatusTypeDef ret1 = HAL_I2C_IsDeviceReady(&hi2c1, 0x68 << 1, 1, 100);
  if (ret1 == HAL_OK) {
      CDC_Transmit_FS((uint8_t *)"device is ready\n", strlen("device is ready\n"));
  } else {
      CDC_Transmit_FS((uint8_t *)"device is not ready\n", strlen("device is not ready\n"));
  }


  //set scale of gyroscope
  uint8_t gyrodegree = 0b00001000;
  HAL_StatusTypeDef ret2 = HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x1B, 1, &gyrodegree, 1, 100); // 0x1B = GYRO_CONFIG
  if (ret2 == HAL_OK) {
      CDC_Transmit_FS((uint8_t *)"register 27 is written\n", strlen("register 27 is written\n"));
  } else {
      CDC_Transmit_FS((uint8_t *)"register 27 is not written\n", strlen("register 27 is not written\n"));
  }

  //set scale of accelerometer
  uint8_t accelrange = 0b00010000;
  HAL_StatusTypeDef ret3 = HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x1C, 1, &accelrange, 1, 100); // 0x1B = GYRO_CONFIG
    if (ret3 == HAL_OK) {
        CDC_Transmit_FS((uint8_t *)"register 28 is written\n", strlen("register 28 is written\n"));
    } else {
        CDC_Transmit_FS((uint8_t *)"register 28 is not written\n", strlen("register 28 is not written\n"));
    }
}
void MPU_ReadAccelGyro(int16_t* ax, int16_t* ay, int16_t* az,
                       int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t rawData[14];
    HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x3B, 1, rawData, 14, 1000);

    *ax = (int16_t)(rawData[0] << 8 | rawData[1]);
    *ay = (int16_t)(rawData[2] << 8 | rawData[3]);
    *az = (int16_t)(rawData[4] << 8 | rawData[5]);
    *gx = (int16_t)(rawData[8] << 8 | rawData[9]);
    *gy = (int16_t)(rawData[10] << 8 | rawData[11]);
    *gz = (int16_t)(rawData[12] << 8 | rawData[13]);
}