/*
 * BMX055.c
 *
 *  Created on: May 15, 2022
 *      Author: luis0
 */


#include "BMX055.h"


const uint16_t i2c_timeout = 500;

float BMX055_pitch = 0.0;
float BMX055_roll = 0.0;
float BMX055_yaw = 0.0;


//Calibration Parameters
float magCalibration[3] = {0.0, 0.0, 0.0};
float gyroBias[3] = {0.0, 0.0, 0.0};
float accelBias[3] = {0.0, 0.0, 0.0};



uint8_t BMX055_Init(I2C_HandleTypeDef *I2Cx){
	//pre-def. vars
	uint8_t readData;
	uint8_t writeData;

	//Read BMX055 WHOAMI
	HAL_I2C_Mem_Read(I2Cx, BMX055_ACC_SLAVE_ADDRESS_DEFAULT<<1, BMX055_WHO_AM_I_REG, 1, &readData, 1, 500);

	if (readData == BMX055_ACC_DEVICE){
		//Self test and reporting

		//Calibration


		//Init Accelerometer

	}

	return readData;
}


uint8_t SearchDevice(I2C_HandleTypeDef *I2Cx){
	uint8_t ret = 0;
	uint8_t size = 0;
	uint8_t buffer[50];
	/* Search for devices from 0x00 to 128 */
	for(int i=0; i<128; i++){
	  ret = HAL_I2C_IsDeviceReady(I2Cx, (uint16_t)(i<<1), 3, 500);
	  if (ret != HAL_OK) /* No ACK Received At That Address */
	  {
		  size = sprintf(buffer, "- ", i);
		  CDC_Transmit_FS(buffer, size);
	  }
	  else if(ret == HAL_OK)
	  {
		  size = sprintf(buffer, "0x%X", i);
		  CDC_Transmit_FS(buffer, size);
	  }

	  HAL_Delay(100);
	}

}
