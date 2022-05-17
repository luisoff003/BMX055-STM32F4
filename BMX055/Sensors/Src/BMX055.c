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

//Specify Sensor Scale
uint8_t Gscale = 0;
uint8_t Ascale = BMX055_ACC_RANGE_4G;
uint8_t Mscale = 0;



uint8_t BMX055_Init(I2C_HandleTypeDef *I2Cx){
	//pre-def. vars
	uint8_t readData;
	uint8_t writeData;

	/* Read ACC BMX055 WHOAMI */
	HAL_I2C_Mem_Read(I2Cx, BMX055_ACC_SLAVE_ADDRESS_DEFAULT<<1, BMX055_WHO_AM_I_REG, 1, &readData, 1, 500);

	if(readData != BMX055_ACC_DEVICE){
		return 0xFF;
	}

	/* Wakeup Magnetometer */
	writeData = BMX055_MAG_WK_UP;
	HAL_I2C_Mem_Write(I2Cx, BMX055_MAG_SLAVE_ADDRESS_DEFAULT<<1, BMX055_MAG_POW_CTL_REG, 1, &writeData, 1, 500);
	/* Read MAG BMX055 WHOAMI */
	HAL_I2C_Mem_Read(I2Cx, BMX055_MAG_SLAVE_ADDRESS_DEFAULT<<1, BMX055_WHO_AM_I_MAG_REG, 1, &readData, 1, 500);
	if( readData != BMX055_MAG_DEVICE && readData == 0x00){
		/* Sleep mode */
		return 0xFE;
	}

	/* Read GYRO BMX055 WHOAMI */
	HAL_I2C_Mem_Read(I2Cx, BMX055_GYRO_SLAVE_ADDRESS_DEFAULT<<1, BMX055_WHO_AM_I_REG, 1, &readData, 1, 500);
	if( readData != BMX055_GYRO_DEVICE ){
		return 0xFD;
	}

	/* Accelerometer Soft Reset */
	writeData = BMX055_INITIATED_SOFT_RESET;
	HAL_I2C_Mem_Write(I2Cx, BMX055_ACC_SLAVE_ADDRESS_DEFAULT<<1, BMX055_RESET_REG, 1, &writeData, 1, 500);
	HAL_Delay(2);	/* Wait 2ms */

	/* Configure Acceleration range */
	writeData = Ascale;
	HAL_I2C_Mem_Write(I2Cx, BMX055_ACC_SLAVE_ADDRESS_DEFAULT<<1, BMX055_ACC_PMU_RANGE_REG, 1, &writeData, 1, 500);

	/* Select Accel BandWidth */
	writeData = BMX055_ACC_PMU_BW_250;
	HAL_I2C_Mem_Write(I2Cx, BMX055_ACC_SLAVE_ADDRESS_DEFAULT<<1, BMX055_ACC_PMU_BW_REG, 1, &writeData, 1, 500);


	/* LP Mode Sleep mode */
	writeData = BMX055_ACC_PMU_LPW_MODE_NOMAL|BMX055_ACC_PMU_LPW_SLEEP_DUR_2MS;
	HAL_I2C_Mem_Write(I2Cx, BMX055_ACC_SLAVE_ADDRESS_DEFAULT<<1, BMX055_ACC_PMU_LPW_REG, 1, &writeData, 1, 500);

	/* Gyroscope Soft Reset  */
	writeData = BMX055_INITIATED_SOFT_RESET;
	HAL_I2C_Mem_Write(I2Cx, BMX055_GYRO_SLAVE_ADDRESS_DEFAULT<<1, BMX055_RESET_REG, 1, &writeData, 1, 500);
	HAL_Delay(2);	/* Wait 2ms */

	/* Select Gyro Range 262.4 LSB/°/s */
	writeData = BMX055_GYRO_RANGE_262_4;
	HAL_I2C_Mem_Write(I2Cx, BMX055_GYRO_SLAVE_ADDRESS_DEFAULT<<1, BMX055_GYRO_RANGE_REG, 1, &writeData, 1, 500);

	/* Select Gyro BandWidth */
	writeData = BMX055_GYRO_BW_230;
	HAL_I2C_Mem_Write(I2Cx, BMX055_GYRO_SLAVE_ADDRESS_DEFAULT<<1, BMX055_GYRO_BW_REG, 1, &writeData, 1, 500);

	/* Select Gyro LPM (NormalMode, SleepDuration 2ms) */
	writeData = BMX055_GYRO_LPM1_MODE_NOMAL|BMX055_GYRO_LPM1_SLEEP_DUR_2MS;
	HAL_I2C_Mem_Write(I2Cx, BMX055_GYRO_SLAVE_ADDRESS_DEFAULT<<1, BMX055_GYRO_LPM1_REG, 1, &writeData, 1, 500);


	/* Magnetometer Soft Reset */
	writeData = BMX055_MAG_POW_CTL_SOFT_RESET;
	HAL_I2C_Mem_Write(I2Cx, BMX055_MAG_SLAVE_ADDRESS_DEFAULT<<1, BMX055_MAG_POW_CTL_REG, 1, &writeData, 1, 500);
	HAL_Delay(2);	/* Wait 2ms */

	/* Magnetometer Rate */
	writeData = BMX055_MAG_DATA_RATE_30;
	HAL_I2C_Mem_Write(I2Cx, BMX055_MAG_SLAVE_ADDRESS_DEFAULT<<1, BMX055_MAG_ADV_OP_OUTPUT_REG, 1, &writeData, 1, 500);

	/* Repetitions for X-Y Axis 0x04 -> 0b0100 -> 1+2(2^2) = 9 */
	writeData = 0x04;
	HAL_I2C_Mem_Write(I2Cx, BMX055_MAG_SLAVE_ADDRESS_DEFAULT<<1, BMX055_MAG_REP_XY_REG, 1, &writeData, 1, 500);

	/* Repetitions for Z Axis 0x0F -> 0b1111 -> 1+2(2^3 + 2^2 + 2^1 + 2^0) = 15 */
	writeData = 0x0F;
	HAL_I2C_Mem_Write(I2Cx, BMX055_MAG_SLAVE_ADDRESS_DEFAULT<<1, BMX055_MAG_REP_Z_REG, 1, &writeData, 1, 500);

	/* Self test and reporting */




	//Calibration


	//Init Accelerometer


	return 0;
}

void readAccelData(int16_t *destination, I2C_HandleTypeDef *I2Cx){
	/* XYZ Data Register Stored Here */
	uint8_t rawData[6];

	HAL_I2C_Mem_Read(I2Cx, BMX055_ACC_SLAVE_ADDRESS_DEFAULT<<1, BMX055_ACC_DATA_START_REG, 1, rawData, 6, 500);

	if((rawData[0] & 0x01) && (rawData[2] & 0x01) && (rawData[4] & 0x01)) {  // Check that all 3 axes have new data
	  destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]) >> 4;  // Turn the MSB and LSB into a signed 12-bit value
	  destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]) >> 4;
	  destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]) >> 4;
	  }

}


uint8_t SetScaleAcce(uint8_t AccScale){


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
