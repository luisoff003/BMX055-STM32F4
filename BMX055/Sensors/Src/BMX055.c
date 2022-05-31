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
const uint8_t Gscale = BMX055_GYRO_RANGE_2000DPS;
const uint8_t Ascale = BMX055_ACC_RANGE_4G;
const uint8_t Mscale = 0;

//Resolution each sensor
float f_accRes = 1.0;
float f_magRes = 1.0;
float f_gyroRes = 1.0;

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
	writeData = BMX055_MAG_SLEEP_MODE;
	HAL_I2C_Mem_Write(I2Cx, BMX055_MAG_SLAVE_ADDRESS_DEFAULT<<1, BMX055_MAG_POW_CTL_REG, 1, &writeData, 1, 500);
	/* Read MAG BMX055 WHOAMI */
	HAL_I2C_Mem_Read(I2Cx, BMX055_MAG_SLAVE_ADDRESS_DEFAULT<<1, BMX055_WHO_AM_I_MAG_REG, 1, &readData, 1, 500);
	if( readData != BMX055_MAG_DEVICE ){
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
	writeData = Gscale;
	HAL_I2C_Mem_Write(I2Cx, BMX055_GYRO_SLAVE_ADDRESS_DEFAULT<<1, BMX055_GYRO_RANGE_REG, 1, &writeData, 1, 500);

	/* Select Gyro BandWidth */
	writeData = BMX055_GYRO_BW_32;
	HAL_I2C_Mem_Write(I2Cx, BMX055_GYRO_SLAVE_ADDRESS_DEFAULT<<1, BMX055_GYRO_BW_REG, 1, &writeData, 1, 500);

	/* Select Gyro LPM (NormalMode, SleepDuration 2ms) */
	writeData = BMX055_GYRO_LPM1_MODE_NOMAL|BMX055_GYRO_LPM1_SLEEP_DUR_4MS;
	HAL_I2C_Mem_Write(I2Cx, BMX055_GYRO_SLAVE_ADDRESS_DEFAULT<<1, BMX055_GYRO_LPM1_REG, 1, &writeData, 1, 500);


	/* Magnetometer Soft Reset */
//	writeData = BMX055_MAG_POW_CTL_SOFT_RESET;
//	HAL_I2C_Mem_Write(I2Cx, BMX055_MAG_SLAVE_ADDRESS_DEFAULT<<1, BMX055_MAG_POW_CTL_REG, 1, &writeData, 1, 500);
//	HAL_Delay(2);	/* Wait 2ms */
//
//	/* WK UP MAGN */
//	writeData = BMX055_MAG_SLEEP_MODE;
//	HAL_I2C_Mem_Write(I2Cx, BMX055_MAG_SLAVE_ADDRESS_DEFAULT<<1, BMX055_MAG_POW_CTL_REG, 1, &writeData, 1, 500);

	/* Normal Mode , ODR 10Hz*/
	writeData = BMX055_MAG_OP_MODE_NORMAL | BMX055_MAG_DATA_RATE_10;
	HAL_I2C_Mem_Write(I2Cx, BMX055_MAG_SLAVE_ADDRESS_DEFAULT<<1, BMX055_MAG_ADV_OP_OUTPUT_REG, 1, &writeData, 1, 500);

	/* Repetitions for X-Y Axis 0x04 -> 0b0100 -> 1+2(2^2) = 9 */
	writeData = 0x04;
	HAL_I2C_Mem_Write(I2Cx, BMX055_MAG_SLAVE_ADDRESS_DEFAULT<<1, BMX055_MAG_REP_XY_REG, 1, &writeData, 1, 500);

	/* Repetitions for Z Axis 0x0F -> 0b1111 -> 1+(2^3 + 2^2 + 2^1 + 2^0) = 15 */
	writeData = 0x0E;
	HAL_I2C_Mem_Write(I2Cx, BMX055_MAG_SLAVE_ADDRESS_DEFAULT<<1, BMX055_MAG_REP_Z_REG, 1, &writeData, 1, 500);


	/* Self test and reporting */

	//Calibration


	//Init Accelerometer

	/* Read Actual Accelerometer Resolution */
	getAcc_Res();

	/* Read Actual Gyroscope Resolution */
	getGyro_Res();


	return 0;
}

void readTemp_BMX055(float *destination, I2C_HandleTypeDef *I2Cx){
	uint16_t rawData;
	HAL_I2C_Mem_Read(I2Cx, BMX055_ACC_SLAVE_ADDRESS_DEFAULT<<1, BMX055_TEMP_SENSOR, 1, &rawData, 1, 500);
	rawData = ((int16_t)((int16_t)rawData << 8)) >> 8 ;
	*destination = ( (float) rawData )/2.0 + 23.0;
}

void readAccelData(int16_t *destination, I2C_HandleTypeDef *I2Cx){
	/* XYZ Data Register Stored Here */
	uint8_t rawData[6];

	HAL_I2C_Mem_Read(I2Cx, BMX055_ACC_SLAVE_ADDRESS_DEFAULT<<1, BMX055_ACC_DATA_START_REG, 1, rawData, 6, 500);

//	if((rawData[0] & 0x01) && (rawData[2] & 0x01) && (rawData[4] & 0x01)) {  // Check that all 3 axes have new data
		destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]) >> 4;  // Turn the MSB and LSB into a signed 12-bit value
		destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]) >> 4;
		destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]) >> 4;
//	}

}

void readGyroData(int16_t *destination, I2C_HandleTypeDef *I2Cx){
	uint8_t rawData[6];
	HAL_I2C_Mem_Read(I2Cx, BMX055_GYRO_SLAVE_ADDRESS_DEFAULT<<1, BMX055_GYRO_DATA_START_REG, 1, rawData, 6, 500);

	destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);
	destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]);

}

void readMagData(int16_t *destination, I2C_HandleTypeDef *I2Cx){
	int16_t mdata_x = 0, mdata_y = 0, mdata_z = 0, temp = 0;
	uint16_t data_r = 0;
	uint8_t rawData[6];
	HAL_I2C_Mem_Read(I2Cx, BMX055_MAG_SLAVE_ADDRESS_DEFAULT<<1, BMX055_MAG_DATA_START_REG, 1, rawData, 8, 2000);  // Read the eight raw data registers sequentially into data array

//	if(rawData[6] & 0x01) { // Check if data ready status bit is set
		destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]) >> 3;  // 13-bit signed integer for x-axis field
		destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]) >> 3;  // 13-bit signed integer for y-axis field
		destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]) >> 1;  // 15-bit signed integer for z-axis field
		data_r = (uint16_t) (((uint16_t)rawData[7] << 8) | rawData[6]) >> 2;  // 14-bit unsigned integer for Hall resistance

	// calculate temperature compensated 16-bit magnetic fields
//	temp = ((int16_t)(((uint16_t)((((int32_t)dig_xyz1) << 14)/(data_r != 0 ? data_r : dig_xyz1))) - ((uint16_t)0x4000)));
//	destination[2] = temp;
//	magData[0] = ((int16_t)((((int32_t)mdata_x) *
//				((((((((int32_t)dig_xy2) * ((((int32_t)temp) * ((int32_t)temp)) >> 7)) +
//				 (((int32_t)temp) * ((int32_t)(((int16_t)dig_xy1) << 7)))) >> 9) +
//			   ((int32_t)0x100000)) * ((int32_t)(((int16_t)dig_x2) + ((int16_t)0xA0)))) >> 12)) >> 13)) +
//			(((int16_t)dig_x1) << 3);
//
//	temp = ((int16_t)(((uint16_t)((((int32_t)dig_xyz1) << 14)/(data_r != 0 ? data_r : dig_xyz1))) - ((uint16_t)0x4000)));
//	magData[1] = ((int16_t)((((int32_t)mdata_y) *
//				((((((((int32_t)dig_xy2) * ((((int32_t)temp) * ((int32_t)temp)) >> 7)) +
//				 (((int32_t)temp) * ((int32_t)(((int16_t)dig_xy1) << 7)))) >> 9) +
//				   ((int32_t)0x100000)) * ((int32_t)(((int16_t)dig_y2) + ((int16_t)0xA0)))) >> 12)) >> 13)) +
//			(((int16_t)dig_y1) << 3);
//	magData[2] = (((((int32_t)(mdata_z - dig_z4)) << 15) - ((((int32_t)dig_z3) * ((int32_t)(((int16_t)data_r) -
//	((int16_t)dig_xyz1))))>>2))/(dig_z2 + ((int16_t)(((((int32_t)dig_z1) * ((((int16_t)data_r) << 1)))+(1<<15))>>16))));

//	}

}


uint8_t SetScaleAcce(uint8_t AccScale){

}

void getMag_Res(uint16_t Mscale){
	switch (Mscale)
	  {
//	  // Possible magnetometer scales (and their register bit settings) are:
//	  // 14 bit resolution (0) and 16 bit resolution (1)
//	    case MFS_14BITS:
//	          mRes = 10.*4912./8190.; // Proper scale to return milliGauss
//	          break;
//	    case MFS_16BITS:
//	          mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
//	          break;
	  }
}

void getAcc_Res(void){
	  switch (Ascale)
	  {
	 	// Possible accelerometer scales (and their register bit settings) are:
		// 2 Gs (0011), 4 Gs (0101), 8 Gs (1000), and 16 Gs  (1100).
	        // BMX055 ACC data is signed 12 bit
	    case BMX055_ACC_RANGE_2G:
	          f_accRes = 2.0/2048.0;
	          break;
	    case BMX055_ACC_RANGE_4G:
	    	f_accRes = 4.0/2048.0;
	          break;
	    case BMX055_ACC_RANGE_8G:
	    	f_accRes = 8.0/2048.0;
	          break;
	    case BMX055_ACC_RANGE_16G:
	    	f_accRes = 16.0/2048.0;
	          break;
	  }
}

void getGyro_Res(void){

	switch (Gscale)
		  {
		 	// Possible accelerometer scales (and their register bit settings) are:
			// 2 Gs (0011), 4 Gs (0101), 8 Gs (1000), and 16 Gs  (1100).
		        // BMX055 ACC data is signed 12 bit
		    case BMX055_GYRO_RANGE_2000DPS:
		    	f_gyroRes =1998.0/32768.0;
		          break;
		    case BMX055_GYRO_RANGE_1000DPS:
		    	f_gyroRes = 999.0/32768.0;
		    	break;
		    case BMX055_GYRO_RANGE_500DPS:
		    	f_gyroRes = 499.5/32768.0;
		    	break;
		    case BMX055_GYRO_RANGE_250DPS:
		    	f_gyroRes = 249.75/32768.0;
		    	break;
		    case BMX055_GYRO_RANGE_125DPS:
		    	f_gyroRes = 124.87/32768.0;
		    	break;
		  }

}

void BMX055_readAllSensors(I2C_HandleTypeDef *I2Cx, BMX055_t *DataStruct){
	int16_t accelCount[3];
	int16_t gyroCount[3];

	/* Read (x,y,z) accelerometer values */
	readAccelData(accelCount, I2Cx);

	/* Calculate actual values in G's */
	DataStruct->AccelX = (float)((int)accelCount[0]*f_accRes);
	DataStruct->AccelY = (float)((int)accelCount[1]*f_accRes);
	DataStruct->AccelZ = (float)((int)accelCount[2]*f_accRes);

	/* Read (x,y,z) gyroscope values */
	readGyroData(gyroCount, I2Cx);

	/* Calculate actual values in °/s */
	DataStruct->GyroX = (float)((int)gyroCount[0]*f_gyroRes);
	DataStruct->GyroY = (float)((int)gyroCount[1]*f_gyroRes);
	DataStruct->GyroZ = (float)((int)gyroCount[2]*f_gyroRes);

	/* Read (x,y,z) magnetometer */


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
