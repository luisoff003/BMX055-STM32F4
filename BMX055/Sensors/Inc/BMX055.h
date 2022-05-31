/*
 * BMX055.h
 *
 *  Created on: May 15, 2022
 *      Author: luison003
 */

#ifndef BMX055_H_
#define BMX055_H_

#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "stdlib.h"

/// @name BMX055 Register Address
/// @note Depends on the wiring of SDO1 SDO2 and CSB3
/// @{
#define BMX055_ACC_SLAVE_ADDRESS_DEFAULT 0x18
#define BMX055_GYRO_SLAVE_ADDRESS_DEFAULT 0x68
#define BMX055_MAG_SLAVE_ADDRESS_DEFAULT 0x10
/// @}

/** Who_am_i register ACC & GYRO */
#define BMX055_WHO_AM_I_REG 0x00
/** Who am I register MAG*/
#define BMX055_WHO_AM_I_MAG_REG 0x40
/** Magnetometer Sleep Mode */
#define BMX055_MAG_SLEEP_MODE 	0x01
/** Device check register ACC */
#define BMX055_ACC_DEVICE		0xFA
/** Device check register GYR */
#define BMX055_GYRO_DEVICE		0x0F
/** Device check register MAG */
#define BMX055_MAG_DEVICE 		0x32
/** Reset register */
#define BMX055_RESET_REG		0x14
/** Soft reset parameter */
#define BMX055_INITIATED_SOFT_RESET		0xB6

/// @name BMX055 Accel Register
/// @{
#define BMX055_ACC_DATA_START_REG			0x02
#define BMX055_ACC_PMU_RANGE_REG			0x0F
#define BMX055_ACC_PMU_BW_REG				0x10
#define BMX055_ACC_PMU_LPW_REG				0x11
#define BMX055_TEMP_SENSOR					0x08
/// @}

/// @name BMX055 Gyro Register
/// @{
#define BMX055_GYRO_DATA_START_REG			0x02
#define BMX055_GYRO_RANGE_REG				0x0F
#define BMX055_GYRO_BW_REG					0x10
#define BMX055_GYRO_LPM1_REG				0x11
/// @}

/// @name BMX055 Mag Register
/// @{
#define BMX055_MAG_DATA_START_REG			0x42
#define BMX055_MAG_POW_CTL_REG				0x4B
#define BMX055_MAG_ADV_OP_OUTPUT_REG	0x4C
#define BMX055_MAG_AXES_REG						0x4E
#define BMX055_MAG_REP_XY_REG         0x51
#define BMX055_MAG_REP_Z_REG          0x52
/// @}

/// @name Gyro Low Power Mode Parameter
/// @{
#define BMX055_GYRO_LPM1_MODE_NOMAL					0b00
#define BMX055_GYRO_LPM1_MODE_DEEP_SUSPEND	0b01
#define BMX055_GYRO_LPM1_MODE_SUSPEND				0b10
/// @}

/// @name Accel MPU Range Parameter
/// @{
#define BMX055_ACC_RANGE_2G		0x03
#define BMX055_ACC_RANGE_4G		0x05
#define BMX055_ACC_RANGE_8G		0x08
#define BMX055_ACC_RANGE_16G	0x0C
/// @}

/// @name Accel MPU Band Width Parameter(Hz)
/// @{
#define BMX055_ACC_PMU_BW_7_81		0b01000
#define BMX055_ACC_PMU_BW_15_63		0b01001
#define BMX055_ACC_PMU_BW_31_25		0b01010
#define BMX055_ACC_PMU_BW_62_5		0b01011
#define BMX055_ACC_PMU_BW_125		0b01100
#define BMX055_ACC_PMU_BW_250		0b01101
#define BMX055_ACC_PMU_BW_500		0b01110
#define BMX055_ACC_PMU_BW_1000	0b01111
/// @}

/// @name Accel Low Power Mode Parameter
/// @{
#define BMX055_ACC_PMU_LPW_MODE_NOMAL					0b00000000
#define BMX055_ACC_PMU_LPW_MODE_DEEP_SUSPEND	0b00100000
#define BMX055_ACC_PMU_LPW_MODE_LOW_POWER			0b01000000
#define BMX055_ACC_PMU_LPW_MODE_SUSPEND				0b10000000
/// @}

/// @name Accel Low Power Mode sleep phase duration Parameter
/// @{
#define BMX055_ACC_PMU_LPW_SLEEP_DUR_0_5MS	0b00000
#define BMX055_ACC_PMU_LPW_SLEEP_DUR_1MS		0b01100
#define BMX055_ACC_PMU_LPW_SLEEP_DUR_2MS		0b01110
#define BMX055_ACC_PMU_LPW_SLEEP_DUR_4MS		0b10000
#define BMX055_ACC_PMU_LPW_SLEEP_DUR_6MS		0b10010
#define BMX055_ACC_PMU_LPW_SLEEP_DUR_10MS		0b10100
#define BMX055_ACC_PMU_LPW_SLEEP_DUR_25MS		0b10110
#define BMX055_ACC_PMU_LPW_SLEEP_DUR_50MS		0b11000
#define BMX055_ACC_PMU_LPW_SLEEP_DUR_100MS	0b11010
#define BMX055_ACC_PMU_LPW_SLEEP_DUR_500MS	0b11100
#define BMX055_ACC_PMU_LPW_SLEEP_DUR_1S			0b11110

#define BMX055_ACC_PMU_SELF_TEST 0x32
#define BMX055_ACC_TRIM_NVM_CTRL 0x33
#define BMX055_ACC_BGW_SPI3_WDT  0x34
/// @}

/// @name Gyroscope Measurement Range
/// @{
/* Scale +-2000°/s Resolution 16.4 LSB/°/s <-> 61m°/s /LSB */
#define BMX055_GYRO_RANGE_2000DPS	0x00
/* Scale +-1000°/s Resolution 32.8 LSB/°/s <-> 30.5m°/s /LSB */
#define BMX055_GYRO_RANGE_1000DPS	0x01
/* Scale +-500°/s Resolution 65.6 LSB/°/s <-> 15.3m°/s /LSB */
#define BMX055_GYRO_RANGE_500DPS	0x02
/* Scale +-250°/s Resolution 131.2 LSB/°/s <-> 7.6m°/s /LSB */
#define BMX055_GYRO_RANGE_250DPS	0x03
/* Scale +-125°/s Resolution 262.4 LSB/°/s <-> 3.8m°/s /LSB */
#define BMX055_GYRO_RANGE_125DPS	0x04
/// @}

/// @name Gyroscope MPU Band Width Filter Parameter(Hz)
/// @{
#define BMX055_GYRO_BW_32		0b0111
#define BMX055_GYRO_BW_64		0b0110
#define BMX055_GYRO_BW_12		0b0101
#define BMX055_GYRO_BW_23		0b0100
#define BMX055_GYRO_BW_47		0b0011
#define BMX055_GYRO_BW_116	0b0010
#define BMX055_GYRO_BW_230	0b0001
#define BMX055_GYRO_BW_523	0b0000
/// @}

/// @name Gyro Sleep Duration Parameter(ms)
/// @{
#define BMX055_GYRO_LPM1_SLEEP_DUR_2MS		0b000
#define BMX055_GYRO_LPM1_SLEEP_DUR_4MS		0b001
#define BMX055_GYRO_LPM1_SLEEP_DUR_5MS		0b010
#define BMX055_GYRO_LPM1_SLEEP_DUR_8MS		0b011
#define BMX055_GYRO_LPM1_SLEEP_DUR_10MS		0b100
#define BMX055_GYRO_LPM1_SLEEP_DUR_15MS		0b101
#define BMX055_GYRO_LPM1_SLEEP_DUR_18MS		0b110
#define BMX055_GYRO_LPM1_SLEEP_DUR_20MS		0b111
/// @}

/// @name Mag Advance Self Test Control Parameter
/// @{
#define BMX055_MAG_ADV_SELF_TEST_NORMAL		0b00000000
#define BMX055_MAG_ADV_SELF_TEST_NEGATIVE	0b10000000
#define BMX055_MAG_ADV_SELF_TEST_POSITIVE	0b11000000
/// @}

/// @name Mag Datarate Control Parameter
/// @{
/* Magnetometer Output Data Rate 10Hz */
#define BMX055_MAG_DATA_RATE_10 0b000000
/* Magnetometer Output Data Rate 2Hz */
#define BMX055_MAG_DATA_RATE_2	0b001000
/* Magnetometer Output Data Rate 6Hz */
#define BMX055_MAG_DATA_RATE_6	0b010000
/* Magnetometer Output Data Rate 8Hz */
#define BMX055_MAG_DATA_RATE_8	0b011000
/* Magnetometer Output Data Rate 15Hz */
#define BMX055_MAG_DATA_RATE_15	0b100000
/* Magnetometer Output Data Rate 20Hz */
#define BMX055_MAG_DATA_RATE_20	0b101000
/* Magnetometer Output Data Rate 25Hz */
#define BMX055_MAG_DATA_RATE_25	0b110000
/* Magnetometer Output Data Rate 30Hz */
#define BMX055_MAG_DATA_RATE_30	0b111000
/// @}

/// @name Mag Operation Mode Control Parameter
/// @{
#define BMX055_MAG_OP_MODE_NORMAL	0b000
#define BMX055_MAG_OP_MODE_FORCED 0b010
#define BMX055_MAG_OP_MODE_SLEEP	0b110
/// @}

/// @name Mag Self Test Control Parameter
/// @{
#define BMX055_MAG_TEST_NORMAL		0b0
#define BMX055_MAG_TEST_SELF_TEST 0b1
/// @}

/// @name Mag Power Control Parameter
/// @{
#define BMX055_MAG_POW_CTL_SOFT_RESET		0b10000010
#define BMX055_MAG_POW_CTL_SLEEP_MODE		0b00000001
#define BMX055_MAG_POW_CTL_SUSPEND_MODE		0b00000000
/// @}

enum ACCBW {    // define BMX055 accelerometer bandwidths
  ABW_8Hz,      // 7.81 Hz,  64 ms update time
  ABW_16Hz,     // 15.63 Hz, 32 ms update time
  ABW_31Hz,     // 31.25 Hz, 16 ms update time
  ABW_63Hz,     // 62.5  Hz,  8 ms update time
  ABW_125Hz,    // 125   Hz,  4 ms update time
  ABW_250Hz,    // 250   Hz,  2 ms update time
  ABW_500Hz,    // 500   Hz,  1 ms update time
  ABW_100Hz     // 1000  Hz,  0.5 ms update time
};

enum Gscale {
  GFS_2000DPS = 0,
  GFS_1000DPS,
  GFS_500DPS,
  GFS_250DPS,
  GFS_125DPS
};

enum GODRBW {
  G_2000Hz523Hz = 0, // 2000 Hz ODR and unfiltered (bandwidth 523Hz)
  G_2000Hz230Hz,
  G_1000Hz116Hz,
  G_400Hz47Hz,
  G_200Hz23Hz,
  G_100Hz12Hz,
  G_200Hz64Hz,
  G_100Hz32Hz  // 100 Hz ODR and 32 Hz bandwidth
};

enum MODR {
  MODR_10Hz = 0,   // 10 Hz ODR
  MODR_2Hz     ,   // 2 Hz ODR
  MODR_6Hz     ,   // 6 Hz ODR
  MODR_8Hz     ,   // 8 Hz ODR
  MODR_15Hz    ,   // 15 Hz ODR
  MODR_20Hz    ,   // 20 Hz ODR
  MODR_25Hz    ,   // 25 Hz ODR
  MODR_30Hz        // 30 Hz ODR
};

enum Mmode {
  lowPower         = 0,   // rms noise ~1.0 microTesla, 0.17 mA power
  Regular             ,   // rms noise ~0.6 microTesla, 0.5 mA power
  enhancedRegular     ,   // rms noise ~0.5 microTesla, 0.8 mA power
  highAccuracy            // rms noise ~0.3 microTesla, 4.9 mA power
};


typedef struct
{
    float AccelX;
    float AccelY;
    float AccelZ;

    float GyroX;
    float GyroY;
    float GyroZ;

    float MagX;
    float MagY;
    float MagZ;

    float pitch;
    float roll;
    float yaw;

} BMX055_t;

uint8_t BMX055_Init(I2C_HandleTypeDef *I2Cx);

uint8_t SearchDevice(I2C_HandleTypeDef *I2Cx);

void readAccelData(int16_t *destination, I2C_HandleTypeDef *I2Cx);

void readTemp_BMX055(float *destination, I2C_HandleTypeDef *I2Cx);

void readGyroData(int16_t *destination, I2C_HandleTypeDef *I2Cx);

void readMagData(int16_t *destination, I2C_HandleTypeDef *I2Cx);

void BMX055_readAllSensors(I2C_HandleTypeDef *I2Cx, BMX055_t *DataStruct);

void getAcc_Res(void);

void getGyro_Res(void);

#endif /* BMX055_H_ */
