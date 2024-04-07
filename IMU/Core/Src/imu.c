#include "stm32l4xx_hal.h" //for I2C
#include "imu.h"

uint8_t calibrationData[22];

HAL_StatusTypeDef IMU_Calibrate(IMU *dev);
HAL_StatusTypeDef IMU_Read_Offsets(IMU *dev);
HAL_StatusTypeDef IMU_Write_Offsets(IMU *dev);

uint8_t IMU_Initialize (IMU *dev, I2C_HandleTypeDef *i2cHandle) {
	dev->i2cHandle = i2cHandle;
	dev->acc[0] = 0;
	dev->acc[1] = 0;
	dev->acc[2] = 0;
	dev->acc[3] = 0;
	dev->acc[4] = 0;
	dev->acc[5] = 0;


	dev->gyr[0] = 0;
	dev->gyr[1] = 0;
	dev->gyr[2] = 0;
	dev->gyr[3] = 0;
	dev->gyr[4] = 0;
	dev->gyr[5] = 0;


	dev->temp = 0;

	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	// check IDs
	uint8_t regData;

	status = IMU_ReadRegister(dev, BNO055_CHIP_ID_ADDR, &regData );
	errNum += ( status != HAL_OK );

	if ( regData != BNO055_CHIP_ID ) {
		return 255;
	}

	status = IMU_ReadRegister(dev, BNO055_ACCEL_REV_ID_ADDR, &regData );
		errNum += ( status != HAL_OK );

	if ( regData != BNO055_ACC_ID ) {
		return 255;
	}

	status = IMU_ReadRegister(dev, BNO055_MAG_REV_ID_ADDR, &regData );
			errNum += ( status != HAL_OK );

	if ( regData != BNO055_MAG_ID ) {
		return 255;
	}

	status = IMU_ReadRegister(dev, BNO055_GYRO_REV_ID_ADDR, &regData );
			errNum += ( status != HAL_OK );

	if ( regData != BNO055_GYR_ID ) {
		return 255;

	}

	// select hse
	uint8_t clockset = 128;
	status = IMU_WriteRegister(dev, BNO055_SYS_TRIGGER_ADDR, &clockset);
	errNum += ( status != HAL_OK );

	// set ODRs and turn on
	uint8_t units = 0x01; //0b00000001, mg, dps, c
	status = IMU_WriteRegister(dev, BNO055_UNIT_SEL_ADDR, &units);
	errNum += (status != HAL_OK);


	// check calibration
	uint8_t calibrationData;
	status = IMU_ReadRegister(dev, BNO055_CALIB_STAT_ADDR, &calibrationData);
	errNum += (status != HAL_OK);

	if (calibrationData != 255) { // 0b11111111
		status = IMU_Calibrate(dev);
	} else {
		// default mode is config mode
		status = IMU_Write_Offsets(dev);
	}

	//select mode
	uint8_t operatingMode = 0x7; //0bxxxx0111, AMG mode
	status = IMU_WriteRegister(dev, BNO055_OPR_MODE_ADDR, &operatingMode);
	errNum += (status != HAL_OK);


	return errNum; // 0 if successful initialization
}

HAL_StatusTypeDef IMU_Calibrate(IMU *dev) {
	uint8_t calibrationData = 0;
	HAL_StatusTypeDef status;
	while (calibrationData != 255) {
		status = IMU_ReadRegister(dev, BNO055_CALIB_STAT_ADDR, &calibrationData);
//		uint8_t bits_7_6 = (calibrationData >> 6) & 0b11;
//		uint8_t bits_5_4 = (calibrationData >> 4) & 0b11;
//		uint8_t bits_3_2 = (calibrationData >> 2) & 0b11;
//		uint8_t bits_1_0 = (calibrationData >> 0) & 0b11;
	}
	// done calibrating, so read offsets
	IMU_Read_Offsets(dev);
	return status;


}
HAL_StatusTypeDef IMU_Read_Offsets(IMU *dev) {
	HAL_StatusTypeDef status;
	//Read acceleration offsets
	status = IMU_ReadRegister(dev, BNO055_ACCEL_OFFSET_X_LSB_ADDR, &calibrationData[0]);
	status = IMU_ReadRegister(dev, BNO055_ACCEL_OFFSET_X_MSB_ADDR, &calibrationData[1]);
	status = IMU_ReadRegister(dev, BNO055_ACCEL_OFFSET_Y_LSB_ADDR, &calibrationData[2]);
	status = IMU_ReadRegister(dev, BNO055_ACCEL_OFFSET_Y_MSB_ADDR, &calibrationData[3]);
	status = IMU_ReadRegister(dev, BNO055_ACCEL_OFFSET_Z_LSB_ADDR, &calibrationData[4]);
	status = IMU_ReadRegister(dev, BNO055_ACCEL_OFFSET_Z_MSB_ADDR, &calibrationData[5]);

	//Read magnetometer offsets
	status = IMU_ReadRegister(dev, BNO055_MAG_OFFSET_X_LSB_ADDR, &calibrationData[6]);
	status = IMU_ReadRegister(dev, BNO055_MAG_OFFSET_X_MSB_ADDR, &calibrationData[7]);
	status = IMU_ReadRegister(dev, BNO055_MAG_OFFSET_Y_LSB_ADDR, &calibrationData[8]);
	status = IMU_ReadRegister(dev, BNO055_MAG_OFFSET_Y_MSB_ADDR, &calibrationData[9]);
	status = IMU_ReadRegister(dev, BNO055_MAG_OFFSET_Z_LSB_ADDR, &calibrationData[10]);
	status = IMU_ReadRegister(dev, BNO055_MAG_OFFSET_Z_MSB_ADDR, &calibrationData[11]);

	//Read gyroscope offsets
	status = IMU_ReadRegister(dev, BNO055_GYRO_OFFSET_X_LSB_ADDR, &calibrationData[12]);
	status = IMU_ReadRegister(dev, BNO055_GYRO_OFFSET_X_MSB_ADDR, &calibrationData[13]);
	status = IMU_ReadRegister(dev, BNO055_GYRO_OFFSET_Y_LSB_ADDR, &calibrationData[14]);
	status = IMU_ReadRegister(dev, BNO055_GYRO_OFFSET_Y_MSB_ADDR, &calibrationData[15]);
	status = IMU_ReadRegister(dev, BNO055_GYRO_OFFSET_Z_LSB_ADDR, &calibrationData[16]);
	status = IMU_ReadRegister(dev, BNO055_GYRO_OFFSET_Z_MSB_ADDR, &calibrationData[17]);

	//Read accleration radius
	status = IMU_ReadRegister(dev, BNO055_ACCEL_RADIUS_LSB_ADDR, &calibrationData[18]);
	status = IMU_ReadRegister(dev, BNO055_ACCEL_RADIUS_MSB_ADDR, &calibrationData[19]);

	//Read magnetometer radius
	status = IMU_ReadRegister(dev, BNO055_MAG_RADIUS_LSB_ADDR, &calibrationData[20]);
	status = IMU_ReadRegister(dev, BNO055_MAG_RADIUS_MSB_ADDR, &calibrationData[21]);
	return status;
}

HAL_StatusTypeDef IMU_Write_Offsets(IMU *dev) {
	HAL_StatusTypeDef Status;
	;
	//write acceleration offsets
	Status = IMU_WriteRegister(dev, BNO055_ACCEL_OFFSET_X_LSB_ADDR, &calibrationData[0]);
	Status = IMU_WriteRegister(dev, BNO055_ACCEL_OFFSET_X_MSB_ADDR, &calibrationData[1]);
	Status = IMU_WriteRegister(dev, BNO055_ACCEL_OFFSET_Y_LSB_ADDR, &calibrationData[2]);
	Status = IMU_WriteRegister(dev, BNO055_ACCEL_OFFSET_Y_MSB_ADDR, &calibrationData[3]);
	Status = IMU_WriteRegister(dev, BNO055_ACCEL_OFFSET_Z_LSB_ADDR, &calibrationData[4]);
	Status = IMU_WriteRegister(dev, BNO055_ACCEL_OFFSET_Z_MSB_ADDR, &calibrationData[5]);

	//write magnetometer offsets
	Status = IMU_WriteRegister(dev, BNO055_MAG_OFFSET_X_LSB_ADDR, &calibrationData[6]);
	Status = IMU_WriteRegister(dev, BNO055_MAG_OFFSET_X_MSB_ADDR, &calibrationData[7]);
	Status = IMU_WriteRegister(dev, BNO055_MAG_OFFSET_Y_LSB_ADDR, &calibrationData[8]);
	Status = IMU_WriteRegister(dev, BNO055_MAG_OFFSET_Y_MSB_ADDR, &calibrationData[9]);
	Status = IMU_WriteRegister(dev, BNO055_MAG_OFFSET_Z_LSB_ADDR, &calibrationData[10]);
	Status = IMU_WriteRegister(dev, BNO055_MAG_OFFSET_Z_MSB_ADDR, &calibrationData[11]);

	//write gyroscope offsets
	Status = IMU_WriteRegister(dev, BNO055_GYRO_OFFSET_X_LSB_ADDR, &calibrationData[12]);
	Status = IMU_WriteRegister(dev, BNO055_GYRO_OFFSET_X_MSB_ADDR, &calibrationData[13]);
	Status = IMU_WriteRegister(dev, BNO055_GYRO_OFFSET_Y_LSB_ADDR, &calibrationData[14]);
	Status = IMU_WriteRegister(dev, BNO055_GYRO_OFFSET_Y_MSB_ADDR, &calibrationData[15]);
	Status = IMU_WriteRegister(dev, BNO055_GYRO_OFFSET_Z_LSB_ADDR, &calibrationData[16]);
	Status = IMU_WriteRegister(dev, BNO055_GYRO_OFFSET_Z_MSB_ADDR, &calibrationData[17]);
	//write accleration radius
	Status = IMU_WriteRegister(dev, BNO055_ACCEL_RADIUS_LSB_ADDR, &calibrationData[18]);
	Status = IMU_WriteRegister(dev, BNO055_ACCEL_RADIUS_MSB_ADDR, &calibrationData[19]);

	//write magnetometer radius
	Status = IMU_WriteRegister(dev, BNO055_MAG_RADIUS_LSB_ADDR, &calibrationData[20]);
	Status = IMU_WriteRegister(dev, BNO055_MAG_RADIUS_MSB_ADDR, &calibrationData[21]);
	return Status;
}

/*
 * LOW LEVEL FUNCTIONS
 */


HAL_StatusTypeDef IMU_ReadRegister(IMU *dev, uint8_t reg, uint8_t *data) {
//	return HAL_I2C_Mem_Read(dev -> i2cHandle, BNO055_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
	// version with Master
	return HAL_I2C_Master_Receive(dev -> i2cHandle, BNO055_I2C_ADDR, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef IMU_ReadRegisters(IMU *dev, uint8_t reg, uint8_t *data, uint8_t length) {
	return HAL_I2C_Mem_Read(dev -> i2cHandle, BNO055_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef IMU_WriteRegister(IMU *dev, uint8_t reg, uint8_t *data) {
//	return HAL_I2C_Mem_Write(dev -> i2cHandle, BNO055_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
	return HAL_I2C_Master_Transmit(dev -> i2cHandle, BNO055_I2C_ADDR, data, 1, HAL_MAX_DELAY);

}


/*
 * DATA ACQUISITION
 */
HAL_StatusTypeDef IMU_ReadTemperature(IMU *dev) {
	uint8_t regData;
	HAL_StatusTypeDef status = IMU_ReadRegister(dev, BNO055_TEMP_ADDR, &regData);
	dev->temp = regData;
	// convert from 16bit twos complement
	return status;
}

HAL_StatusTypeDef IMU_ReadGyroscope(IMU *dev) {
	uint8_t regDataGX[2];
	uint8_t regDataGY[2];
	uint8_t regDataGZ[2];

	HAL_StatusTypeDef status = IMU_ReadRegister(dev, BNO055_GYRO_DATA_X_LSB_ADDR, &regDataGX[0]);
	status = IMU_ReadRegister(dev, BNO055_GYRO_DATA_X_LSB_ADDR, &regDataGX[1]);
	status = IMU_ReadRegister(dev, BNO055_GYRO_DATA_Y_LSB_ADDR, &regDataGY[0]);
	status = IMU_ReadRegister(dev, BNO055_GYRO_DATA_Y_MSB_ADDR, &regDataGY[1]);
	status = IMU_ReadRegister(dev, BNO055_GYRO_DATA_Z_LSB_ADDR, &regDataGZ[0]);
	status = IMU_ReadRegister(dev, BNO055_GYRO_DATA_Z_MSB_ADDR, &regDataGZ[1]);
//	uint16_t gyroRawX = (((uint16_t) regDataGX[1] << 8 ) | (uint16_t) regDataGX[0]); // combining data from both registers X
//	uint16_t gyroRawY = (((uint16_t) regDataGY[1] << 8 ) | (uint16_t) regDataGY[0]); // combining data from both registers Y
//	uint16_t gyroRawZ = (((uint16_t) regDataGZ[1] << 8 ) | (uint16_t) regDataGZ[0]); // combining data from both registers Z

	// convert from 16bit twos complement to dps
//	dev->gyr[0] = 0.0076293945f * (int16_t) gyroRawX;

	dev->gyr[0] = regDataGX[0];
	dev->gyr[1] = regDataGX[1];
	dev->gyr[2] = regDataGY[0];
	dev->gyr[3] = regDataGY[1];
	dev->gyr[4] = regDataGZ[0];
	dev->gyr[5] = regDataGZ[1];


	return status;

}

HAL_StatusTypeDef IMU_ReadAccelerometer(IMU *dev) {
	uint8_t regDataAX[2];
	uint8_t regDataAY[2];
	uint8_t regDataAZ[2];



	HAL_StatusTypeDef status = IMU_ReadRegister(dev, BNO055_ACCEL_DATA_X_LSB_ADDR, &regDataAX[0]);
	IMU_ReadRegister(dev, BNO055_ACCEL_DATA_X_MSB_ADDR, &regDataAX[1]);
	IMU_ReadRegister(dev, BNO055_ACCEL_DATA_Y_LSB_ADDR, &regDataAY[0]);
	IMU_ReadRegister(dev, BNO055_ACCEL_DATA_Y_MSB_ADDR, &regDataAY[1]);
	IMU_ReadRegister(dev, BNO055_ACCEL_DATA_Z_LSB_ADDR, &regDataAZ[0]);
	IMU_ReadRegister(dev, BNO055_ACCEL_DATA_Z_MSB_ADDR, &regDataAZ[1]);
//	uint16_t accelRawX = (((uint16_t) regDataAX[1] << 8 ) | (uint16_t) regDataAX[0]); // combining data from both registers X
//	uint16_t accelRawY = (((uint16_t) regDataAY[1] << 8 ) | (uint16_t) regDataAY[0]); // combining data from both registers Y
//	uint16_t accelRawZ = (((uint16_t) regDataAZ[1] << 8 ) | (uint16_t) regDataAZ[0]); // combining data from both registers Z

	// convert from 16bit twos complement to mps
//	dev->acc[0] = 9.81f * 0.00006103515625f * (int16_t) accelRawX;

	dev->acc[0] = regDataAX[0];
	dev->acc[1] = regDataAX[1];
	dev->acc[2] = regDataAY[0];
	dev->acc[3] = regDataAY[1];
	dev->acc[4] = regDataAZ[0];
	dev->acc[5] = regDataAZ[1];

	return status;
}

HAL_StatusTypeDef IMU_ReadMagnetometer(IMU *dev) {
	uint8_t regDataMX[2];
	uint8_t regDataMY[2];
	uint8_t regDataMZ[2];

	HAL_StatusTypeDef status = IMU_ReadRegister(dev, BNO055_MAG_DATA_X_LSB_ADDR, &regDataMX[0]);
	status = IMU_ReadRegister(dev, BNO055_MAG_DATA_X_MSB_ADDR, &regDataMX[1]);
	status = IMU_ReadRegister(dev, BNO055_MAG_DATA_Y_LSB_ADDR, &regDataMY[0]);
	status = IMU_ReadRegister(dev, BNO055_MAG_DATA_Y_MSB_ADDR, &regDataMY[1]);
	status = IMU_ReadRegister(dev, BNO055_MAG_DATA_Z_LSB_ADDR, &regDataMZ[0]);
	status = IMU_ReadRegister(dev, BNO055_MAG_DATA_Z_MSB_ADDR, &regDataMZ[1]);
//	uint16_t gyroRawX = (((uint16_t) regDataGX[1] << 8 ) | (uint16_t) regDataGX[0]); // combining data from both registers X
//	uint16_t gyroRawY = (((uint16_t) regDataGY[1] << 8 ) | (uint16_t) regDataGY[0]); // combining data from both registers Y
//	uint16_t gyroRawZ = (((uint16_t) regDataGZ[1] << 8 ) | (uint16_t) regDataGZ[0]); // combining data from both registers Z

	// convert from 16bit twos complement to dps
//	dev->gyr[0] = 0.0076293945f * (int16_t) gyroRawX;

	dev->mag[0] = regDataMX[0];
	dev->mag[1] = regDataMX[1];
	dev->mag[2] = regDataMY[0];
	dev->mag[3] = regDataMY[1];
	dev->mag[4] = regDataMZ[0];
	dev->mag[5] = regDataMZ[1];


	return status;

}


