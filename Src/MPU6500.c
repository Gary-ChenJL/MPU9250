/*
 * MPU6500.c
 *
 *  Created on: Nov 16, 2022
 *      Author: Garyc
 */
#include <math.h>
#include <MPU6500.h>
#include "stm32f1xx_hal.h"

// Define constants
uint16_t i2c_timeout = 100;

/// @brief Initialize MPU6500 IMU
/// @param I2Cx pointer to I2C_HandleTypeDef structure
/// @return returns 1 if successful, 0 if not
uint8_t MPU6500_init(I2C_HandleTypeDef *I2Cx)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read(I2Cx, MPU6500_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

	if (check == 113) // 0x70 will be returned by the sensor if I2C is working
	{
		Data = 0;
		HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

		// Set DATA RATE of 500Hz by writing SMPLRT_DIV register
		Data = 0x01;
		HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// FS_SEL=0 -> +-2g, check datasheet for other sensitivities
		Data = 0x00;
		HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// FS_SEL=0 -> +-250dps, check datasheet for other sensitivities
		Data = 0x00;
		HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);

		// Set DLPF_CFG to 4 in CONFIG Register
		Data = 0x04;
		HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, CONFIG_REG, 1, &Data, 1, i2c_timeout);

		return 1;
	}
	return 0;
}

/// @brief Initialize MPU9250 Magnetometer
/// @param I2Cx pointer to I2C_HandleTypeDef structure
/// @param Magnetometer pointer to AK8963_t(Magnetometer) structure
/// @return returns 1 if successful, 0 if not
uint8_t MPU9250_initMagnetometer(I2C_HandleTypeDef *I2Cx, AK8963_t *Magnetometer)
{
	uint8_t Data = 0;
	uint8_t check = 0;

	// set I2C_MST_EN to 1
	HAL_I2C_Mem_Read(I2Cx, MPU6500_ADDR, USER_CTRL_REG, 1, &Data, 1, i2c_timeout);
	Data |= 0x20;
	HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, USER_CTRL_REG, 1, &Data, 1, i2c_timeout);

	check = readAK8963Register(I2Cx, MAG_WAI_REG);
	// check if AK8963 is connected
	if (!(check == MAG_WAI_CODE))
	{
		return 0;
	}

	writeAK8963Register(I2Cx, MAG_CNTL_2, 0x01);
	HAL_Delay(100);
	/*Magnetometer operation mode
		"0000": Power-down mode
		"0001": Single measurement mode
		"0010": Continuous measurement mode 1 8Hz
		"0110": Continuous measurement mode 2 100Hz
		"0100": External trigger measurement mode
		"1000": Self-test mode
		"1111": Fuse ROM access mode
	 */
	// Set the magnetometer to Fuse_ROM mode
	MPU9250_Set_MagMode(I2Cx, MAG_MODE_FUSEROM);

	// Read the Sensitivity Adjustment values
	uint8_t raw_data;
	raw_data = readAK8963Register(I2Cx, 0x10);
	Magnetometer->corrfactor_x = (raw_data - 128) * 0.5 / 128 + 1;
	raw_data = readAK8963Register(I2Cx, 0x11);
	Magnetometer->corrfactor_y = (raw_data - 128) * 0.5 / 128 + 1;
	raw_data = readAK8963Register(I2Cx, 0x12);
	Magnetometer->corrfactor_z = (raw_data - 128) * 0.5 / 128 + 1;

	// Set the magnetometer to measurement mode 1
	MPU9250_Set_MagMode(I2Cx, MAG_MODE_8Hz);
	readAK8963Data(I2Cx);
	// return true if initialized successfully
	return 1;
}

/// @brief Read a single byte from the specified AK8963 register
/// @param I2Cx pointer to I2C_HandleTypeDef structure
/// @param reg AK8963 register address
/// @return returns the byte read from the specified register
uint8_t readAK8963Register(I2C_HandleTypeDef *I2Cx, uint8_t reg)
{
	uint8_t data;

	// set slave 0 to read mode and the address of AK8963 I2C address
	data = 0x80 | MAG_I2C_ADDR;
	HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, I2C_SLV4_ADDR, 1, &data, 1, i2c_timeout);

	// AK8963 register to be read
	data = reg;
	HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, I2C_SLV4_REG, 1, &data, 1, i2c_timeout);

	// read
	data = 0x80;
	HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, I2C_SLV4_CTRL, 1, &data, 1, i2c_timeout);
	HAL_I2C_Mem_Read(I2Cx, MPU6500_ADDR, I2C_SLV4_DI, 1, &data, 1, i2c_timeout);

	return data;
}

/// @brief enable reading of magnetometer data into external sensor data registers
/// @param I2Cx pointer to I2C_HandleTypeDef structure
void readAK8963Data(I2C_HandleTypeDef *I2Cx)
{
	uint8_t data;

	// set slave 0 to read mode and the address of AK8963 I2C address
	data = 0x80 | MAG_I2C_ADDR;
	HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, I2C_SLV0_ADDR, 1, &data, 1, i2c_timeout);

	// AK8963 register to be read
	data = MAG_HXL;
	HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, I2C_SLV0_REG, 1, &data, 1, i2c_timeout);

	// read 6 data registers starting from HXL
	data = 0x80 | 0x07;
	HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, I2C_SLV0_CTRL, 1, &data, 1, i2c_timeout);
}

/// @brief write one byte of data to the specified AK8963 register
/// @param I2Cx pointer to I2C_HandleTypeDef structure
/// @param reg destination AK8963 register address
/// @param val byte to be written to the specified register
void writeAK8963Register(I2C_HandleTypeDef *I2Cx, uint8_t reg, uint8_t val)
{
	uint8_t data;

	// set slave 4 to write mode and the address of AK8963 I2C address
	data = 0x00 | MAG_I2C_ADDR;
	HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, I2C_SLV4_ADDR, 1, &data, 1, i2c_timeout);
	data = reg;
	HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, I2C_SLV4_REG, 1, &data, 1, i2c_timeout);

	// write data to I2C_SLV4_DO
	data = val;
	HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, I2C_SLV4_DO, 1, &data, 1, i2c_timeout);

	// Enable data transfer
	data = 0x80;
	HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, I2C_SLV4_CTRL, 1, &data, 1, i2c_timeout);
}

/// @brief read 6-axis(accelerometer and gyroscope) from MPU6500/MPU9250 and store in the MPU6500_t datastruct
/// @param I2Cx pointer to I2C_HandleTypeDef structure
/// @param DataStruct pointer to MPU6500_t datastruct
void MPU6500_readAll(I2C_HandleTypeDef *I2Cx, MPU6500_t *DataStruct)
{
	uint8_t raw_data[14];

	// Read 14 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read(I2Cx, MPU6500_ADDR, ACCEL_XOUT_H_REG, 1, raw_data, 14, i2c_timeout);

	DataStruct->Accel_X_RAW = (int16_t)(raw_data[0] << 8 | raw_data[1]);
	DataStruct->Accel_Y_RAW = (int16_t)(raw_data[2] << 8 | raw_data[3]);
	DataStruct->Accel_Z_RAW = (int16_t)(raw_data[4] << 8 | raw_data[5]);
	DataStruct->Temp_RAW = (int16_t)(raw_data[6] << 8 | raw_data[7]);
	DataStruct->Gyro_X_RAW = (int16_t)(raw_data[8] << 8 | raw_data[9]);
	DataStruct->Gyro_Y_RAW = (int16_t)(raw_data[10] << 8 | raw_data[11]);
	DataStruct->Gyro_Z_RAW = (int16_t)(raw_data[12] << 8 | raw_data[13]);
}

/// @brief Auto-calibrate the MPU6500/MPU9250 accelerometer and gyroscope, and store the offsets in the MPU6500_t datastruct
/// @param I2Cx pointer to I2C_HandleTypeDef structure
/// @param DataStruct pointer to MPU6500_t datastruct
/// NOTE: This function will take 50 samples and take the average of those samples to determine the offset
/// 	  so make sure the sensor is not moving during the calibration process
void MPU6500_calibrate(I2C_HandleTypeDef *I2Cx, MPU6500_t *DataStruct)
{
	// Record and
	for (int i = 0; i < 50; i++)
	{
		MPU6500_readAll(I2Cx, DataStruct);
		DataStruct->accel_x_offset += DataStruct->Accel_X_RAW;
		DataStruct->accel_y_offset += DataStruct->Accel_Y_RAW;
		DataStruct->accel_z_offset += DataStruct->Accel_Z_RAW;
		DataStruct->gyro_x_offset += DataStruct->Gyro_X_RAW;
		DataStruct->gyro_y_offset += DataStruct->Gyro_Y_RAW;
		DataStruct->gyro_z_offset += DataStruct->Gyro_Z_RAW;
	}
	DataStruct->accel_x_offset /= 50.f;
	DataStruct->accel_y_offset /= 50.f;
	DataStruct->accel_z_offset /= 50.f;
	DataStruct->gyro_x_offset /= 50.f;
	DataStruct->gyro_y_offset /= 50.f;
	DataStruct->gyro_z_offset /= 50.f;
}

/// @brief Convert the raw data from the MPU6500/MPU9250 into meaningful values in g's and degrees per second
/// @param DataStruct pointer to MPU6500_t datastruct
/// NOTE: This function should be called after the MPU6500_readAll() function
/// 	  If you changed the scale of accelerometer or gyroscope in MPU6500_init(), you will need to update the denominator according to the datasheet
void MPU6500_getValues(MPU6500_t *DataStruct)
{
	DataStruct->Ax = (DataStruct->Accel_X_RAW - DataStruct->accel_x_offset) / 16384.0;
	DataStruct->Ay = (DataStruct->Accel_Y_RAW - DataStruct->accel_y_offset) / 16384.0;
	DataStruct->Az = (DataStruct->Accel_Z_RAW - DataStruct->accel_z_offset + 16384) / 16384.0;
	DataStruct->Temperature = (float)(((int16_t)DataStruct->Temp_RAW) / (float)337.87 + (float)21.0);
	DataStruct->Gx = (DataStruct->Gyro_X_RAW - DataStruct->gyro_x_offset) / 131.0;
	DataStruct->Gy = (DataStruct->Gyro_Y_RAW - DataStruct->gyro_y_offset) / 131.0;
	DataStruct->Gz = (DataStruct->Gyro_Z_RAW - DataStruct->gyro_z_offset) / 131.0;
}

/// @brief Transmit the data from the MPU6500/MPU9250 to the UART port in a formatted string 
/// @param UARTx pointer to UART_HandleTypeDef structure
/// @param buf pointer to the buffer to store the formatted string
/// @param MPU6500 pointer to MPU6500_t datastruct
void MPU6500_transmitUART(UART_HandleTypeDef *UARTx, uint8_t *buf, MPU6500_t *MPU6500)
{
	snprintf(buf, 128, "Temp: %.2f\n\r Acc_x:%.2f\t Acc_y:%.2f\t Acc_z:%.2f\n\r Gyro_x:%.2f\t Gyro_y:%.2f\t Gyro_z:%.2f\n\r",
			 MPU6500->Temperature, MPU6500->Ax, MPU6500->Ay, MPU6500->Az, MPU6500->Gx, MPU6500->Gy, MPU6500->Gz);
	HAL_UART_Transmit(UARTx, buf, strlen(buf), i2c_timeout);
}

/// @brief Set AK8963 magnetometer mode 
/// @param I2Cx pointer to I2C_HandleTypeDef structure
/// @param Opmode operation mode code (MAG_MODE_*) 
void MPU9250_Set_MagMode(I2C_HandleTypeDef *I2Cx, uint8_t Opmode)
{
	uint8_t regVal;

	regVal = readAK8963Register(I2Cx, MAG_CNTL_1);
	regVal &= 0xF0;
	regVal |= Opmode;
	writeAK8963Register(I2Cx, MAG_CNTL_1, regVal);
	HAL_Delay(10);
	if (Opmode != MAG_MODE_PWRDWN)
	{
		readAK8963Data(I2Cx);
	}
}

/// @brief Read 3-axis AK8963 magnetometer data from external sensor registers(EXT_SENS_DATA_00 ~ EXT_SENS_DATA_06), and store the data in the AK8963_t datastruct 
/// @param I2Cx pointer to I2C_HandleTypeDef structure
/// @param Magnetometer pointer to AK8963_t datastruct
void MPU9250_Read_AK8963Data(I2C_HandleTypeDef *I2Cx, AK8963_t *Magnetometer)
{
	uint8_t raw_data[7];

	// Read 6 bytes of raw data starting from EXT_SENS_DATA_00
	HAL_I2C_Mem_Read(I2Cx, MPU6500_ADDR, EXT_SLV_DATA_00, 1, raw_data, 7, i2c_timeout);
	// Check HOFL bit for sensor overflow
	if (raw_data[6] == 0x18)
	{
		readAK8963Data(I2Cx);
	}
	Magnetometer->hx_raw = (int16_t)raw_data[1] << 8 | raw_data[0];
	Magnetometer->hy_raw = (int16_t)raw_data[3] << 8 | raw_data[2];
	Magnetometer->hz_raw = (int16_t)raw_data[5] << 8 | raw_data[4];

	// convert raw data into magnetic flux density in microTesla
	float sensitivity = 4912.0 / 32760.0;
	Magnetometer->hx = Magnetometer->hx_raw * sensitivity * Magnetometer->corrfactor_x;
	Magnetometer->hy = Magnetometer->hy_raw * sensitivity * Magnetometer->corrfactor_y;
	Magnetometer->hz = Magnetometer->hz_raw * sensitivity * Magnetometer->corrfactor_z;
}

/// @brief Transmit the data from the MPU6500 and AK8963 datastructs to the UART port in a formatted string
/// @param UARTx pointer to UART_HandleTypeDef structure
/// @param buf pointer to the buffer to store the formatted string
/// @param MPU6500 pointer to MPU6500_t datastruct
/// @param Magnetometer pointer to AK8963_t datastruct
void MPU9250_transmitUART(UART_HandleTypeDef *UARTx, uint8_t *buf, MPU6500_t *MPU6500, AK8963_t *Magnetometer)
{
	snprintf(buf, 128, "Temp: %.2f\n\r Acc_x:%.2f\t Acc_y:%.2f\t Acc_z:%.2f\n\r Gyro_x:%.2f\t Gyro_y:%.2f\t Gyro_z:%.2f\n\r H_x:%.2f\t H_y:%.2f\t H_z:%.2f\n\r",
			 MPU6500->Temperature, MPU6500->Ax, MPU6500->Ay, MPU6500->Az, MPU6500->Gx, MPU6500->Gy, MPU6500->Gz,
			 Magnetometer->hx, Magnetometer->hy, Magnetometer->hz);
	HAL_UART_Transmit(UARTx, buf, strlen(buf), i2c_timeout);
}
