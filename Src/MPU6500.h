/*
 * mpu6500.h
 *
 *  Created on: Nov 16, 2022
 *  	Author: Garyc
 */

#ifndef INC_MPU6500_H
#define INC_MPU6500_H

#endif /* INC_MPU6500_H */

#include <stdint.h>
#include "stm32f1xx_hal.h"

#define RAD_TO_DEG 57.295779513082320876798154814105

//
#define MPU6500_ADDR 0xD0
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define CONFIG_REG 0x1A
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43
#define I2C_MST_CTRL_REG 0X24
#define USER_CTRL_REG 0x6A
#define I2C_SLV0_ADDR 0x25
#define I2C_SLV0_REG 0x26
#define I2C_SLV0_CTRL 0x27
#define I2C_SLV0_DO 0x63
#define I2C_SLV4_ADDR 0x31
#define I2C_SLV4_REG 0x32
#define I2C_SLV4_CTRL 0x34
#define I2C_SLV4_DO 0x33
#define I2C_SLV4_DI 0x35
#define EXT_SLV_DATA_00 0x49


// Magnetometer registers
#define MAG_I2C_ADDR 0x0C
#define MAG_WAI_CODE 0x48
#define MAG_WAI_REG 0x00
#define MAG_CNTL_1 0x0A
#define MAG_CNTL_2 0x0B
#define MAG_MODE_PWRDWN 0x00
#define MAG_MODE_Single 0x1
#define MAG_MODE_8Hz 0x02
#define MAG_MODE_100Hz 0x06
#define MAG_SELF_TEST 0x8
#define MAG_MODE_FUSEROM 0xF
#define MAG_HXL 0x3
#define MAG_HXH 0x4
#define MAG_HYL 0x5
#define MAG_HYH 0x6
#define MAG_HZL 0x7
#define MAG_HZH 0x8

// MPU6500 structure
typedef struct
{

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    int16_t Temp_RAW;
    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;

    // offsets
    float gyro_x_offset;
	float gyro_y_offset;
	float gyro_z_offset;
	float accel_x_offset;
	float accel_y_offset;
	float accel_z_offset;
} MPU6500_t;

//Magnetometer structure
typedef struct
{
	float hx;
	float hy;
	float hz;

	int16_t  hx_raw;
	int16_t  hy_raw;
	int16_t  hz_raw;

	// Sensitivity Adjustment values
	float corrfactor_x;
	float corrfactor_y;
	float corrfactor_z;
} AK8963_t;

uint8_t MPU6500_init(I2C_HandleTypeDef *I2Cx);

uint8_t MPU9250_initMagnetometer(I2C_HandleTypeDef *I2Cx, AK8963_t *Magnetometer);

uint8_t readAK8963Register(I2C_HandleTypeDef *I2Cx, uint8_t Reg);

void readAK8963Data(I2C_HandleTypeDef *I2Cx);

void writeAK8963Register(I2C_HandleTypeDef *I2Cx, uint8_t reg, uint8_t val);

void MPU6500_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6500_t *DataStruct);

void MPU6500_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6500_t *DataStruct);

void MPU6500_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6500_t *DataStruct);

void MPU6500_readAll(I2C_HandleTypeDef *I2Cx, MPU6500_t *DataStruct);

void MPU6500_calibrate(I2C_HandleTypeDef *I2Cx, MPU6500_t *DataStruct);

void MPU6500_getValues(MPU6500_t *DataStruct);

void MPU6500_transmitUART(UART_HandleTypeDef *UARTx, uint8_t* buf, MPU6500_t* DataStruct);

void MPU9250_Set_MagMode(I2C_HandleTypeDef *I2Cx, uint8_t Opmode);

void MPU9250_Get_MagCorrectCoeff(I2C_HandleTypeDef *I2Cx);

void MPU9250_Read_AK8963Data(I2C_HandleTypeDef *I2Cx, AK8963_t *Magnetometer);

void MPU9250_transmitUART(UART_HandleTypeDef *UARTx, uint8_t* buf, MPU6500_t* DataStruct, AK8963_t* Magnetometer);
