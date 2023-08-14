/*
 * MPU6050.c
 *
 *  Created on: 2023. 7. 12.
 *      Author: miner
 */
#include "i2c.h"
#include "spi.h"
#include "MPU6050.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>


MPU6050 IMU;

int curTime, prevTime, dt;

void MPU6050_Init(void){
//	======================== LL_I2C ========================
//	HAL_Delay(100);
//	LL_I2C_Enable(I2C1);
//
//	LL_I2C_GenerateStartCondition(I2C1);
//	while(!LL_I2C_IsActiveFlag_SB(I2C1));
//
//	LL_I2C_TransmitData8(I2C1, 0x68 << 1);	// MPU6050 Address(Write Mode)
//	while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
//	while(!LL_I2C_IsActiveFlag_TXE(I2C1));
//	LL_I2C_ClearFlag_ADDR(I2C1);
//
//	LL_I2C_TransmitData8(I2C1, 0x6B);		// PWR_MGMT_1 Register
//	while(!LL_I2C_IsActiveFlag_TXE(I2C1));
//
//	LL_I2C_TransmitData8(I2C1, 0x00);		// Write 0x00(SLEEP 0)
//	while(!LL_I2C_IsActiveFlag_TXE(I2C1));
//	while(!LL_I2C_IsActiveFlag_BTF(I2C1));
//
//	LL_I2C_GenerateStopCondition(I2C1);

//	======================== HAL_I2C ========================
//	HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x6B, 1, 0x00, 1, 10);


//	======================== LL_SPI ========================
//	LL_SPI_Enable(SPI1);
//
//	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
//
//	LL_SPI_TransmitData8(SPI1, 0x7F & 0x6B);
//	while(!LL_SPI_IsActiveFlag_TXE(SPI1));
//
//	LL_SPI_TransmitData8(SPI1, 0x00);
//	while(!LL_SPI_IsActiveFlag_TXE(SPI1));
//
//	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);


//	======================== HAL_SPI ========================
	printf("Waiting for 5 seconds to start...\n\n");
	HAL_Delay(5000);

	HAL_SPI_Init(&hspi1);

	printf("MPU6050 Initialization has completed.\n\n");
}

void MPU6050_Calibration(void){
	IMU.ax_offset = 0;
	IMU.ay_offset = 0;
	IMU.az_offset = 0;
	IMU.gx_offset = 0;
	IMU.gy_offset = 0;
	IMU.gz_offset = 0;

	long long ax_offset_sum = 0;
	long long ay_offset_sum = 0;
	long long az_offset_sum = 0;
	long long gx_offset_sum = 0;
	long long gy_offset_sum = 0;
	long long gz_offset_sum = 0;

	for (int i = 0; i < 100; i++){
		MPU6050_GetAccel();
		MPU6050_GetGyro();

		ax_offset_sum += IMU.ax;
		ay_offset_sum += IMU.ay;
		az_offset_sum += IMU.az + 16384;
		gx_offset_sum += IMU.gx;
		gy_offset_sum += IMU.gy;
		gz_offset_sum += IMU.gz;
	}

	IMU.ax_offset = ax_offset_sum / 1000;
	IMU.ay_offset = ay_offset_sum / 1000;
	IMU.az_offset = az_offset_sum / 1000;
	IMU.gx_offset = gx_offset_sum / 1000;
	IMU.gy_offset = gy_offset_sum / 1000;
	IMU.gz_offset = gz_offset_sum / 1000;

	printf("MPU6050 Calibration has completed.\n\n");
}

void MPU6050_TransmitData(unsigned char Address, unsigned char Data){

	LL_I2C_GenerateStartCondition(I2C1);
	while(!LL_I2C_IsActiveFlag_SB(I2C1));

	LL_I2C_TransmitData8(I2C1, 0x68 << 1);	// MPU6050 Address(Write Mode)
	while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));
	LL_I2C_ClearFlag_ADDR(I2C1);

	LL_I2C_TransmitData8(I2C1, Address);	// Transmit Address
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));

	LL_I2C_TransmitData8(I2C1, Data);		// Transmit Data
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));
	while(!LL_I2C_IsActiveFlag_BTF(I2C1));

	LL_I2C_GenerateStopCondition(I2C1);
}

unsigned char MPU6050_ReceiveData(unsigned char Address){

//	======================== LL_I2C ========================
//	LL_I2C_DisableIT_RX(I2C1);
//
//	if(!LL_I2C_IsEnabled(I2C1)) LL_I2C_Enable(I2C1);
//
//	LL_I2C_DisableBitPOS(I2C1);
//
//	while(LL_I2C_IsActiveFlag_BUSY(I2C1));
//
//	LL_I2C_GenerateStartCondition(I2C1);
//	while(!LL_I2C_IsActiveFlag_SB(I2C1));
//
//	LL_I2C_TransmitData8(I2C1, 0x68 << 1);
//	while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
//	while(!LL_I2C_IsActiveFlag_TXE(I2C1));
//	LL_I2C_ClearFlag_ADDR(I2C1);
//
//	LL_I2C_TransmitData8(I2C1, Address);
//	while(!LL_I2C_IsActiveFlag_TXE(I2C1));
//	while(!LL_I2C_IsActiveFlag_BTF(I2C1));
//
//	LL_I2C_GenerateStartCondition(I2C1);
//	while(!LL_I2C_IsActiveFlag_SB(I2C1));
//
//	LL_I2C_TransmitData8(I2C1, (0x68 << 1 | 0x01));
//	while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
//	LL_I2C_ClearFlag_ADDR(I2C1);
//
//	while(!LL_I2C_IsActiveFlag_RXNE(I2C1));
//	unsigned char Data = LL_I2C_ReceiveData8(I2C1);
//	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
//
//	LL_I2C_GenerateStopCondition(I2C1);
//
//	return Data;


//	======================== HAL_I2C ========================
//	unsigned char Data = 0x00;
//	HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, Address, 1, &Data, 1, 10);
//	return Data;


//	======================== LL_SPI ========================
//	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
//
//	LL_SPI_TransmitData8(SPI1, 0x80 | Address);
//	while(!LL_SPI_IsActiveFlag_TXE(SPI1));
//
//	while(!LL_SPI_IsActiveFlag_RXNE(SPI1));
//	LL_SPI_ReceiveData8(SPI1);
//
//	LL_SPI_TransmitData8(SPI1, 0x00);
//	while(!LL_SPI_IsActiveFlag_TXE(SPI1));
//
//	while(!LL_SPI_IsActiveFlag_RXNE(SPI1));
//	unsigned char Data = LL_SPI_ReceiveData8(SPI1);
//
//	while(LL_SPI_IsActiveFlag_BSY(SPI1));
//	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
//
//	return Data;

//	======================== HAL_SPI ========================
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);

	unsigned char dataReg = 0x80 | Address;

	HAL_SPI_Transmit(&hspi1, &dataReg, 1, 100);

	unsigned char Data;
	HAL_SPI_Receive(&hspi1, &Data, 1, 100);


	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);

	return Data;
}

void MPU6050_GetAccel(void){
	IMU.ax = MPU6050_ReceiveData(0x3B) << 8 | MPU6050_ReceiveData(0x3C);
	IMU.ay = MPU6050_ReceiveData(0x3D) << 8 | MPU6050_ReceiveData(0x3E);
	IMU.az = MPU6050_ReceiveData(0x3F) << 8 | MPU6050_ReceiveData(0x40);

	IMU.ax -= IMU.ax_offset;
	IMU.ay -= IMU.ay_offset;
	IMU.az -= IMU.az_offset;
}

void MPU6050_GetGyro(void){
	IMU.gx = MPU6050_ReceiveData(0x43) << 8 | MPU6050_ReceiveData(0x44);
	IMU.gy = MPU6050_ReceiveData(0x45) << 8 | MPU6050_ReceiveData(0x46);
	IMU.gz = MPU6050_ReceiveData(0x47) << 8 | MPU6050_ReceiveData(0x48);

	IMU.gx -= IMU.gx_offset;
	IMU.gy -= IMU.gy_offset;
	IMU.gz -= IMU.gz_offset;
}

void MPU6050_GetRoll_Acc(void){
	IMU.roll_acc = atan(IMU.ay / (sqrt(-IMU.ax * IMU.ax + IMU.az * IMU.az))) * 180 / M_PI;
}

void MPU6050_GetPitch_Acc(void){
	IMU.pitch_acc = atan(IMU.ax / (sqrt(-IMU.ay * IMU.ay + IMU.az * IMU.az))) * 180 / M_PI;
}

void MPU6050_GetRoll_Gyr(void){
	IMU.roll_gyr = IMU.gx * dt / 1000.f;
}

void MPU6050_GetPitch_Gyr(void){
	IMU.pitch_gyr = IMU.gy * dt / 1000.f;
}

void MPU6050_getRoll_Filtered(void){
	IMU.roll_filtered = complementaryFilter_double(IMU.roll_acc, IMU.roll_gyr);
}

void MPU6050_getPitch_Filtered(void){
	IMU.pitch_filtered = complementaryFilter_double(IMU.pitch_acc, IMU.pitch_gyr);
}

void getDeltaTime(void){
	prevTime = curTime;
	curTime = count_ms;
	dt = curTime - prevTime;
	curTime = count_ms;
}

double complementaryFilter_double(double val1, double val2){
	return ALPHA * val1 + (1 - ALPHA) * val2;
}
