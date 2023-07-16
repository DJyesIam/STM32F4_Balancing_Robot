/*
 * MPU6050.c
 *
 *  Created on: 2023. 7. 12.
 *      Author: miner
 */
#include "i2c.h"
#include "MPU6050.h"

MPU6050 IMU;

void MPU6050_Init(void){

	LL_I2C_Enable(I2C1);

	LL_I2C_GenerateStartCondition(I2C1);
	while(!LL_I2C_IsActiveFlag_SB(I2C1));

	LL_I2C_TransmitData8(I2C1, 0x68 << 1);	// MPU6050 Address(Write Mode)
	while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));
	LL_I2C_ClearFlag_ADDR(I2C1);

	LL_I2C_TransmitData8(I2C1, 0x6B);		// PWR_MGMT_1 Register
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));

	LL_I2C_TransmitData8(I2C1, 0x00);		// Write 0x00(SLEEP 0)
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));
	while(!LL_I2C_IsActiveFlag_BTF(I2C1));

	LL_I2C_GenerateStopCondition(I2C1);

}

unsigned char MPU6050_ReceiveData(unsigned char Address){

	LL_I2C_GenerateStartCondition(I2C1);
	while(!LL_I2C_IsActiveFlag_SB(I2C1));

	LL_I2C_TransmitData8(I2C1, 0x68 << 1);
	while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));
	LL_I2C_ClearFlag_ADDR(I2C1);

	LL_I2C_TransmitData8(I2C1, Address);
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));
	while(!LL_I2C_IsActiveFlag_BTF(I2C1));

	LL_I2C_GenerateStartCondition(I2C1);
	while(!LL_I2C_IsActiveFlag_SB(I2C1));

	LL_I2C_TransmitData8(I2C1, (0x68 << 1 | 0x01));
	while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
	LL_I2C_ClearFlag_ADDR(I2C1);

	while(!LL_I2C_IsActiveFlag_RXNE(I2C1));
	unsigned char Data = LL_I2C_ReceiveData8(I2C1);
	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);

	LL_I2C_GenerateStopCondition(I2C1);

	return Data;
}

void MPU6050_GetAccel(void){

	IMU.ax = MPU6050_ReceiveData(0x3B) << 8 | MPU6050_ReceiveData(0x3C);
	IMU.ay = MPU6050_ReceiveData(0x3D) << 8 | MPU6050_ReceiveData(0x3E);
	IMU.az = MPU6050_ReceiveData(0x3F) << 8 | MPU6050_ReceiveData(0x40);
}

void MPU6050_GetGyro(void){
	IMU.gx = MPU6050_ReceiveData(0x43) << 8 | MPU6050_ReceiveData(0x44);
	IMU.gy = MPU6050_ReceiveData(0x45) << 8 | MPU6050_ReceiveData(0x46);
	IMU.gz = MPU6050_ReceiveData(0x47) << 8 | MPU6050_ReceiveData(0x48);
}
