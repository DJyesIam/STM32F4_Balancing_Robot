/*
 * MPU6050.h
 *
 *  Created on: 2023. 7. 12.
 *      Author: miner
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

typedef struct _MPU6050{
	unsigned short ax;
	unsigned short ay;
	unsigned short az;
	unsigned short gx;
	unsigned short gy;
	unsigned short gz;
}MPU6050;

extern MPU6050 IMU;

void MPU6050_Init(void);
void MPU6050_TransmitData(unsigned char Address, unsigned char Data);
unsigned char MPU6050_ReceiveData(unsigned char Address);
void MPU6050_GetAccel(void);
void MPU6050_GetGyro(void);

#endif /* INC_MPU6050_H_ */
