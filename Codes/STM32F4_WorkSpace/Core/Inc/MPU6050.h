/*
 * MPU6050.h
 *
 *  Created on: 2023. 7. 12.
 *      Author: miner
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#define ALPHA 0.9


typedef struct _MPU6050{
	short ax;
	short ay;
	short az;
	short gx;
	short gy;
	short gz;

	short ax_offset;
	short ay_offset;
	short az_offset;
	short gx_offset;
	short gy_offset;
	short gz_offset;

	double roll_acc;
	double pitch_acc;

	double roll_gyr;
	double pitch_gyr;

	double roll_filtered;
	double pitch_filtered;
}MPU6050;


extern MPU6050 IMU;

extern unsigned int count_ms;
extern unsigned int curTime, prevTime, dt;


void MPU6050_Init(void);
void MPU6050_Calibration(void);
void MPU6050_TransmitData(unsigned char Address, unsigned char Data);
unsigned char MPU6050_ReceiveData(unsigned char Address);

void MPU6050_GetAccel(void);
void MPU6050_GetGyro(void);

void MPU6050_GetRoll_Acc(void);
void MPU6050_GetPitch_Acc(void);
void MPU6050_GetRoll_Gyr(void);
void MPU6050_GetPitch_Gyr(void);
void MPU6050_getRoll_Filtered(void);
void MPU6050_getPitch_Filtered(void);

void getDeltaTime(void);
double complementaryFilter_double(double val1, double val2);


#endif /* INC_MPU6050_H_ */
