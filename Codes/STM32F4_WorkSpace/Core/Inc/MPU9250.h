/*
 * MPU9250.h
 *
 *  Created on: 2023. 7. 12.
 *      Author: miner
 */

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_

#define ALPHA 0.9


typedef struct _MPU9250{
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
	double roll_filtered_prev;
	double pitch_filtered;
	double pitch_filtered_prev;

}MPU9250;


extern MPU9250 IMU;

extern int count_ms;
extern int curTime, prevTime, dt;


void MPU9250_Init(void);
void MPU9250_Calibration(void);
void MPU9250_TransmitData(unsigned char Address, unsigned char Data);
unsigned char MPU9250_ReceiveData(unsigned char Address);

void MPU9250_GetAccel(void);
void MPU9250_GetGyro(void);

void MPU9250_GetRoll_Acc(void);
void MPU9250_GetPitch_Acc(void);
void MPU9250_GetRoll_Gyr(void);
void MPU9250_GetPitch_Gyr(void);
void MPU9250_getRoll_Filtered(void);
void MPU9250_getPitch_Filtered(void);

void getDeltaTime(void);
double complementaryFilter_double(double val1, double val2);


#endif /* INC_MPU9250_H_ */
