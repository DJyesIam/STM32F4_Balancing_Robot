/*
 * DCmotor.h
 *
 *  Created on: Jul 17, 2023
 *      Author: miner
 */

#ifndef INC_DCMOTOR_H_
#define INC_DCMOTOR_H_

#define MOTOR_MIN_PWM = 1000
#define MOTOR_MAX_PWM = 7000

void DCmotor_Init(void);
void DCmotor_Forward(short PWM);
void DCmotor_Backward(short PWM);
void DCmotor_Stop(void);

#endif /* INC_DCMOTOR_H_ */
