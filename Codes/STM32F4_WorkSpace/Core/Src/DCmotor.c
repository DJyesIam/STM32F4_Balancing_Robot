/*
 * DCmotor.c
 *
 *  Created on: Jul 17, 2023
 *      Author: miner
 */

#include "DCmotor.h"
#include "stm32f4xx_ll_tim.h"
#include <stdio.h>


void DCmotor_Init(void){
	LL_TIM_EnableCounter(TIM2);
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH2);

	printf("DCmotor Initialization has completed.\n\n");
}

void DCmotor_Forward(short PWM){
	TIM2->CCR1 = PWM;
	TIM2->CCR2 = 0;
}

void DCmotor_Backward(short PWM){
	TIM2->CCR1 = 0;
	TIM2->CCR2 = PWM;
}

void DCmotor_Stop(void){
	TIM2->CCR1 = 0;
	TIM2->CCR2 = 0;
}
