#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h" 

#define ENCODER_TIM_PERIOD (u16)(65535)   //不可大于65535 因为F103的定时器是16位的。

void Encoder_Init_TIM3(void);             //右轮 PA6 PA7
void Encoder_Init_TIM4(void);             //左轮 PB6 PB7

s16 getTIMx_DetaCnt(TIM_TypeDef * TIMx);
void Get_Motor_Speed(int *leftSpeed,int *rightSpeed);

#endif
