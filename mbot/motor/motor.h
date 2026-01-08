#ifndef __MOTOR_H
#define __MOTOR_H

#include <sys.h>	 
#include "pid.h"
#include "control.h"

//电机PWM变量
extern int    motorLeft,motorRight;                      
//速度
extern int    leftSpeedNow; 
extern int    rightSpeedNow; 
//乘以1000之后的速度设定值
extern int    leftSpeedSet; 
extern int    rightSpeedSet; 

//舵机PWM变量
extern int motorFrontSteer;    
//舵机控制角度的设定值
extern int frontAngleSet;

// * -------------左右轮电机---------------
#define PWMB1   TIM2->CCR4    //PA3
#define PWMB2   TIM2->CCR2    //PA1
#define PWMA1   TIM2->CCR3    //PA2
#define PWMA2   TIM2->CCR1    //PA0 

// * -------------舵机---------------------
#define PWMC    TIM1->CCR2    //PB0

void Steer_Ctrl(int frontAngleSet,int *motorFrontSteer);
void Set_Pwm(int motorLeft,int motorRight,int motorFrontSteer);

u8 Turn_Off(int voltage);
int myabs(int a);


#endif

