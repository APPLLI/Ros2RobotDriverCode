#include "control.h"	
#include "pid.h"
#include "motor.h"
#include "adc.h"
#include "encoder.h"
#include "led.h"
#include "beep.h"

int   Voltage          =0;           //电池电压采样相关的变量
float yaw              =0;           //转向陀螺仪
float yaw_acc_error    =0;           //yaw累积误差
#define FIVE_MS_ERROR   0.00002115   //yaw每5ms的向上漂移的度数，这里近似线性，可以做到半小时偏1度，每个人的这个值可能有所不同，可以自行计算


/**************************************************************************
函数功能：所有的控制代码都在这里面
          5ms定时中断由MPU6050的INT引脚触发		
**************************************************************************/
void EXTI9_5_IRQHandler(void) 
{                                                         
	EXTI_ClearITPendingBit(EXTI_Line8);                             //===清除LINE8线路挂起位		
	
	Led_Flash(200);                                                 //===LED闪烁，证明程序正常运行	
	
	yaw_acc_error += FIVE_MS_ERROR;								    //===yaw漂移误差累加
	
	Get_battery_volt_average(&Voltage,100);		                    //===电池电压测量，单位mv，100次取一次平均
	
	Get_Motor_Speed(&leftSpeedNow,&rightSpeedNow);                  //===获取左右轮子真实速度
	
	pid_Task_Letf.speedSet  = leftSpeedSet;	                        //===给速度设定值和实时值赋值
	pid_Task_Right.speedSet = rightSpeedSet;
	pid_Task_Letf.speedNow  = leftSpeedNow;
	pid_Task_Right.speedNow = rightSpeedNow;
	
	Pid_Ctrl(&motorLeft,&motorRight);                               //===执行PID控制函数
	Steer_Ctrl(frontAngleSet,&motorFrontSteer);                     //===执行舵机，差速小车无需修改完全兼容

	if(Turn_Off(Voltage) == 0)                                      //===如果电量异常低
	{	
		BEEP_Disable();                                             //===正常情况，蜂鸣器失能 
		Set_Pwm(motorLeft,motorRight,motorFrontSteer);              //===赋值给PWM寄存器 		
	} 	  
	else
	{
		BEEP_Flash(20);                                            	//===电量报警
	}
} 








