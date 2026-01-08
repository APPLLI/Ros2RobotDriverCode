#include "sys.h"

//====================�Լ������ͷ�ļ�===============================
#include "delay.h"
#include "led.h"
#include "myexti.h"
#include "adc.h"
#include "pwm.h"
#include "encoder.h"
#include "usart1.h"	
#include "usart3.h"
#include "ioi2c.h"
#include "mpu6050.h"

#include "show.h"					
#include "mbotLinuxUsart.h"
#include "pid.h"
#include "control.h"
#include "motor.h"
#include "beep.h"
/*===================================================================
功能说明 : ROS 小车底层程序（全向底盘 + 阿克曼转向 + 与上位机通信）
编写时间 : 最初版本 2024-07
当前版本 : 2025-07 版
版本变更 :
  - 2024-07：
      * 完成基础底层驱动，包含编码器测速、PWM 电机控制、舵机转向、MPU6050 姿态解算等。
      * 实现与上位机的串口自定义协议通信。
  - 2025-07：
      * 对主循环发送频率与数据内容做轻微调整，优化与 ROS2 上位机的交互体验。
      * 调整 PID 参数，提升在中低速段的速度跟踪稳定性（减小振荡，改善起步响应）。
      * 为后续 ROS2 路径规划与多传感器融合导航预留扩展接口（在上位机侧新增 astar_nav_node 节点）。
说明     : 本文件主要负责 STM32 底层初始化与与 ROS2 的通信入口，
           与路径规划等高层算法通过串口话题进行解耦。
=====================================================================
------------------ 转载请注明来源，感谢支持与交流 ------------------
===================================================================*/
int main(void)
{ 
	//���ͼ���
	char sendCount=0;
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//����JTAG ���� SWD
	
	MY_NVIC_PriorityGroupConfig(2);	//=====�����жϷ���
	
	delay_init();	    	        //=====��ʱ������ʼ��
	LED_Init();                     //=====LED��ʼ��    �����	
	BEEP_Init();                    //=====��������ʼ�����쳣����
	
	usart1_init(115200);	        //=====����1��ʼ��  ��ݮ��ͨ��
	usart3_init(115200);            //=====����3��ʼ��  ���������ƺ͵���

	IIC_Init();                     //=====IIC��ʼ��    ��ȡMPU6050����
	MPU6050_initialize();           //=====MPU6050��ʼ��	
	DMP_Init();                     //=====��ʼ��DMP 

	Encoder_Init_TIM3();            //=====��ʼ�����ֱ������ӿ�
	Encoder_Init_TIM4();            //=====��ʼ�����ֱ������ӿ�
	
	MY_ADC_Init();                  //=====adc��ʼ��    ��ص������
	
	Motor_PWM_Init(7200-1, 1-1);    //=====��ʼ��PWM 10KHZ(AT8870)������������� 
	Steer_PWM_Init(60000-1, 24-1);  //=====��ʼ��PWM 50HZ�������������

	PID_Init();                     //=====PID��ʼ��
	
	MBOT_EXTI_Init();               //=====MPU6050 5ms��ʱ�жϳ�ʼ��

	while(1)
	{
		// 发送给上位机（树莓派 + ROS2）的速度 / 姿态数据
		// 说明：之前为大约 70Hz，这里改为约 50Hz，降低串口占用并提高上位机解析稳定性。
		if(sendCount==0)             // 约 20ms 发送一帧数据（~50Hz）
		{
			// 角速度放大系数从 50 调整为 100，提升角度分辨率
			usartSendData(USART1,(short)leftSpeedNow,(short)rightSpeedNow,(short)(yaw*100),sendCtrlFlag);
			// 如需在 PC 端调试，可打开下行打印
			// pcShow();
			sendCount++;
		}
		else
		{
			sendCount++;
			if(sendCount==25)
				sendCount=0;
		}
		// 实时读取姿态角（Yaw），用于上位机导航与可视化
		getAngle(&yaw,&yaw_acc_error);
	} 
}

//�жϷ�����
void USART1_IRQHandler()
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
 	 {
		 USART_ClearITPendingBit(USART1,USART_IT_RXNE);//��������жϱ�־λ
		 usartReceiveOneData(USART1,&leftSpeedSet,&rightSpeedSet,&frontAngleSet,&receCtrlFlag);
	 }
}

