#include "pid.h"

/*===================================================================
功能说明 : 双路电机速度 PID 控制
作者说明 : 小车项目底层控制模块

版本历史 :
  - 2024-07  初始版本：
      * 使用增量式 PID 控制左右轮速度，参数以“快速响应”为主，适合直线加速。
  - 2025-07  更新版本：
      * 适当降低 Kd，引入轻微积分（Ki > 0），减小中低速段抖动并改善稳态误差。
      * 调整 Ur 上限，使得在急加速时不至于过度打满 PWM，提升电池与电机寿命。
说明     :
  - 由于 MCU 不带 FPU，本模块仍使用“×1024 的定点表示”，所有参数均预先放大。
  - 若在实际车上出现振荡，可适当减小 Kp 或增大 Kd，同时限制速度设定值变化斜率。
===================================================================*/

struct pid_uint pid_Task_Letf;
struct pid_uint pid_Task_Right;

/****************************************************************************
*�������ƣ�PID_Init(void)
*�������ܣ���ʼ��PID�ṹ�����
****************************************************************************/

void PID_Init(void)
{
// 说明：仍然使用 1024 作为放大倍数做定点运算，确保在 M3 上运算效率
/*********************** 左轮速度 PID（2025-07 调整版） ****************************/
	pid_Task_Letf.Kp = 1024 * 3.5f;    // 略微降低比例，减轻振荡
 	pid_Task_Letf.Ki = 1024 * 0.02f;  // 引入微小积分，减小稳态误差
	pid_Task_Letf.Kd = 1024 * 12.0f;  // 减小微分抑制，避免噪声放大
	pid_Task_Letf.Ur = 1024 * 3500;   // 限制输出上限，保护电机与驱动
	pid_Task_Letf.Adjust   = 0;
	pid_Task_Letf.En       = 1;
	pid_Task_Letf.speedSet = 0;
	pid_Task_Letf.speedNow = 0;
	reset_Uk(&pid_Task_Letf);		
/*********************** 右轮速度 PID（2025-07 调整版） ****************************/
	pid_Task_Right.Kp = 1024 * 3.5f;
 	pid_Task_Right.Ki = 1024 * 0.02f;	// 轻微积分
	pid_Task_Right.Kd = 1024 * 12.0f; 
	pid_Task_Right.Ur = 1024 * 3500;
	pid_Task_Right.Adjust   = 0;
	pid_Task_Right.En       = 1;
	pid_Task_Right.speedSet = 0;
	pid_Task_Right.speedNow = 0;
	reset_Uk(&pid_Task_Right);
}

/***********************************************************************************************
 �� �� ����void reset_Uk(PID_Uint *p)
 ��    �ܣ���ʼ��U_kk,ekk,ekkk
 ˵    �����ڳ�ʼ��ʱ���ã��ı�PID����ʱ�п�����Ҫ����
 ��ڲ�����PID��Ԫ�Ĳ����ṹ�� ��ַ
************************************************************************************************/

void reset_Uk(struct pid_uint *p)
{
	p->U_kk=0;
	p->ekk=0;
	p->ekkk=0;
}

/***********************************************************************************************
 �� �� ����s32 PID_commen(int set,int jiance,PID_Uint *p)
 ��    �ܣ�PID���㺯��
 ˵    ���������ⵥ��PID�Ŀ�����
 ��ڲ���������ֵ��ʵ��ֵ��PID��Ԫ�ṹ��
 �� �� ֵ��PID������
************************************************************************************************/

s32 PID_common(int set,int jiance,struct pid_uint *p)
{
	int ek=0,U_k=0;

	ek=jiance - set;                                                               
	
	U_k=p->U_kk + p->Kp*(ek - p->ekk) + p->Ki*ek + p->Kd*(ek - 2*p->ekk + p->ekkk);
	
	p->U_kk=U_k;
    p->ekkk=p->ekk;
	p->ekk=ek;
	
	if(U_k>(p->Ur))		                                    
		U_k=p->Ur;
	if(U_k<-(p->Ur))
		U_k=-(p->Ur);
	
	return U_k>>10; 
}

/***********************************************************************************
** �������� ��void Pid_Which(struct pid_uint *pl, struct pid_uint *pr)
** �������� ��pidѡ����	      
***********************************************************************************/

void Pid_Which(struct pid_uint *pl, struct pid_uint *pr)
{
	/**********************�����ٶ�pid*************************/
	if(pl->En == 1)
	{									
		pl->Adjust = PID_common(pl->speedSet, pl->speedNow, pl);	// * ������������ת�����Ը�������ķ���	
	}	
	else
	{
		pl->Adjust = 0;
		reset_Uk(pl);
		pl->En = 2; 
	}
	/***********************�����ٶ�pid*************************/
	if(pr->En == 1)
	{
		pr->Adjust = PID_common(pr->speedSet, pr->speedNow, pr);	// * ������������ת�����Ը�������ķ���	
	}	
	else
	{
		pr->Adjust = 0;
		reset_Uk(pr);
		pr->En = 2; 
	}
}

/*******************************************************************************
 * ��������Pid_Ctrl(int *leftMotor,int  *rightMotor)
 * ����  ��Pid����
 *******************************************************************************/

void Pid_Ctrl(int *leftMotor,int  *rightMotor)
{
	Pid_Which(&pid_Task_Letf, &pid_Task_Right); 
	*leftMotor  += pid_Task_Letf.Adjust;
	*rightMotor += pid_Task_Right.Adjust;
}

