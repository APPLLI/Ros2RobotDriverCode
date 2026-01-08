#include "beep.h"

#define BEEP_CLOCK  RCC_APB2Periph_GPIOA
#define BEEP_IO     GPIOA
#define BEEP_PORT   GPIO_Pin_15

/**************************************************************************
函数功能：BEEP接口初始化
入口参数：无 
返回  值：无
**************************************************************************/
void BEEP_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(BEEP_CLOCK | RCC_APB2Periph_AFIO,ENABLE);//使能端口时钟
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);          
	GPIO_InitStructure.GPIO_Pin = BEEP_PORT;	          //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	GPIO_Init(BEEP_IO, &GPIO_InitStructure);			  //根据设定参数初始化GPIO
	GPIO_ResetBits(BEEP_IO,BEEP_PORT);
}

/**************************************************************************
函数功能：BEEP闪烁
入口参数：闪烁频率 
返回  值：无
**************************************************************************/
void BEEP_Flash(uint16_t time)
{
	static int temp = 0;
	if(++temp == time)
	{
		if(GPIO_ReadInputDataBit(BEEP_IO,BEEP_PORT)==0)
			GPIO_SetBits(BEEP_IO,BEEP_PORT);
		else
			GPIO_ResetBits(BEEP_IO,BEEP_PORT);
		temp=0;
	}
}

/**************************************************************************
函数功能：BEEP关闭
入口参数：无
返回  值：无
**************************************************************************/
void BEEP_Disable()
{
	GPIO_ResetBits(BEEP_IO,BEEP_PORT);
}
