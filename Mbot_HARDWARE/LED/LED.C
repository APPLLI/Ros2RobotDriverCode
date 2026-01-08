#include "led.h"

#define LED_CLOCK  RCC_APB2Periph_GPIOC
#define LED_IO     GPIOC
#define LED_PORT   GPIO_Pin_13

/**************************************************************************
函数功能：LED接口初始化
入口参数：无 
返回  值：无
**************************************************************************/
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(LED_CLOCK, ENABLE);            //使能端口时钟
	GPIO_InitStructure.GPIO_Pin = LED_PORT;	              //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	GPIO_Init(LED_IO, &GPIO_InitStructure);				  //根据设定参数初始化GPIOC 
}

/**************************************************************************
函数功能：LED闪烁
入口参数：闪烁频率 
返回  值：无
**************************************************************************/
void Led_Flash(uint16_t time)
{
	static int temp = 0;
	if(++temp == time)
	{
		if(GPIO_ReadInputDataBit(LED_IO,LED_PORT)==0)
			GPIO_SetBits(LED_IO,LED_PORT);
		else
			GPIO_ResetBits(LED_IO,LED_PORT);
		temp=0;
	}
}
