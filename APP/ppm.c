#include "application.h"
#include "ppm.h"
#include "usart.h"

uint16_t ppm_channel[8] = {1500,1500,1000,1500,1500,1500,1500,1500};	//ppm通道值
uint16_t ppm_channel_default[8] = {1500,1500,1000,1500,1500,1500,1500,1500};	//ppm通道值
uint16_t ppm_ch_pos = 0;												//ppm通道值输出位置偏移
uint16_t ppm_out_bit = 0;												//指示当前ppm该输出的电平

void TIM2_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
   	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	//初始化定时器2 TIM2	 
	TIM_TimeBaseStructure.TIM_Period = arr; 					//设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 					//预分频器   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	//中断分组初始化
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  			//TIM2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  	//先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  		//从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 			//IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);					//使能更新中断
	
   	TIM_Cmd(TIM2,ENABLE);
}

void Set_TIM2_Period(uint16_t arr)
{
	TIM2->ARR = arr;
}

//定时器2中断服务程序	 
void TIM2_IRQHandler(void)
{
 	//更新中断
 	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
 	{
 		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
		//Usart_Puts(COM1,"F\n");

 		if(ppm_out_bit == 0)
 		{
 			ppm_out_bit = 1;
 			PPM_PIN = 0;
 			Set_TIM2_Period(300);		//输出固定300us的每通道起始低电平 			
 		}
 		else
 		{
 			ppm_out_bit = 0;
 			PPM_PIN = 1;
 			if(ppm_ch_pos<8)			//正常输出8通道
 				Set_TIM2_Period(ppm_channel[ppm_ch_pos++]-300);
 			else						//输出最后的高电平
 			{
 				uint16_t i,time_add = 0;

 				for(i=0;i<8;i++)
 					time_add += ppm_channel[i];

 				Set_TIM2_Period(20000 - time_add);

 				ppm_ch_pos = 0;
 			}		
 		}
 	}
}

void PPM_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	PPM_PIN = 1;					//复位时PPM引脚置高电平

	TIM2_Init(20000,72-1);			//1MHz计数频率
}

//逻辑通道值转换到PPM通道值，直接复制，并进行范围判断处理，防止错误的数据
void LogicChan2ppm(uint8_t* logic_ch)
{
	uint16_t i;
	uint16_t chan;

	for(i=0;i<8;i++)
	{
		chan = ((uint16_t*)logic_ch)[i];

		if(chan<1000)
			ppm_channel[i] = 1000;
		else if(chan>2000)
			ppm_channel[i] = 2000;
		else
			ppm_channel[i] = chan;
	}
}
