#include "stm32f10x.h"                  // Device header
#include "TCS3200.h"
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include <stdbool.h>
#include "GPIO.h"
int	Output_color;
bool IC_colorMode = false;
uint8_t calibrate_number;

uint16_t  TimeColor_H =0, TimeColor_L = 0;
uint16_t TimeColor;
int	Freq;
void TCS3200_init()
{
	
	TIM_TimeBaseInitTypeDef TIM_TCS3200;
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2 , ENABLE);
	

	TIM_TCS3200.TIM_Period = 19999;
	TIM_TCS3200.TIM_Prescaler = 71;
	TIM_TCS3200.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TCS3200.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM2, &TIM_TCS3200);

	TIM_ICInitTypeDef TIM_ICTCS3200;
	TIM_ICTCS3200.TIM_Channel = TIM_Channel_1;
	TIM_ICTCS3200.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICTCS3200.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICTCS3200.TIM_ICFilter = 5;
	TIM_ICTCS3200.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit (TIM2,&TIM_ICTCS3200);
	TIM_Cmd( TIM2,ENABLE);
	
	NVIC_InitTypeDef NVIC_TCS3200;
	NVIC_TCS3200.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_TCS3200.NVIC_IRQChannelCmd = ENABLE;
	NVIC_TCS3200.NVIC_IRQChannelPreemptionPriority =0;
	NVIC_TCS3200.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_TCS3200);
	
}

void TCS3200_config()
	{
		GPIO_type GPIO_pin;
		GPIO_pin.pin = 0 | 1 | 3 | 4;
		GPIO_pin.port = PORTB;
		GPIO_pin.mode = Output_mode;
		GPIO_pin.mode_type = Output_GP_PP;
		GPIO_pin.speed = SPEED_50MHZ;
		port_init(GPIO_pin);
		
	}
void Set_filter(uint8_t mode)
{
	switch (mode){
		case (RED):
		GPIO_ResetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1);
		break;
		case (BLUE):
		GPIO_ResetBits(GPIOB, GPIO_Pin_0);
		GPIO_SetBits(GPIOB, GPIO_Pin_1);
		break;
		case(CLEAR):
		GPIO_ResetBits(GPIOB, GPIO_Pin_1);
		GPIO_SetBits(GPIOB, GPIO_Pin_0);
		break;
		case(GREEN):
		GPIO_SetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1);
		break;
	}
}
	void Set_scaling(uint8_t mode)
	{
		switch(mode)
		{
			case(Scl0):
			GPIO_ResetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_4);
			break;
		case(Scl2):
			GPIO_ResetBits(GPIOB, GPIO_Pin_3);
			GPIO_SetBits(GPIOB, GPIO_Pin_4);
			break;
		case(Scl20):
			GPIO_ResetBits(GPIOB, GPIO_Pin_4);
			GPIO_SetBits(GPIOB, GPIO_Pin_3);
			break;
		case(Scl100):
			GPIO_SetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_4);
			break;
		}
}
	int	Get_color(int	set_color)
	{
		calibrate_number=0;

	TimeColor_H=0;
	TimeColor_L=0;
	TimeColor=0;
	TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
	Set_filter(set_color);
	IC_colorMode = true;
		
	while(IC_colorMode == true);
		
	Set_filter(CLEAR);
	TIM_ITConfig(TIM2, TIM_IT_CC1, DISABLE);
	if(TimeColor_H > TimeColor_L) 
		TimeColor = TimeColor_H - TimeColor_L;
	else
		TimeColor = 0xFFFF - TimeColor_L + TimeColor_H;
	
	Freq= SystemCoreClock/(TimeColor*72);
	
	switch (set_color){
 
		case RED:
			Output_color = (255.0/(MAX_RED-MIN_RED))*(Freq-MIN_RED); 
			break;
		
		case GREEN :
			Output_color = (255.0/(MAX_GREEN-MIN_GREEN))*(Freq-MIN_GREEN);  
			break;		
		
		case BLUE :
			Output_color = (255.0/(MAX_BLUE-MIN_BLUE))*(Freq-MIN_BLUE);		
			break;
	}
	if (Output_color > 255) Output_color = 255;
	if (Output_color < 0) Output_color = 0;
	
	return Output_color	; 	
	}
void TIM2_IRQHandler(void)
{
	
	if (TIM_GetITStatus(TIM2,TIM_IT_CC1)!= RESET){
	
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
		
		if((IC_colorMode == true) && (calibrate_number == 0))
		{
			TimeColor_L = TIM_GetCapture1(TIM2);
			calibrate_number = 1;
		}
		else if((IC_colorMode == true) && (calibrate_number == 1))
		{
			TimeColor_H = TIM_GetCapture1(TIM2);
			IC_colorMode = false;
			calibrate_number = 0;
		}
	}
	
}
