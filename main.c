	#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
	#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
	#include "stm32f10x.h"                  // Device header
	#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
	#include "stm32f10x_usart.h"            // Keil::Device:StdPeriph Drivers:USART
	#include "GPIO.h"
	#include "stm32f10x_exti.h"             // Keil::Device:StdPeriph Drivers:EXTI
	#include <stdio.h>
	#include "TCS3200.h"
	#include "stm32f10x_dma.h"              // Keil::Device:StdPeriph Drivers:DMA
	#include "LCD.h"
	#define VIRTUALIZAR_SENSOR 0
	#define IR_SENSOR_PORT PORTA
	#define IR_SENSOR_PIN 1
	int color[3][6];
	void GPIO_config_IR_sensor(void);
	void init_Color(void);
	void UART_SendString(const char* str);
	void UART_SendNumber(const uint32_t a);
	void Read_color(void);
	void Uart_config(void);
	void GPIO_config_TCS3200(void);
 static uint32_t cntY, cntR, cntB, SUM;
uint16_t test, tik=0;
	volatile uint16_t temp =0;
	uint16_t c =0;

	void Motor_init(void);
	void Motor_SetSpeed(uint8_t	speed);
	void Motor_SetDirection(uint8_t	direct);

	void Servo1_init(void);
	void Servo1_SetAngle(uint32_t angle);
	void Servo2_init(void);
	void Servo2_SetAngle(uint32_t angle);
	void I2C_LCD_init(void);
	int R,G,B;
	int main(){
			Servo1_init();
			Servo2_init();
			Motor_init();

		GPIO_config_IR_sensor();
			Systick_init();
			Uart_config();
			GPIO_config_TCS3200();
			TCS3200_init();
			TCS3200_config();
			init_Color();
			I2C_LCD_init();	
			Set_filter(CLEAR);
			Set_scaling(Scl20);
		
		I2C_TypeDef hi2c2;
		I2C_LCD_HandleTypeDef lcd1;
		lcd1.hi2c = &hi2c2;
    lcd1.address = 0x4E;
    lcd_init(&lcd1);                                                                
    lcd_clear(&lcd1);
		
		lcd_gotoxy(&lcd1, 0, 0);
    lcd_puts(&lcd1, "CountR");
		lcd_gotoxy(&lcd1, 0, 1);
		lcd_puts(&lcd1, "CountB");
		lcd_gotoxy(&lcd1, 0, 2);
		lcd_puts(&lcd1, "CountY");
		lcd_gotoxy(&lcd1, 0, 3);
		lcd_puts(&lcd1, "SUM");
		
		Motor_SetSpeed(70);
		Motor_SetDirection(1);
	while(1)
				{	tik =1;
		lcd_gotoxy(&lcd1, 8, 0);
    LCD_puts_interger(&lcd1, cntR);
		lcd_gotoxy(&lcd1, 8, 1);
    LCD_puts_interger(&lcd1, cntB);
		lcd_gotoxy(&lcd1, 8, 2);
    LCD_puts_interger(&lcd1, cntY);
		lcd_gotoxy(&lcd1, 8, 3);
    LCD_puts_interger(&lcd1, SUM);
		if(test==1)
			{
				cntB++;
				Delay_mS(1500);
				test=0;
				}
		else if(test ==2)
			{	
				cntR++;
				Delay_mS(1500);
				test=0;
				}
		else if(test ==3)
				{	
				cntY++;
					Delay_mS(1500);
				test=0;
				}
			
			SUM = cntB + cntY + cntR;
			R = Get_color(RED);
					Delay_mS(100);
			G = Get_color(GREEN);
					Delay_mS(100);
			B = Get_color(BLUE);
					Delay_mS(100);
			for(int i=0; i < 3; i++)
		{
			c=3;
					if(R >= color[i][0] && R <= color[i][1] &&
						 G >= color[i][2] && G <= color[i][3] &&
						 B >= color[i][4] && B <= color[i][5])
							{
								c=i;
								break;
							}	
		}
		if(c==0 || c ==1|| c==2)
			temp=c;
		else 
			{
				Delay_mS(900);
				temp =3;
			}
		
				switch(c)
					{
					case 0: 
							UART_SendString("RED\r\n");
									break;
					case 1: 		
								UART_SendString("BLUE\r\n");
									break;
					case 2: 
						UART_SendString("YELLOW\r\n");
									break;
					case 3: UART_SendString("Others\r\n");
								break;
						}
//					if(!GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1))
//						{
//						if(temp) cntB++;
//							else cntR++;
//						}
						tik=0;
	UART_SendString("R: ");
	UART_SendNumber(R);															//UART_SendNumber(cntR);																//UART_SendNumber(R);
	UART_SendString("\t");
			
	UART_SendString("G: ");																//UART_SendString("G: ");
	UART_SendNumber(G);
	UART_SendString("\t");	
			
	UART_SendString("B: ");
	UART_SendNumber(B);
	UART_SendString("\r\n");
						
	UART_SendString("CntB: ");
	UART_SendNumber(cntB);
	UART_SendString("\r\n");
	UART_SendString("CntR: ");
	UART_SendNumber(cntR);
	UART_SendString("\r\n");
	UART_SendString("CntY: ");
	UART_SendNumber(cntY);
	UART_SendString("\r\n");

	UART_SendString("SUM: ");
	UART_SendNumber(SUM);
	UART_SendString("\r\n");
	UART_SendString("\r\n");

//	Servo1_SetAngle(90);
//	Delay_mS(900);
//	Servo1_SetAngle(0);
	Delay_mS(1000);
				}			
	}
	void GPIO_config_IR_sensor()
	{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // Enable clock for AFIO
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			GPIO_InitTypeDef GPIO_InitStructure;
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // Input Pull-up
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
		
			GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
			EXTI_InitTypeDef EXTI_InitStruct;
			EXTI_InitStruct.EXTI_Line = EXTI_Line1; // Select EXTI Line1
			EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
			EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 
			EXTI_InitStruct.EXTI_LineCmd = ENABLE;
			EXTI_Init(&EXTI_InitStruct);
		
		NVIC_InitTypeDef PIN;
		PIN.NVIC_IRQChannel = EXTI1_IRQn;
		PIN.NVIC_IRQChannelPreemptionPriority = 0;
		PIN.NVIC_IRQChannelSubPriority = 0x00;
		PIN.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&PIN);
		
	}
	void GPIO_config_TCS3200()
	{
		GPIO_type GPIO_TCS3200_S0;
		GPIO_TCS3200_S0.port = PORTB;
		GPIO_TCS3200_S0.pin = 3 ;
		GPIO_TCS3200_S0.mode = Output_mode;
		GPIO_TCS3200_S0.mode_type = Output_GP_PP;	
		GPIO_TCS3200_S0.speed = SPEED_50MHZ;
		port_init( GPIO_TCS3200_S0);
		
		GPIO_type GPIO_TCS3200_S1;
		GPIO_TCS3200_S1.port = PORTB;
		GPIO_TCS3200_S1.pin =  4; 
		GPIO_TCS3200_S1.mode = Output_mode;
		GPIO_TCS3200_S1.mode_type = Output_GP_PP;	
		GPIO_TCS3200_S1.speed = SPEED_50MHZ;
		port_init( GPIO_TCS3200_S1);
		
		GPIO_type GPIO_TCS3200_s2;
		GPIO_TCS3200_s2.port = PORTB;
		GPIO_TCS3200_s2.pin = 0 ;
		GPIO_TCS3200_s2.mode = Output_mode;
		GPIO_TCS3200_s2.mode_type = Output_GP_PP;
		GPIO_TCS3200_s2.speed = SPEED_50MHZ;
		port_init(GPIO_TCS3200_s2);
		
		GPIO_type GPIO_TCS3200_s3;
		GPIO_TCS3200_s3.port = PORTB;
		GPIO_TCS3200_s3.pin = 1 ;
		GPIO_TCS3200_s3.mode = Output_mode;
		GPIO_TCS3200_s3.mode_type = Output_GP_PP;
		GPIO_TCS3200_s3.speed = SPEED_50MHZ;
		port_init(GPIO_TCS3200_s3);
	}

	void I2C_LCD_init()
	{
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	I2C_InitTypeDef I2C_InitStruct;
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_ClockSpeed = 100000;
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	
	I2C_Init(I2C2, &I2C_InitStruct);
	I2C_Cmd(I2C2, ENABLE);
		
		// I2C_TypeDef hi2c2;
	//	I2C_LCD_HandleTypeDef lcd1;
		

	}	
	void Uart_config()
	{
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA,ENABLE);
		GPIO_type GPIO_TX;
		GPIO_TX.port = PORTA;
		GPIO_TX.pin = 9;
		GPIO_TX.mode = Output_mode;
		GPIO_TX.mode_type = Output_ALT_Func_PP;
		GPIO_TX.speed = SPEED_50MHZ;
		port_init(GPIO_TX);
		
		GPIO_type GPIO_RX;
		GPIO_RX.port = PORTA;
		GPIO_RX.pin = 10;
		GPIO_RX.speed = SPEED_50MHZ;
		GPIO_RX.mode = Input_mode;
		GPIO_RX.mode_type = Input_FLOATING;
		port_init(GPIO_RX);
		
		USART_InitTypeDef UART_init;
		UART_init.USART_BaudRate = 9600;
		UART_init.USART_StopBits = USART_StopBits_1;
		UART_init.USART_Parity = USART_Parity_No;
		UART_init.USART_WordLength = USART_WordLength_8b;
		UART_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		UART_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART1,&UART_init);
		USART_Cmd(USART1,ENABLE);
		
	}
	void Servo1_init()
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		GPIO_type GPIO_servo;
		GPIO_servo.port = PORTA;
		GPIO_servo.pin = 8;
		GPIO_servo.mode = Output_mode;
		GPIO_servo.mode_type = Output_ALT_Func_PP;
		GPIO_servo.speed = SPEED_50MHZ;
		port_init(GPIO_servo);
		
		TIM_TimeBaseInitTypeDef TIM_BASE;
		TIM_BASE.TIM_Period = 19999;
		TIM_BASE.TIM_Prescaler = 71;
		TIM_BASE.TIM_ClockDivision = 0;
		TIM_BASE.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM1, &TIM_BASE);
		
		TIM_OCInitTypeDef TIM_PWM;
		TIM_PWM.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_PWM.TIM_OutputState = TIM_OutputState_Enable;
		TIM_PWM.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_PWM.TIM_Pulse =500;
		TIM_OC1Init(TIM1,&TIM_PWM);
		TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
		
		 TIM_ARRPreloadConfig(TIM1, ENABLE);
		TIM_Cmd(TIM1, ENABLE);
		TIM_CtrlPWMOutputs(TIM1, ENABLE);
	}
	void Servo2_init()
	{
		 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			GPIO_InitTypeDef GPIO_InitStruct;
			GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
			GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;   
			GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOB, &GPIO_InitStruct);
		
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		TIM_TimeBaseInitTypeDef SER2;
		SER2.TIM_Period = 19999;
		SER2.TIM_Prescaler = 71;
		SER2.TIM_CounterMode = TIM_CounterMode_Up;
		SER2.TIM_ClockDivision = 0;
		TIM_TimeBaseInit(TIM4,&SER2);
		
		TIM_OCInitTypeDef TIM_PWM;
		TIM_PWM.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_PWM.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_PWM.TIM_Pulse = 2500;
		TIM_PWM.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OC3Init(TIM4, &TIM_PWM);
		TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable);
		
		TIM_ARRPreloadConfig(TIM4, ENABLE);
		TIM_Cmd(TIM4, ENABLE);
	}
	void Motor_init() {
		 
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
			 
			GPIO_type GPIO_IN1;
			GPIO_IN1.port = PORTB;
			GPIO_IN1.mode = Output_mode;
			GPIO_IN1.mode_type = Output_GP_PP;
			GPIO_IN1.speed = SPEED_50MHZ;
			GPIO_IN1.pin = 12;
			port_init(GPIO_IN1);

			GPIO_type GPIO_IN2;
			GPIO_IN2.port = PORTB;
			GPIO_IN2.mode = Output_mode;
			GPIO_IN2.mode_type = Output_GP_PP;
			GPIO_IN2.speed = SPEED_50MHZ;
			GPIO_IN2.pin = 13;
			port_init(GPIO_IN2);

			GPIO_type GPIO_ENA;
			GPIO_ENA.port = PORTA;
			GPIO_ENA.mode = Output_mode;
			GPIO_ENA.mode_type = Output_ALT_Func_PP;
			GPIO_ENA.speed = SPEED_50MHZ;
			GPIO_ENA.pin = 6;
			port_init(GPIO_ENA);

			TIM_TimeBaseInitTypeDef TIM_BASE;
			TIM_BASE.TIM_Period = 1023;             
			TIM_BASE.TIM_Prescaler = 71;           
			TIM_BASE.TIM_ClockDivision = 0;
			TIM_BASE.TIM_CounterMode = TIM_CounterMode_Up;
			TIM_TimeBaseInit(TIM3, &TIM_BASE);

			TIM_OCInitTypeDef TIM_PWM;
			TIM_PWM.TIM_OCMode = TIM_OCMode_PWM1;
			TIM_PWM.TIM_OutputState = TIM_OutputState_Enable;
			TIM_PWM.TIM_OCPolarity = TIM_OCPolarity_High;
			TIM_PWM.TIM_Pulse = 0;                
			TIM_OC1Init(TIM3, &TIM_PWM);           
			TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

		 
			TIM_ARRPreloadConfig(TIM3,ENABLE);    
			TIM_Cmd(TIM3, ENABLE);
	}

	void Motor_SetSpeed(uint8_t speed) {
			if (speed > 100) speed = 100;                  
			uint32_t pulse = (speed * 1023) / 100;         
			TIM_SetCompare1(TIM3, pulse);                 
	}
	void Motor_SetDirection(uint8_t direct)
	{
		switch (direct)
		{
			case 0: 
		port_write(	PORTB,12,HIGH);
		port_write(	PORTB,13,LOW);
		break;
			case 1:
		port_write(	PORTB,12,LOW);
		port_write(	PORTB,13,HIGH);
		break;
			case 3:
		port_write(	PORTB,12,LOW);
		port_write(	PORTB,13,LOW);
			break;
	}
	}

	void Servo1_SetAngle(uint32_t angle)
	{
		uint32_t pulse = 500 + ((angle*2000)/180);
		TIM_SetCompare1(TIM1, pulse);
	}
	void Servo2_SetAngle(uint32_t angle)
	{
		uint32_t pulse = 500 +	((angle*2000)/180);
		TIM_SetCompare3(TIM4, pulse);
	}
	void UART_SendString(const char* str)
	{
		while(*str)
		{
			while (!USART_GetFlagStatus(USART1, USART_FLAG_TXE));
			USART_SendData(USART1, *str++);
			 while (!USART_GetFlagStatus(USART1, USART_FLAG_TC)); 
		}
	}
	void UART_SendNumber(uint32_t num)
	{
			char buffer[20];
			sprintf(buffer, "%lu", (unsigned long)num);  
			UART_SendString(buffer);           
	}

	void EXTI1_IRQHandler() {
			if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
				if(temp ==1)
				{	
					if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1))
					{
						for(int i; i<1000000;i++);
						Servo1_SetAngle(0); //Servo2_SetAngle(90);
						test =1;
					}
				else
				Servo1_SetAngle(90); //Servo2_SetAngle(180);
				}
			 else if(temp==0)
	 		{
				if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1))
					{
					for(int i; i<1000000;i++);
						test =2;
					Servo2_SetAngle(180); //Servo2_SetAngle(90);
					}
				else
					Servo2_SetAngle(90); //Servo2_SetAngle(180);
			} else if(temp ==2) test =3;
				}
				EXTI_ClearITPendingBit(EXTI_Line1);
	
}																																						


	void init_Color()
	{
		color[0][0] =	0;// RED
		color[0][1] = 4;
		
		color[0][2] = 4; //8
		color[0][3] = 23;
		
		color[0][4] = 0;
		color[0][5] = 4;
		
		//----------------------------------//
		color[1][0] = 0; // BLUE
		color[1][1] = 0;
		
		color[1][2] = 20; //20
		color[1][3] =	90;//79
		
		color[1][4] = 6; //10
		color[1][5] = 100;
		//----------------------------------//
		color[2][0] = 0;	// YELLOW
		color[2][1] =	7;
		
		color[2][2] = 17;
		color[2][3] = 75;
		
		color[2][4] = 0;
		color[2][5] = 13;
	}

