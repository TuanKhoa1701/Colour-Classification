#include "stm32f10x.h"                  // Device header
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_i2c.h"              // Keil::Device:StdPeriph Drivers:I2C
#include "LCD.h"
#include <stdio.h>
/*
#define
uint8_t I2C_Receive1Byte(){
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == 1);
	uint8_t temp = I2C_ReceiveData(I2C1);
	return temp;
}
*/
void RCC_config()
{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);

}

void GPIO_config()
{
		GPIO_InitTypeDef GPIOC_LED;
		GPIOC_LED.GPIO_Mode= GPIO_Mode_Out_PP;
		GPIOC_LED.GPIO_Pin = GPIO_Pin_13;
		GPIOC_LED.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOC,&GPIOC_LED);
}
int main(){
	RCC_config();
	GPIO_config();
/*
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
	
	I2C_TypeDef hi2c2;
	I2C_LCD_HandleTypeDef lcd1;
	
	Systick_init();
		lcd1.hi2c = &hi2c2;
    lcd1.address = 0x4E;
    lcd_init(&lcd1);                                                                
    lcd_clear(&lcd1);
		
		lcd_gotoxy(&lcd1, 0, 0);
    lcd_puts(&lcd1, "CountRED");
		lcd_gotoxy(&lcd1, 8, 0);
    lcd_puts(&lcd1, "1");
		lcd_gotoxy(&lcd1, 0, 1);
		lcd_puts(&lcd1, "CountY");
		lcd_gotoxy(&lcd1, 8, 1);
		lcd_puts(&lcd1, "1");
		lcd_gotoxy(&lcd1, 0, 2);
		lcd_puts(&lcd1, "CountB");
		lcd_gotoxy(&lcd1, 8, 2);
		lcd_puts(&lcd1, "1");
		lcd_gotoxy(&lcd1, 0, 3);
		lcd_puts(&lcd1, "SUM");
		lcd_gotoxy(&lcd1, 8, 3);
		lcd_puts(&lcd1, "3");
		*/
	while(1)
		{		
			GPIO_SetBits(GPIOC, GPIO_Pin_13);
			Delay_mS(1000);
			GPIO_ResetBits(GPIOC, GPIO_Pin_13);
			Delay_mS(1000);
			
		}
}
