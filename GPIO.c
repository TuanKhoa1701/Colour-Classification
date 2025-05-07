#include "stm32f10x.h"                  // Device header
#include <stdint.h>
#include "GPIO.h"
static void config_pin (GPIO_TypeDef*PORT, uint32_t pinNumber, uint32_t	mode_type	)
{
		if(pinNumber >= 8)
	{
		//-------------In&Output Modes----------------//
		switch(mode_type)
			{
			case Output_GP_PP | Input_Analog :
				PORT->CRH &= ~( (1 << (CNF_POS_BIT1(pinNumber))) | (1<< (CNF_POS_BIT2(pinNumber))));
			break;
			case Output_GP_OD | Input_FLOATING	:
				PORT->CRH &= ~(1<< (CNF_POS_BIT2(pinNumber)));
				PORT->CRH |= (1<<(CNF_POS_BIT1(pinNumber)));
			break;
			case Output_ALT_Func_PP	 | Input_PU_PD	:
				PORT->CRH |= (1<< (CNF_POS_BIT2(pinNumber)));
				PORT->CRH &= ~(1<<(CNF_POS_BIT1(pinNumber)));
			break;
			case Output_ALT_FUNC_OD:
				PORT->CRH |= ( (1 << (CNF_POS_BIT1(pinNumber))) | (1<< (CNF_POS_BIT2(pinNumber))));
			break;
			} //endswitch
	}
	else
	{
		//-------------In&Output Modes----------------//
		switch(mode_type)
			{
			case Output_GP_PP | Input_Analog :
				PORT->CRL &= ~( (1 << (CNF_POS_BIT1(pinNumber))) | (1<< (CNF_POS_BIT2(pinNumber))));
			break;
			case Output_GP_OD | Input_FLOATING	:
				PORT->CRL &= ~(1<< (CNF_POS_BIT2(pinNumber)));
				PORT->CRL |= (1<<(CNF_POS_BIT1(pinNumber)));
			break;
			case Output_ALT_Func_PP	 | Input_PU_PD	:
				PORT->CRL |= (1<< (CNF_POS_BIT2(pinNumber)));
				PORT->CRL &= ~(1<<(CNF_POS_BIT1(pinNumber)));
			break;
			case Output_ALT_FUNC_OD:
				PORT->CRL |= ( (1 << (CNF_POS_BIT1(pinNumber))) | (1<< (CNF_POS_BIT2(pinNumber))));
			break;
			} //endswitch
	}
}
static void config_pin_speed (GPIO_TypeDef*PORT, uint32_t pinNumber, uint32_t pinSpeed,uint32_t	mode)
{


	if(pinNumber >=8)
	{
		if(mode == Input_mode)
		{
			PORT->CRH &= ~(1 << (MODE_POS_BIT1(pinNumber)) | 1 <<(MODE_POS_BIT2(pinNumber)));
		}
		else
		{
			PORT->CRH |= (pinSpeed << (MODE_POS_BIT1(pinNumber)));
		}
	}
	else
	{
		if(mode == Input_mode)
		{
			PORT->CRL &= ~(1 << (MODE_POS_BIT1(pinNumber)) | 1 <<(MODE_POS_BIT2(pinNumber)));
		}
		else
		{
			PORT->CRL |= (pinSpeed << (MODE_POS_BIT1(pinNumber)));
		}
	}
}
void port_write(GPIO_TypeDef*PORT, uint32_t pinNumber, uint32_t state)
{
	if(state)
		{
			PORT->BSRR= (1<< pinNumber);
		}
	else
		{
			PORT->BSRR= (1<< (pinNumber+16));
		}
}
void port_toggle(GPIO_TypeDef*PORT, uint32_t pinNumber)
{
	PORT->ODR ^= (1<< pinNumber);
}
void port_init(GPIO_type GPIO_type)
{
		if(GPIO_type.port == PORTA)
			GPIO_Clock_Enable_PORT_A ;
		if(GPIO_type.port == PORTB)
			GPIO_Clock_Enable_PORT_B ;
		if(GPIO_type.port == PORTC)
			GPIO_Clock_Enable_PORT_C ;
		if(GPIO_type.port == PORTD)
			GPIO_Clock_Enable_PORT_D ;
		config_pin (GPIO_type.port, GPIO_type.pin, GPIO_type.mode_type);
		config_pin_speed (GPIO_type.port, GPIO_type.pin, GPIO_type.speed,GPIO_type.mode);
}
//-------------------------INTERRUPT FUNCTIONS----------------------//
void configure_port_interrupt(GPIO_TypeDef *PORT, uint32_t pinNumber, edge_select edge)
{
	RCC->APB2ENR |= 1<<0;
	if(PORT == PORTA)
	{
		switch(pinNumber)
		{
			case 0:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PA;
			break;
			case 1:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PA;
			break;
			case 2:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI2_PA;
			break;
			case 3:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI3_PA;
			break;
			case 4:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI4_PA;
			break;
			case 5:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI5_PA;
			break;
			case 6:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI6_PA;
			break;
			case 7:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI7_PA;
			break;
			case 8:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI8_PA;
			break;
			case 9:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI9_PA;
			break;
			case 10:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI10_PA;
			break;
			case 11:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI11_PA;
			break;
			case 12:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI12_PA;
			break;
			case 13:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI13_PA;
			break;
			case 14:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI14_PA;
			break;
			case 15:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI15_PA;
			break;
		}
	}
	if(PORT == PORTB)
	{
		switch(pinNumber)
		{
			case 0:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PB;
			break;
			case 1:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PB;
			break;
			case 2:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI2_PB;
			break;
			case 3:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI3_PB;
			break;
			case 4:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI4_PB;
			break;
			case 5:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI5_PB;
			break;
			case 6:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI6_PB;
			break;
			case 7:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI7_PB;
			break;
			case 8:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI8_PB;
			break;
			case 9:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI9_PB;
			break;
			case 10:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI10_PB;
			break;
			case 11:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI11_PB;
			break;
			case 12:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI12_PB;
			break;
			case 13:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI13_PB;
			break;
			case 14:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI14_PB;
			break;
			case 15:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI15_PB;
			break;
		}
	}
	if(PORT == PORTC)
	{
		switch(pinNumber)
		{
			case 0:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PC;
			break;
			case 1:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PC;
			break;
			case 2:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI2_PC;
			break;
			case 3:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI3_PC;
			break;
			case 4:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI4_PC;
			break;
			case 5:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI5_PC;
			break;
			case 6:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI6_PC;
			break;
			case 7:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI7_PC;
			break;
			case 8:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI8_PC;
			break;
			case 9:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI9_PC;
			break;
			case 10:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI10_PC;
			break;
			case 11:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI11_PC;
			break;
			case 12:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI12_PC;
			break;
			case 13:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI13_PC;
			break;
			case 14:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI14_PC;
			break;
			case 15:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI15_PC;
			break;
		}
	}
	if(PORT == PORTD)
	{
		switch(pinNumber)
		{
			case 0:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PD;
			break;
			case 1:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PD;
			break;
			case 2:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI2_PD;
			break;
			case 3:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI3_PD;
			break;
			case 4:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI4_PD;
			break;
			case 5:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI5_PD;
			break;
			case 6:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI6_PD;
			break;
			case 7:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI7_PD;
			break;
			case 8:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI8_PD;
			break;
			case 9:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI9_PD;
			break;
			case 10:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI10_PD;
			break;
			case 11:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI11_PD;
			break;
			case 12:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI12_PD;
			break;
			case 13:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI13_PD;
			break;
			case 14:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI14_PD;
			break;
			case 15:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI15_PD;
			break;
		}
	}
	if(PORT == PORTE)
	{
		switch(pinNumber)
		{
			case 0:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PE;
			break;
			case 1:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PE;
			break;
			case 2:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI2_PE;
			break;
			case 3:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI3_PE;
			break;
			case 4:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI4_PE;
			break;
			case 5:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI5_PE;
			break;
			case 6:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI6_PE;
			break;
			case 7:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI7_PE;
			break;
			case 8:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI8_PE;
			break;
			case 9:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI9_PE;
			break;
			case 10:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI10_PE;
			break;
			case 11:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI11_PE;
			break;
			case 12:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI12_PE;
			break;
			case 13:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI13_PE;
			break;
			case 14:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI14_PE;
			break;
			case 15:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI15_PE;
			break;
		}
	}
	if(PORT == PORTF)
	{
		switch(pinNumber)
		{
			case 0:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PF;
			break;
			case 1:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PF;
			break;
			case 2:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI2_PF;
			break;
			case 3:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI3_PF;
			break;
			case 4:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI4_PF;
			break;
			case 5:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI5_PF;
			break;
			case 6:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI6_PF;
			break;
			case 7:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI7_PF;
			break;
			case 8:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI8_PF;
			break;
			case 9:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI9_PF;
			break;
			case 10:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI10_PF;
			break;
			case 11:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI11_PF;
			break;
			case 12:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI12_PF;
			break;
			case 13:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI13_PF;
			break;
			case 14:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI14_PF;
			break;
			case 15:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI15_PF;
			break;
		}
	}
	if(PORT == PORTG)
	{
		switch(pinNumber)
		{
			case 0:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PG;
			break;
			case 1:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PG;
			break;
			case 2:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI2_PG;
			break;
			case 3:
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI3_PG;
			break;
			case 4:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI4_PG;
			break;
			case 5:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI5_PG;
			break;
			case 6:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI6_PG;
			break;
			case 7:
				AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI7_PG;
			break;
			case 8:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI8_PG;
			break;
			case 9:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI9_PG;
			break;
			case 10:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI10_PG;
			break;
			case 11:
				AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI11_PG;
			break;
			case 12:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI12_PG;
			break;
			case 13:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI13_PG;
			break;
			case 14:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI14_PG;
			break;
			case 15:
				AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI15_PG;
			break;
		}
	}
	
	if(edge == RISING_EDGE)
	{
		EXTI ->RTSR |= 1<< pinNumber;
	}
	if(edge == FALLING_EDGE )
	{
		EXTI -> FTSR |= 1<<pinNumber;
	}
	if(edge ==RISING_FALLING_EDGE)
	{
		EXTI ->FTSR|= 1<< pinNumber;
		EXTI ->RTSR|= 1<< pinNumber;
	}
}
void enable_port_interrupt(uint32_t pinNumber,IRQn_Type IrqNumber)
{
	EXTI ->IMR |= 1 << pinNumber;
	NVIC_EnableIRQ(IrqNumber);
}
void clear_port_interrupt(uint32_t pinNumber)
{
	
		EXTI -> PR |= 1<< pinNumber;
}	
//----------------DEEPSLEEP/STANDBY MODE---------------//
void checkStandbyMode()
{
	if((PWR->CSR)&(PWR_CSR_SBF))// kiem tra xem thuc day boi power lost hay stanby mode, standby thi thuc hien
	{	
		PWR -> CR |= PWR_CR_CWUF;
		PWR -> CR |= PWR_CR_CSBF;
	}
}
void gotosleep()
{ 
	// enable pmw control clock
	RCC ->APB1ENR |= (RCC_APB1ENR_PWREN);
	// set SLEEPDEEP bit of Cortex system control register
	SCB -> SCR |= SCB_SCR_SLEEPDEEP_Msk;
	// select standby mode
	PWR -> CR |= PWR_CR_PDDS;
	// clear wake up flag
	PWR -> CR |= PWR_CR_CWUF;
	// enable wake up pin
	PWR -> CSR |= PWR_CSR_EWUP;
	// Rquest WFI
	__WFI();
}

