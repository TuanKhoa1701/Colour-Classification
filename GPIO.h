#ifndef GPIO
#define GPIO
#include "stm32f10x.h"                  // Device header
// define PORT names
#define PORTA GPIOA
#define PORTB GPIOB
#define PORTC GPIOC
#define PORTD GPIOD
#define PORTE GPIOE
#define PORTF GPIOF
#define PORTG GPIOG

#define HIGH 1
#define LOW 0

// define PIN MODE
#define Input_mode 							((uint32_t) 0x01)
#define Output_mode 						((uint32_t) 0x02)
// Input mode types
#define Input_Analog 						((uint32_t)0x00)
#define Input_FLOATING					((uint32_t)0x01) // default at reset
#define Input_PU_PD							((uint32_t)0x02)	//  input with pull up or pull down
// Output mode 	types

#define Output_GP_PP 										((uint32_t)0x00)	// generous purpose
#define Output_GP_OD										((uint32_t)0x01) // open-drain
#define Output_ALT_Func_PP							((uint32_t)0x02)	// push pull
#define Output_ALT_FUNC_OD							((uint32_t)0x03)

// PIN SPEED
#define SPEED_2MHZ ((uint32_t)0x02)
#define SPEED_10MHZ ((uint32_t)0x01)
#define SPEED_50MHZ ((uint32_t)0x03)

// clock enabling
#define GPIO_Clock_Enable_Alt_Func (RCC->APB2ENR |= (1<<0))
#define GPIO_Clock_Enable_PORT_A (RCC->APB2ENR |= (1<<2))
#define GPIO_Clock_Enable_PORT_B (RCC->APB2ENR |= (1<<3))
#define GPIO_Clock_Enable_PORT_C (RCC->APB2ENR |= (1<<4))
#define GPIO_Clock_Enable_PORT_D (RCC->APB2ENR |= (1<<5))
// #define GPIO_Clock_Enable_PORT_E (RCC->APB2ENR |= (1<<6))

#define MODE_POS_BIT1(pinNumber)       (pinNumber<8) ? (pinNumber*4) : ((pinNumber-8)*4)
#define MODE_POS_BIT2(pinNumber)       (pinNumber<8) ? ((pinNumber*4)+1) : (((pinNumber-8)*4)+1)
#define CNF_POS_BIT1(pinNumber)       (pinNumber<8) ? ((pinNumber*4)+2) : (((pinNumber-8)*4)+2)
#define CNF_POS_BIT2(pinNumber)       (pinNumber<8) ? ((pinNumber*4)+3) : (((pinNumber-8)*4)+3) 
// configuration structio n
typedef struct{
	GPIO_TypeDef *port;
	uint32_t pin;
	uint32_t  mode;
	uint32_t mode_type;
	uint32_t pull;
	uint32_t speed;
	
}GPIO_type;
typedef enum
	{
		RISING_EDGE ,
		FALLING_EDGE,
		RISING_FALLING_EDGE
	} edge_select;
// Function_prototype
//******************************************************//
// GPIO CONFIGURATION
static void config_pin (GPIO_TypeDef*PORT, uint32_t pinNumber, uint32_t	mode_type	);
static void config_pin_speed (GPIO_TypeDef*PORT, uint32_t pinNumber, uint32_t pinSpeed,uint32_t	mode);
// GPIO USER PIN FUCTION
void port_write(GPIO_TypeDef*PORT, uint32_t pinNumber, uint32_t state);
void port_toggle(GPIO_TypeDef*PORT, uint32_t pinNumber);
void port_init(GPIO_type GPIO_type);
// INTERRUPT FUNCTION
void configure_port_interrupt(GPIO_TypeDef *PORT, uint32_t pinNumber, edge_select edge);
void enable_port_interrupt(uint32_t pinNumber,IRQn_Type IrqNumber);
void clear_port_interrupt(uint32_t pinNumber);
	//----------------DEEPSLEEP/STANDBY MODE---------------//
void checkStandbyMode(void);
void gotosleep(void);
#endif
