#ifndef TCS3200
#define TCS3200
#include <stdint.h>
#include <stdbool.h>
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO


#define MIN_RED     10000 		
#define MAX_RED     500000
#define MIN_GREEN   10000
#define MAX_GREEN   500000
#define MIN_BLUE 		10000	
#define MAX_BLUE 		500000	

enum color {RED, BLUE, CLEAR ,GREEN};
enum scaling{Scl0, Scl2, Scl20 ,Scl100 };

void TCS3200_init(void);
void TCS3200_config(void);
void Set_filter(uint8_t mode);
void Set_scaling(uint8_t mode);
int Get_color(int set_color);
void TIM3_IRQHandler(void);


#endif

