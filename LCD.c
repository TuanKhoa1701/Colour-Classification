#include "LCD.h"
#include "stm32f10x_i2c.h"
#include <stdio.h>
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO

#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
uint16_t msTicks;
/**
 * @brief  Sends data over I2C using SPL.
 * @param  lcd: Pointer to the LCD handle
 * @param  data: Data buffer to send
 * @param  length: Length of data buffer
 * @retval None
 */
void I2C_SendData_Split(I2C_LCD_HandleTypeDef *lcd, uint8_t *data, uint8_t length)
{
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));

    I2C_GenerateSTART(I2C2, ENABLE);
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C2, lcd->address, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    for (int i = 0; i < length; i++)
    {
        I2C_SendData(I2C2, data[i]);
        while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    }

    I2C_GenerateSTOP(I2C2, ENABLE);
}

/**
 * @brief  Sends a command to the LCD.
 * @param  lcd: Pointer to the LCD handle
 * @param  cmd: Command byte to send
 * @retval None
 */
void lcd_send_cmd(I2C_LCD_HandleTypeDef *lcd, char cmd)
{
    char upper_nibble, lower_nibble;
    uint8_t data_t[4];

    upper_nibble = (cmd & 0xF0);
    lower_nibble = ((cmd << 4) & 0xF0);

    data_t[0] = upper_nibble | 0x0C; // en=1, rs=0
    data_t[1] = upper_nibble | 0x08; // en=0, rs=0
    data_t[2] = lower_nibble | 0x0C; // en=1, rs=0
    data_t[3] = lower_nibble | 0x08; // en=0, rs=0

    I2C_SendData_Split(lcd, data_t, 4);
}

/**
 * @brief  Sends data (character) to the LCD.
 * @param  lcd: Pointer to the LCD handle
 * @param  data: Data byte to send
 * @retval None
 */
void lcd_send_data(I2C_LCD_HandleTypeDef *lcd, char data)
{
    char upper_nibble, lower_nibble;
    uint8_t data_t[4];

    upper_nibble = (data & 0xF0);
    lower_nibble = ((data << 4) & 0xF0);

    data_t[0] = upper_nibble | 0x0D; // en=1, rs=1
    data_t[1] = upper_nibble | 0x09; // en=0, rs=1
    data_t[2] = lower_nibble | 0x0D; // en=1, rs=1
    data_t[3] = lower_nibble | 0x09; // en=0, rs=1

    I2C_SendData_Split(lcd, data_t, 4);
}

/**
 * @brief  Initializes the LCD in 4-bit mode.
 * @param  lcd: Pointer to the LCD handle
 * @retval None
 */
void lcd_init(I2C_LCD_HandleTypeDef *lcd)
{
   
	Systick_init();
    // LCD-specific initialization
    Delay_mS(50);
    lcd_send_cmd(lcd, 0x30);
    Delay_mS(5);
    lcd_send_cmd(lcd, 0x30);
    Delay_mS(1);
    lcd_send_cmd(lcd, 0x30);
    Delay_mS(10);
    lcd_send_cmd(lcd, 0x20);
    Delay_mS(10);

    lcd_send_cmd(lcd, 0x28);
    Delay_mS(1);
    lcd_send_cmd(lcd, 0x08);
    Delay_mS(1);
    lcd_send_cmd(lcd, 0x01);
    Delay_mS(2);
    lcd_send_cmd(lcd, 0x06);
    Delay_mS(1);
    lcd_send_cmd(lcd, 0x0C);
}
void lcd_clear(I2C_LCD_HandleTypeDef *lcd)
{
    lcd_send_cmd(lcd, 0x80);  // Move cursor to the home position
    for (int i = 0; i < 70; i++)
    {
        lcd_send_data(lcd, ' ');  // Write          a space on each position
    }
}

/**
 * @brief  Moves the cursor to a specific position on the LCD.
 * @param  lcd: Pointer to the LCD handle
 * @param  col: Column number (0-15)
 * @param  row: Row number (0 or 1)
 * @retval None
 */
void lcd_gotoxy(I2C_LCD_HandleTypeDef *lcd, int col, int row)
{
    switch (row)
    {
    case 0:
        col |= 0x80;  // 1st row offset
        break;
    case 1:
        col |= 0xC0;  // 2nd row offset
        break;
		 case 2:
        col |= 0x94;  // 3rd row offset
        break;
    case 3:
        col |= 0xD4;  // 4th row offset
        break;
    default:
        return;       // Invalid row
    }

    lcd_send_cmd(lcd, col);  // Send command to move the cursor
}
void lcd_puts(I2C_LCD_HandleTypeDef *lcd, char *str)
{
    while (*str) lcd_send_data(lcd, *str++);  // Send each character in the string
}

/**
 * @brief  Sends a single character to the LCD.
 * @param  lcd: Pointer to the LCD handle
 * @param  ch: Character to send
 * @retval None
 */
void lcd_putchar(I2C_LCD_HandleTypeDef *lcd, char ch)
{
    lcd_send_data(lcd, ch);  // Send the character to the display
}
void LCD_puts_interger(I2C_LCD_HandleTypeDef *lcd, uint32_t data)
	{
		char buffer[16];
		 sprintf(buffer, "%lu", (unsigned long)data);
		lcd_puts(lcd,buffer);
		}
void Systick_init()
{
	SysTick_Config(SystemCoreClock / 1000);
}
void SysTick_Handler()
{
	msTicks++;
}

void Delay_mS(uint32_t	mS)
{
	uint32_t startTick = msTicks;
    while ((msTicks - startTick) < mS	);
}