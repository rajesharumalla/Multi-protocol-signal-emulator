#ifndef LCD_I2C_H
#define LCD_I2C_H

#include "stm32f4xx_hal.h"

void LCD_Init(void);
void LCD_Clear(void);
void LCD_Set_Cursor(uint8_t row, uint8_t col);
void LCD_Send_String(char *str);
void LCD_Send_Char(char data);
void LCD_Create_Char(uint8_t location, uint8_t *charmap);

#endif
