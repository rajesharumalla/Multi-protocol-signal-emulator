#include "lcd_i2c.h"
#include "i2c.h"

extern I2C_HandleTypeDef hi2c1;

#define LCD_ADDR (0x27 << 1)

#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE    0x04
#define LCD_RS        0x01

static void LCD_SendInternal(uint8_t data, uint8_t flags)
{
    uint8_t high = (data & 0xF0) | flags | LCD_BACKLIGHT;
    uint8_t low  = ((data << 4) & 0xF0) | flags | LCD_BACKLIGHT;

    uint8_t data_arr[4] = {
        high | LCD_ENABLE,
        high,
        low | LCD_ENABLE,
        low
    };

    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data_arr, 4, 100);
}

void LCD_Send_Command(uint8_t cmd)
{
    LCD_SendInternal(cmd, 0x00);
    HAL_Delay(2);
}

void LCD_Send_Char(char ch)
{
    LCD_SendInternal(ch, LCD_RS);
}

void LCD_Send_String(char *str)
{
    while(*str) LCD_Send_Char(*str++);
}

void LCD_Clear(void)
{
    LCD_Send_Command(0x01);
    HAL_Delay(2);
}

void LCD_Set_Cursor(uint8_t row, uint8_t col)
{
    uint8_t addr = (row == 0) ? 0x80 : 0xC0;
    LCD_Send_Command(addr + col);
}

void LCD_Init(void)
{
    HAL_Delay(50);

    LCD_Send_Command(0x30);
    HAL_Delay(5);
    LCD_Send_Command(0x30);
    HAL_Delay(1);
    LCD_Send_Command(0x30);
    HAL_Delay(10);

    LCD_Send_Command(0x20);
    HAL_Delay(10);

    LCD_Send_Command(0x28);
    LCD_Send_Command(0x08);
    LCD_Send_Command(0x01);
    HAL_Delay(5);
    LCD_Send_Command(0x06);
    LCD_Send_Command(0x0C);
}

void LCD_Create_Char(uint8_t location, uint8_t *charmap)
{
    location &= 0x07;
    LCD_Send_Command(0x40 | (location << 3));
    for (uint8_t i = 0; i < 8; i++) {
        LCD_Send_Char(charmap[i]);
    }
    LCD_Send_Command(0x80);
}
