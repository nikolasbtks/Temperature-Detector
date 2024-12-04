#include "stdint.h"
#include "lcdlib.h"

#define ADDR 0x27 << 1
extern I2C_HandleTypeDef hi2c1;

void LCD_Command_Send(uint8_t command)
{
	uint8_t data[4];
	uint8_t lower, upper;

	lower = ((command << 4) & 0xF0);
	upper = (command & 0xF0);

	data[0] = upper | 0x0C;
	data[1] = upper | 0x08;
	data[2] = lower | 0x0C;
	data[3] = lower | 0x08;

	HAL_I2C_Master_Transmit(&hi2c1, ADDR, data, 4, HAL_MAX_DELAY);
	HAL_Delay(2);
}

void LCD_Initialize(void)
{
    HAL_Delay(50);

    LCD_Command_Send(0x30);
    HAL_Delay(5);
    LCD_Command_Send(0x30);
    HAL_Delay(1);
    LCD_Command_Send(0x30);
    HAL_Delay(1);

    LCD_Command_Send(0x20);
    HAL_Delay(1);

    LCD_Command_Send(0x28);
    HAL_Delay(1);

    LCD_Command_Send(0x0C);
    HAL_Delay(1);

    LCD_Command_Send(0x01);
    HAL_Delay(2);

    LCD_Command_Send(0x06);
    HAL_Delay(1);
}


void LCD_Data_Send(uint8_t data)
{
	uint8_t data_t[4];
	uint8_t lower, upper;

	lower = ((data << 4) & 0xF0);
	upper = (data & 0xF0);

	data_t[0] = upper | 0x0D;
	data_t[1] = upper | 0x09;
	data_t[2] = lower | 0x0D;
	data_t[3] = lower | 0x09;

	HAL_I2C_Master_Transmit(&hi2c1, ADDR, data_t, 4, HAL_MAX_DELAY);
	HAL_Delay(2);
}

void LCD_String_Send(char *str)
{
	while(*str)
	{
		LCD_Data_Send(*str++);
	}
}

void LCD_Cursor_Set(uint8_t row, uint8_t column)
{
	uint8_t addr;
	switch(row)
	{
		case 0: addr = 0x80 + column; break;
		case 1: addr = 0xC0 + column; break;
		case 2: addr = 0x94 + column; break;
		case 3: addr = 0xD4 + column; break;
		default: addr = 0x80 + column;
	}
	LCD_Command_Send(addr);
}
