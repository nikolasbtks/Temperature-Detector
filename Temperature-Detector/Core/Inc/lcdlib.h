#ifndef INC_LCDLIB_H_
#define INC_LCDLIB_H_

#include "stm32f4xx_hal.h"

void LCD_Command_Send(uint8_t command);
void LCD_Initialize(void);
void LCD_Data_Send(uint8_t data);
void LCD_String_Send(char *str);
void LCD_Cursor_Set(uint8_t column, uint8_t row);

#endif
