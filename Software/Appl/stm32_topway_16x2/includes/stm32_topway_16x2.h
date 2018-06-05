/*
	Library: 				LCD 16X2 - 
	Written by:  		Mohamed Ashraf
	Date Written:		04/12/2017
	Description:		This is a library for the standard 16X2 LCD display, for the STM32 MCUs based on HAL libraries. 
						It perfroms the basic Text/Number printing to your 16X2 LCD, in 8 bits and 4 bits modes of operation.

*/


//***** Header files *****//
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include <stdlib.h>


//*Mapping LCD configuration ports with module*//
#define D4_GPIO_Port 			LCD_D0_GPIO_Port
#define D4_Pin 					LCD_D0_Pin
#define D5_GPIO_Port 			LCD_D1_GPIO_Port
#define D5_Pin 					LCD_D1_Pin
#define D6_GPIO_Port 			LCD_D2_GPIO_Port
#define D6_Pin 					LCD_D2_Pin
#define D7_GPIO_Port 			LCD_D3_GPIO_Port
#define D7_Pin 					LCD_D3_Pin


//***** List of COMMANDS *****//
#define LCD_CLEARDISPLAY 		0x01
#define LCD_RETURNHOME 			0x02
#define LCD_ENTRYMODESET 		0x04
#define LCD_DISPLAYCONTROL 	0x08
#define LCD_CURSORSHIFT 		0x10
#define LCD_FUNCTIONSET 		0x20
#define LCD_SETCGRAMADDR 		0x40
#define LCD_SETDDRAMADDR 		0x80

//***** List of commands Bitfields *****//
//1) Entry mode Bitfields
#define LCD_ENTRY_SH		 				0x01
#define LCD_ENTRY_ID 						0x02
//2) Entry mode Bitfields
#define LCD_ENTRY_SH		 				0x01
#define LCD_ENTRY_ID 						0x02
//3) Display control
#define LCD_DISPLAY_B						0x01
#define LCD_DISPLAY_C						0x02
#define LCD_DISPLAY_D						0x04
//4) Shift control
#define LCD_SHIFT_RL						0x04
#define LCD_SHIFT_SC						0x08
//5) Function set control
#define LCD_FUNCTION_F					0x04
#define LCD_FUNCTION_N					0x08
#define LCD_FUNCTION_DL					0x10


//Public functions
//1) LCD begin 8 bits function
void LCD1602_Begin8BIT(GPIO_TypeDef* PORT_RS_E, uint16_t RS, uint16_t E, GPIO_TypeDef* PORT_LSBs0to3, uint16_t D0, uint16_t D1, uint16_t D2, uint16_t D3, GPIO_TypeDef* PORT_MSBs4to7, uint16_t D4, uint16_t D5, uint16_t D6, uint16_t D7);
//2) LCD begin 4 bits function
void LCD1602_Begin4BIT(GPIO_TypeDef* PORT_RS_E, uint16_t RS, uint16_t E, GPIO_TypeDef* PORT_MSBs4to7, uint16_t D4, uint16_t D5, uint16_t D6, uint16_t D7);
//3) LCD print string
void LCD1602_print(char string[]);
//4) set cursor position
void LCD1602_setCursor(uint8_t row, uint8_t col);
void LCD1602_1stLine(void);
void LCD1602_2ndLine(void);
//5) Enable two lines
void LCD1602_TwoLines(void);
void LCD1602_OneLine(void);
//6) Cursor ON/OFF
void LCD1602_noCursor(void);
void LCD1602_cursor(void);
//7) Clear display
void LCD1602_clear(void);
//8) Blinking cursor
void LCD1602_noBlink(void);
void LCD1602_blink(void);
//9) Display ON/OFF
void LCD1602_noDisplay(void);
void LCD1602_display(void);
//10) Shift Display, right or left
void LCD1602_shiftToRight(uint8_t num);
void LCD1602_shiftToLeft(uint8_t num);

//********** Print numbers to LCD **********//
//1. Integer
void LCD1602_PrintInt(int number, uint8_t digitnumber);
//2. Float
void LCD1602_PrintFloat(float number, int decimalPoints);
