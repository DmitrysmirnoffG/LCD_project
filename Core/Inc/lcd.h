#ifndef LCD_H_
#define LCD_H_

#include "stm32l4xx_hal.h"
#include "main.h"

#define DB4_set() 		HAL_GPIO_WritePin(GPIOA, DB4_Pin, GPIO_PIN_SET)
#define DB5_set() 		HAL_GPIO_WritePin(GPIOA, DB5_Pin, GPIO_PIN_SET)
#define DB6_set() 		HAL_GPIO_WritePin(GPIOA, DB6_Pin, GPIO_PIN_SET)
#define DB7_set() 		HAL_GPIO_WritePin(GPIOA, DB7_Pin, GPIO_PIN_SET)
#define DB4_reset() 	HAL_GPIO_WritePin(GPIOA, DB4_Pin, GPIO_PIN_RESET)
#define DB5_reset() 	HAL_GPIO_WritePin(GPIOA, DB5_Pin, GPIO_PIN_RESET)
#define DB6_reset() 	HAL_GPIO_WritePin(GPIOA, DB6_Pin, GPIO_PIN_RESET)
#define DB7_reset() 	HAL_GPIO_WritePin(GPIOA, DB7_Pin, GPIO_PIN_RESET)
#define E1 						HAL_GPIO_WritePin(GPIOC, Enable_Pin, GPIO_PIN_SET)
#define E0 						HAL_GPIO_WritePin(GPIOC, Enable_Pin, GPIO_PIN_RESET)
#define RS1 					HAL_GPIO_WritePin(GPIOA, Register_select_Pin, GPIO_PIN_SET)
#define RS0 					HAL_GPIO_WritePin(GPIOA, Register_select_Pin, GPIO_PIN_RESET)

struct protocol
{
	uint8_t number;
	uint8_t uart_port;
	uint32_t data;
	uint16_t check_sum;
};

void LCD_init(void);
void LCD_WriteData(uint8_t);
void LCD_Command(uint8_t);
void LCD_Print_String(char*);
void LCD_Print(uint8_t);
void LCD_Clear(void);
void LCD_PrintWithBits(char*,uint8_t,uint8_t);
void LCD_Set_Cursor(uint8_t,uint8_t);
void LCD_Cursor_On_Off(_Bool);
void delay(float ms);
void clock(void);
void get_time(void);
void get_date(void);
void random_print(void);
void set_brightness(uint16_t);
void LCD_blink(void);
uint8_t analyze_protocol(struct protocol*,char*);
void print_protocol(struct protocol*);
void random_protocol(void); 
#endif /* LCD_H_  */



