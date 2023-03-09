#include "FSA.h"
#include "lcd.h"
#include "stdio.h"
#include "math.h"

const uint8_t States_Table[4][2] =
{
	{1,2},
	{2,3},
	{3,0},
	{0,0}
};
const uint8_t Output_Table[4][2] =
{
	{0,0},
	{0,0},
	{0,1},
	{1,2}
};

const uint8_t balance[4] = {0,5,10,15};

void vend_mach(uint8_t input)
{
	LCD_Clear();
	static uint8_t state_current;
	uint8_t state_previous = state_current;
	uint8_t signal;
	uint8_t output;
	char balance_str[30];
	if(input == 5)
		signal = 0;
	else if(input == 10)
		signal = 1;
	state_current = States_Table[state_previous][signal];
	output = Output_Table[state_previous][signal];
	switch(output)
	{
		case 0:
			LCD_Print_String("Не хватает денег");
		  LCD_Set_Cursor(2,0);
	    sprintf(balance_str,"  Баланс: %dр.",balance[state_current]);
		  LCD_Print_String(balance_str);
		  LCD_Set_Cursor(1,0);
			break;
		case 1:
			LCD_Print_String("Возьмите шокола-");
		  LCD_Set_Cursor(2,0);
		  LCD_Print_String("дку");
		  LCD_Set_Cursor(1,0);
		  delay(500);
			LCD_Print_String("Торговый автомат");
	    LCD_Set_Cursor(2,0);
	    LCD_Print_String(" Цена шок. 20р.");
	    LCD_Set_Cursor(1,0);
			break;
		case 2:
			LCD_Print_String("Возьмите шокола-");
		  LCD_Set_Cursor(2,0);
		  LCD_Print_String("дку и сдачу 5р.");
		  LCD_Set_Cursor(1,0);
		  delay(500);
			LCD_Print_String("Торговый автомат");
	    LCD_Set_Cursor(2,0);
	    LCD_Print_String(" Цена шок. 20р.");
	    LCD_Set_Cursor(1,0);
			break;
	}
}

void write_to_ringbuffer(struct ring_buffer *usart_recieve)
{
	if(usart_recieve->head == RB_ELEMENTS)
		usart_recieve->head = 0;
	usart_recieve->arr[usart_recieve->head++] = USART2->RDR;
}

void print_to_USART(char* str)
{
	while(*str != '\0')
	{
		while((USART2->ISR & USART_ISR_TC) == 0) {}
		USART2->TDR = (uint8_t)*str;
		str++;
	}
	while((USART2->ISR & USART_ISR_TC) == 0) {}
		USART2->TDR = 0x0A;
}

void clear_array(char* array)
{
	while(*array != '\0')
		*array++ = '\0';
}

int string_to_int(char *str)
{
	int i = -1;
	int number;
	char *buffer = str;
	while(*buffer != '\0')
	{
		buffer++;
		i++;
	}
	while(i >= 0)
	{
		number += (*str - '0')*pow(10,i);
		i--;
		str++;
	}
	return number;
}

int read_from_ringbuffer(char* array, struct ring_buffer* usart_recieve)
{
	char *buffer = array;
	int value;
	if(usart_recieve->arr[usart_recieve->head - 1] == READY)
	{
		while(usart_recieve->head != usart_recieve->tail)
		{
			if(((usart_recieve->head > usart_recieve->tail)&&((usart_recieve->head - usart_recieve->tail) > ARRAY_ELEMENTS)) || 
				 ((usart_recieve->head < usart_recieve->tail)&&((usart_recieve->head + (RB_ELEMENTS - usart_recieve->tail)) > ARRAY_ELEMENTS)))                                                                                    
			{
				print_to_USART("Reach arrays's limit");
				usart_recieve->arr[usart_recieve->head - 1] = ERROR; 
				usart_recieve->tail = usart_recieve->head;
				return 1;
			}
			if(usart_recieve->tail == RB_ELEMENTS)
			{
				*array = usart_recieve->arr[usart_recieve->tail];
				usart_recieve->tail = 0;
			}
			if(usart_recieve->head == usart_recieve->tail)
				break;
			*array++ = usart_recieve->arr[usart_recieve->tail++];
		}
		usart_recieve->arr[usart_recieve->tail - 1] = DONE;
		*(array - 1) = '\0';
		print_to_USART(buffer);
		value = string_to_int(buffer);
		if(value == 5 || value == 10)
			vend_mach(value);
		clear_array(buffer);
	}	 
	return 0;
}










