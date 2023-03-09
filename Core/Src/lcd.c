#include "lcd.h"
#include "stdio.h"
#include "stm32l4xx_hal_rtc.h"
#include "stm32l4xx_hal_rng.h"
#include "stm32l4xx_hal_crc.h"
#include "math.h"
#include "FSA.h"

RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef DateToUpdate = {0};
char trans_str[64];
extern RTC_HandleTypeDef hrtc;
extern RNG_HandleTypeDef hrng;
extern TIM_HandleTypeDef htim2;
extern char buff[30];
extern CRC_HandleTypeDef hcrc;
extern UART_HandleTypeDef huart2;
char data_pixels[8] = {0B11111, 0B10001, 0B10001, 0B10001, 0B10001, 0B10001, 0B10001, 0B11111};
char protocol[14];

void delay(float ms)
{
	uint32_t i;
	int timerEnd = (64000.0 / 0.65 * ms);
	for(i = 0;i < timerEnd; i++)
		__nop();
}

void LCD_init(void)
{
	delay(70);
	LCD_Command(0x02);
  LCD_Command(0x2a);
  delay(1);
  LCD_Command(0x0f);
  delay(1);
  LCD_Command(0x01);
	delay(2);
  LCD_Command(0x06);
  delay(1);
}

void LCD_WriteData(uint8_t data)
{
	if(((data >> 3)&0x01)==1) 
		DB7_set(); 
	else 
		DB7_reset();
	if(((data >> 2)&0x01)==1) 
		DB6_set(); 
	else 
		DB6_reset();
	if(((data >> 1)&0x01)==1) 
		DB5_set();
	else 
		DB5_reset();
	if((data&0x01)==1) 
		DB4_set(); 
	else 
		DB4_reset();
}

void LCD_Command(uint8_t data)
{
	RS0;
	LCD_WriteData(data>>4);
	E1;
	delay(0.5);
	E0;
	LCD_WriteData(data);
  E1;
  delay(0.5);
  E0;
}

void LCD_Print(uint8_t data)
{
	RS1;
	LCD_WriteData(data>>4);
	E1;
	delay(0.5);
	E0;
	LCD_WriteData(data);
  E1;
  delay(0.5);
  E0;
}

void LCD_Print_String(char *ch)
{
	char charTable[66] = {
        0x41,0xa0,0x42,0xa1,0xe0,0x45,0xa2,0xa3,0xa4,0xa5,0xa6,0x4b,0xa7,0x4d,0x48,0x4f,
        0xa8,0x50,0x43,0x54,0xa9,0xaa,0x58,0xe1,0xab,0xac,0xe2,0xad,0xae,0x62,0xaf,0xb0,
        0xb1,
	      0x61,0xb2,0xb3,0xb4,0xe3,0x65,0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xbb,0xbc,0xbd,0x6f,
        0xbe,0x70,0x63,0xbf,0x79,0xe4,0x78,0xe5,0xc0,0xc1,0xe6,0xc2,0xc3,0xc4,0xc5,0xc6,
        0xc7};
	while(*ch != '\0')
	{
		switch((*ch)&0xff)
		{
			case 0xd0: break;	
			case 0xd1: break;
			case 0x90: LCD_Print((uint8_t)charTable[0]); break; //А
			case 0x91: LCD_Print((uint8_t)charTable[1]); break; //Б
			case 0x92: LCD_Print((uint8_t)charTable[2]); break; //В
			case 0x93: LCD_Print((uint8_t)charTable[3]); break; //Г
			case 0x94: LCD_Print((uint8_t)charTable[4]); break; //Д
			case 0x95: LCD_Print((uint8_t)charTable[5]); break; //Е
			//case : LCD_Print((uint8_t)charTable[6]); break; //Ё
			case 0x96: LCD_Print((uint8_t)charTable[7]); break; //Ж
			case 0x97: LCD_Print((uint8_t)charTable[8]); break; //З
			case 0x98: LCD_Print((uint8_t)charTable[9]); break; //И
			case 0x99: LCD_Print((uint8_t)charTable[10]); break; //Й
			case 0x9a: LCD_Print((uint8_t)charTable[11]); break; //К
			case 0x9b: LCD_Print((uint8_t)charTable[12]); break; //Л
			case 0x9c: LCD_Print((uint8_t)charTable[13]); break; //М
			case 0x9d: LCD_Print((uint8_t)charTable[14]); break; //Н
			case 0x9e: LCD_Print((uint8_t)charTable[15]); break; //О
			case 0x9f: LCD_Print((uint8_t)charTable[16]); break; //П
			case 0xa0: LCD_Print((uint8_t)charTable[17]); break; //Р
			case 0xa1: LCD_Print((uint8_t)charTable[18]); break; //С
			case 0xa2: LCD_Print((uint8_t)charTable[19]); break; //Т
			case 0xa3: LCD_Print((uint8_t)charTable[20]); break; //У
			case 0xa4: LCD_Print((uint8_t)charTable[21]); break; //Ф
			case 0xa5: LCD_Print((uint8_t)charTable[22]); break; //Х
			case 0xa6: LCD_Print((uint8_t)charTable[23]); break; //Ц
			case 0xa7: LCD_Print((uint8_t)charTable[24]); break; //Ч
			case 0xa8: LCD_Print((uint8_t)charTable[25]); break; //Ш
			case 0xa9: LCD_Print((uint8_t)charTable[26]); break; //Щ
			case 0xaa: LCD_Print((uint8_t)charTable[27]); break; //Ъ
			case 0xab: LCD_Print((uint8_t)charTable[28]); break; //Ы
			case 0xac: LCD_Print((uint8_t)charTable[29]); break; //Ь
			case 0xad: LCD_Print((uint8_t)charTable[30]); break; //Э
			case 0xae: LCD_Print((uint8_t)charTable[31]); break; //Ю
			case 0xaf: LCD_Print((uint8_t)charTable[32]); break; //Я
			
			case 0xb0: LCD_Print((uint8_t)charTable[33]); break; //а
			case 0xb1: LCD_Print((uint8_t)charTable[34]); break; //б
			case 0xb2: LCD_Print((uint8_t)charTable[35]); break; //в
			case 0xb3: LCD_Print((uint8_t)charTable[36]); break; //г
			case 0xb4: LCD_Print((uint8_t)charTable[37]); break; //д
			case 0xb5: LCD_Print((uint8_t)charTable[38]); break; //е
			//case : LCD_Print((uint8_t)charTable[39]); break; //ё
			case 0xb6: LCD_Print((uint8_t)charTable[40]); break; //ж
			case 0xb7: LCD_Print((uint8_t)charTable[41]); break; //з
			case 0xb8: LCD_Print((uint8_t)charTable[42]); break; //и
			case 0xb9: LCD_Print((uint8_t)charTable[43]); break; //й
			case 0xba: LCD_Print((uint8_t)charTable[44]); break; //к
			case 0xbb: LCD_Print((uint8_t)charTable[45]); break; //л
			case 0xbc: LCD_Print((uint8_t)charTable[46]); break; //м
			case 0xbd: LCD_Print((uint8_t)charTable[47]); break; //н
			case 0xbe: LCD_Print((uint8_t)charTable[48]); break; //о
			case 0xbf: LCD_Print((uint8_t)charTable[49]); break; //п
			case 0x80: LCD_Print((uint8_t)charTable[50]); break; //р
			case 0x81: LCD_Print((uint8_t)charTable[51]); break; //с
			case 0x82: LCD_Print((uint8_t)charTable[52]); break; //т
			case 0x83: LCD_Print((uint8_t)charTable[53]); break; //у
			case 0x84: LCD_Print((uint8_t)charTable[54]); break; //ф
			case 0x85: LCD_Print((uint8_t)charTable[55]); break; //х
			case 0x86: LCD_Print((uint8_t)charTable[56]); break; //ц
			case 0x87: LCD_Print((uint8_t)charTable[57]); break; //ч
			case 0x88: LCD_Print((uint8_t)charTable[58]); break; //ш
			case 0x89: LCD_Print((uint8_t)charTable[59]); break; //щ
			case 0x8a: LCD_Print((uint8_t)charTable[60]); break; //ъ
			case 0x8b: LCD_Print((uint8_t)charTable[61]); break; //ы
			case 0x8c: LCD_Print((uint8_t)charTable[62]); break; //ь
			case 0x8d: LCD_Print((uint8_t)charTable[63]); break; //э
			case 0x8e: LCD_Print((uint8_t)charTable[64]); break; //ю
			case 0x8f: LCD_Print((uint8_t)charTable[65]); break; //я
			default: LCD_Print((uint8_t)*ch);
		}
		ch++;
	}
	delay(1);
}

void LCD_Clear(void)
{
	LCD_Command(0x01);
}

void LCD_PrintWithBits(char *bits,uint8_t Location_1,uint8_t Location_2)
{
  static uint8_t memory = 0;
	LCD_Command(0x40 + 8*memory);
	
	for (int i=0; i<8; i++)
	{
	  LCD_Print(*bits);
		bits++;
	}
	
	if(Location_1 == 1)
	{
		LCD_Command(0x80 + Location_2);
		LCD_Print(0x00 + memory++);
	}
	else
	{
		LCD_Command(0xc0 + Location_2);
		LCD_Print(0x00 + memory++);
	}
	if(memory > 7)
		memory = 0;
	delay(1);
}

void LCD_Set_Cursor(uint8_t Location_1,uint8_t Location_2)
{
	if(Location_1 == 1)
		LCD_Command(0x80 + Location_2);
	else
		LCD_Command(0xc0 + Location_2);
	delay(1);
}

void LCD_Cursor_On_Off(_Bool flag)
{
	if(flag)
		LCD_Command(0x0f);
	else
		LCD_Command(0x0c);
	delay(1);
}

void clock(void)
{
		HAL_RTC_GetTime(&hrtc,&sTime, RTC_FORMAT_BIN);
		sprintf(trans_str,"Время:%02d:%02d:%02d",sTime.Hours,sTime.Minutes,sTime.Seconds);
		LCD_Set_Cursor(1,0);
		LCD_Print_String(trans_str);
		
		HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
		sprintf(trans_str,"Дата:%02d-%02d-20%02d", DateToUpdate.Date, DateToUpdate.Month, DateToUpdate.Year);
		LCD_Set_Cursor(2,0);
		LCD_Print_String(trans_str);
}

void get_time(void)
{
	sTime.Hours = 10*(buff[0] - '0') + (buff[1] - '0');
	sTime.Minutes = 10*(buff[3] - '0') + (buff[4] - '0');
	sTime.Seconds = 10*(buff[6] - '0') + (buff[7] - '0');
	HAL_RTC_SetTime(&hrtc,&sTime,RTC_FORMAT_BIN);
}

void get_date(void)
{
	DateToUpdate.Date = 10*(buff[9] - '0') + (buff[10] - '0');
	DateToUpdate.Month = 10*(buff[12] - '0') + (buff[13] - '0');
	DateToUpdate.Year = 10*(buff[15] - '0') + (buff[16] - '0');
	HAL_RTC_SetDate(&hrtc,&DateToUpdate,RTC_FORMAT_BIN);
}

void random_print(void)
{
	static uint8_t rn_array[32],i,j;
	uint8_t rn_new,k = 0;
	rn_new = HAL_RNG_GetRandomNumber(&hrng)%32;
	rn_array[i] = rn_new;
	while(i > 0)
	{
		if(rn_new != rn_array[i - 1])
			k++;
		i--;
	}
	if(k == j)
	{
		if(rn_new < 16)
			LCD_PrintWithBits(data_pixels,1,rn_new);
		else
			LCD_PrintWithBits(data_pixels,2,rn_new%16);
		j++;
		delay(100);
  }
	i = j;
}

void set_brightness(uint16_t br)
{
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,br);
}

uint8_t analyze_protocol(struct protocol* prot,char* buff)
{
	uint8_t head_data[5];
	uint8_t number;
	uint8_t uart_port;
	uint32_t data_recieved;
	uint16_t check_sum_recieved;
	uint16_t check_sum_calculated;
	uint64_t protocol_recieved = 0;
	while(*buff != '\0')
	{
		switch(*buff)
		{
			case 'a': protocol_recieved |= 0xa; break;
			case 'A': protocol_recieved |= 0xa; break;
			case 'b': protocol_recieved |= 0xb; break;
			case 'B': protocol_recieved |= 0xb; break;
			case 'c': protocol_recieved |= 0xc; break;
			case 'C': protocol_recieved |= 0xc; break;
			case 'd': protocol_recieved |= 0xd; break;
			case 'D': protocol_recieved |= 0xd; break;
			case 'e': protocol_recieved |= 0xe; break;
			case 'E': protocol_recieved |= 0xe; break;
			case 'f': protocol_recieved |= 0xf; break;
			case 'F': protocol_recieved |= 0xf; break;
			default: protocol_recieved |= *buff - '0'; break;
		}
		protocol_recieved <<= 4;
		buff++;
  }
	protocol_recieved >>= 4;
	number = (protocol_recieved >> 52) & 0xf;
	uart_port = (protocol_recieved >> 48) & 0xf;
	data_recieved = (protocol_recieved >> 16) & 0xffffffff;
	check_sum_recieved = protocol_recieved & 0xffff;
	head_data[0] = (number << 4) | uart_port;
	head_data[1] = (data_recieved >> 24) & 0xff;
	head_data[2] = (data_recieved >> 16) & 0xff;
	head_data[3] = (data_recieved >> 8) & 0xff;
	head_data[4] = data_recieved & 0xff;
	check_sum_calculated = HAL_CRC_Calculate(&hcrc,(uint32_t *)head_data,5);
	if(check_sum_recieved == check_sum_calculated)
	{
		prot->number = number;
		prot->uart_port = uart_port;
		prot->data = data_recieved;
		prot->check_sum = check_sum_recieved;
		return 1;
	}
	else
		return 0;
}

void print_protocol(struct protocol* prot)
{
	char output[150];
	sprintf(output,"Датчик № = %d\nПорт USART = %d\nДанные = %u\nКонтрольная сумма = %04X\n",prot->number,prot->uart_port,prot->data,prot->check_sum);
	print_to_USART(output);
}

void random_protocol(void)
{
	uint64_t head_data_rnd = 0;
	uint64_t protocol_rnd;
	uint8_t head_data[5];
	uint8_t i = 0,j = 13,k = 0;
	uint16_t check_sum;
	while(i < 10)
	{
		head_data_rnd |= HAL_RNG_GetRandomNumber(&hrng)%16;
		head_data_rnd <<= 4;
		i++;
	}
	head_data_rnd >>= 4;
	head_data[0] = (head_data_rnd >> 32) & 0xff;
	head_data[1] = (head_data_rnd >> 24) & 0xff;
	head_data[2] = (head_data_rnd >> 16) & 0xff;
	head_data[3] = (head_data_rnd >> 8) & 0xff;
  head_data[4] = head_data_rnd & 0xff;
	check_sum = HAL_CRC_Calculate(&hcrc,(uint32_t *)head_data,5);
	protocol_rnd = (head_data_rnd << 16) + check_sum;
	while(k < 14)
	{
		switch((protocol_rnd >> 4*j--) & 0xf)
		{
			case 0: protocol[k] = '0'; break;
			case 1: protocol[k] = '1'; break;
			case 2: protocol[k] = '2'; break;
			case 3: protocol[k] = '3'; break;
			case 4: protocol[k] = '4'; break;
			case 5: protocol[k] = '5'; break;
			case 6: protocol[k] = '6'; break;
			case 7: protocol[k] = '7'; break;
			case 8: protocol[k] = '8'; break;
			case 9: protocol[k] = '9'; break;
			case 10: protocol[k] = 'a'; break;
			case 11: protocol[k] = 'b'; break;
			case 12: protocol[k] = 'c'; break;
			case 13: protocol[k] = 'd'; break;
			case 14: protocol[k] = 'e'; break;
			case 15: protocol[k] = 'f'; break;
		}
		k++;
	}
}