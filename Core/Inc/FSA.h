#ifndef FSA_H_
#define FSA_H_

#include "main.h"
#define RB_ELEMENTS 40
#define ARRAY_ELEMENTS 20
#define READY 'R'
#define DONE 'D'
#define ERROR 'E'

struct ring_buffer
{
	uint8_t tail;
	uint8_t head;
	uint8_t arr[RB_ELEMENTS];
};

void vend_mach(uint8_t);
void write_to_ringbuffer(struct ring_buffer*);
int read_from_ringbuffer(char*,struct ring_buffer*);
void clear_array(char*);
int string_to_int(char*);
void print_to_USART(char*);

#endif /* FSA_H_  */