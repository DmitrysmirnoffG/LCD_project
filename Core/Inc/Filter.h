#ifndef FILTER_H_
#define FILTER_H_

void make_noise(float*,int);
void Filter(float*,float*,int,int);
void transmit_to_USART(float*,float*,int);

#endif /* FILTER_H_  */