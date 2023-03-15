#ifndef FILTER_H_
#define FILTER_H_

#define COEF 4095.0f/5.0f
#define COUNTS 360*1
#define NOISE_AMP 10
#define WINDOW 50
#define NOISE 0

void make_noise(float*,int);
void Filter(float*,float*,int,int);
void transmit_to_USART(float*,float*,int);
double normal_distribution(void);
double uniform_distribution(void);

#endif /* FILTER_H_  */