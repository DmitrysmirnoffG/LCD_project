#include "Filter.h"
#include "main.h"
#include "FastMath.h"
#include "stdlib.h"
#include "math.h"
#include "stdio.h"
#include "FSA.h"
#include "lcd.h"

double normal_distribution(void)
{
  double x = (double)rand() / RAND_MAX;
  double y = (double)rand() / RAND_MAX;
  double z = FastSqrt(-2*logf(x))*FastCos(2*PI*y)/30;
  return z;
}

double uniform_distribution(void)
{
	double x = (rand()%1001 - 500)/10000.0f;
	return x;
}

void make_noise(float* signal_noise,int N)
{
	for(uint16_t i = 0; i < N; i++)
	{
		if(NOISE == 1)
			signal_noise[i] = 1.5f + FastSin(i*PI/180) + NOISE_AMP*normal_distribution();
		else
			signal_noise[i] = 1.5f + FastSin(i*PI/180) + NOISE_AMP*uniform_distribution();
	}
}

void Filter(float* input,float* output,int N,int window)
{
	int i,j,z,k1,k2,hw;
	float tmp;
	if(fmod(window,2) == 0) 
		window++;
	hw = (window - 1)/2;
	output[0] = input[0];
	
	for (i = 1;i < N;i++)
	{
	 tmp = 0;
	 if(i < hw)			
	 {
		 k1 = 0;
		 k2 = 2*i;
		 z = k2 + 1;
	 }
	 else if((i + hw)>(N - 1))
	 {
		 k1 = i - N + i + 1;
		 k2 = N - 1;
		 z = k2 - k1 + 1;
	 }
	 else
	 {
		 k1 = i - hw;
		 k2 = i + hw;
		 z = window;
	 }

	 for (j = k1;j <= k2;j++)
	   tmp = tmp + input[j];
	 output[i] = tmp/z;
	}
}

//void transmit_to_USART(float* signal_noise,float* signal_filtered,int N)
//{
//	static uint8_t flag = 0;
//	char signal_data[10];
//	if(flag == 0)
//	{
//		print_to_USART("Сигнал с шумом:");
//		print_to_USART("");
//		for(uint16_t i = 0; i < N; i++)
//		{
//			sprintf(signal_data,"%f",signal_noise[i]);
//			print_to_USART(signal_data);
//			clear_array(signal_data);
//		}
//		print_to_USART("---------------------------------");
//		print_to_USART("Отфильтрованный сигнал:");
//		print_to_USART("");
//		for(uint16_t i = 0; i < N; i++)
//		{
//			sprintf(signal_data,"%f",signal_filtered[i]);
//			print_to_USART(signal_data);
//			clear_array(signal_data);
//		}
//		flag = 1;
//  }
//}