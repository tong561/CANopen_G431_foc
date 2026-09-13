#include "main.h"
#include "adc.h"
#include "myadc.h"


ADC_Type_t ADC_parm;
unsigned char ADC_Init(ADC_Type_t *ADC_parm)//初始化偏置电流
{
	static uint16_t add_i=0;
	//50倍增益，10m欧
	ADC_parm->buff_Ride_R=2;//50*10; a/500*1000=a*2
	
	static uint64_t sum_a=0,sum_b=0;
	if(add_i<2000)
	{		add_i++;
			sum_a += ADC_parm->V_a;
			sum_b += ADC_parm->V_b;
			return 0;
	}
	ADC_parm->offset_a =	(uint16_t)	(sum_a / 2000U);
	ADC_parm->offset_b =	(uint16_t)	(sum_b / 2000U);
	return 1;
}
//电流计算公式
/*
U=RI
Uo=U/F
Uo=RIF
I=Uo/(R*F)
其中F为运放增益，Uo为运放输出电压即采样电压，R为采样电阻
3300/4095=0.80586
0.80586*2=	1.61172
*/
void CurrentCalculation(ADC_Type_t *ADC_parm)//电流计算
{
	ADC_parm->I_a=(float)ADC_parm->V_a*0.80586f;//1.61172f*2;//ADC_parm->buff_Ride_R*0.80586;
	ADC_parm->I_b=(float)ADC_parm->V_b*0.80586f;//1.61172f*2;//ADC_parm->buff_Ride_R*0.80586;
	ADC_parm->I_c=-(float)(ADC_parm->I_a+ADC_parm->I_b);
}
