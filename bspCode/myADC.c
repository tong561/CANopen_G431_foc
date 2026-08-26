#include "main.h"
#include "adc.h"
#include "myadc.h"


ADC_Type_t ADC_parm;
unsigned char ADC_Init(ADC_Type_t *ADC_parm)//初始化偏置电流
{
	uint16_t add_i=0;
	ADC_parm->buff_Ride_R=50*10;
	
	static uint32_t sum_a=0,sum_b=0;
	if(add_i<2000)
	{
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
*/
void CurrentCalculation(ADC_Type_t *ADC_parm)//电流计算
{
	ADC_parm->I_a=ADC_parm->V_a/ADC_parm->buff_Ride_R;
	ADC_parm->I_b=ADC_parm->V_b/ADC_parm->buff_Ride_R;
	ADC_parm->I_c=-(ADC_parm->I_a+ADC_parm->I_b);
}
