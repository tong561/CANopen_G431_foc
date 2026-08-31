#ifndef _MY_ADC_H_
#define _MY_ADC_H_
#include "main.h"
typedef struct ADC_Type
{
	uint16_t offset_a;//A相静态偏置电压
	uint16_t offset_b;//B相静态偏置电压
	int16_t V_a;//A相电压
	int16_t V_b;//B相电压
	float I_a;//A项电流
	float I_b;//B项电流
	float I_c;//C项电流

	uint16_t buff_Ride_R;	 //采样电阻单位m欧(固定)*增益倍数（固定）
}ADC_Type_t;

extern ADC_Type_t ADC_parm;
unsigned char ADC_Init(ADC_Type_t *ADC_parm);
void CurrentCalculation(ADC_Type_t *ADC_parm);//电流计算
#endif
