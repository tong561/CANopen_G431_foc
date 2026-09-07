#include "main.h"
#include "my_main.h"
#include "tim.h"
#include "adc.h"
#include "CO_app_STM32.h"
#include "fdcan.h"
#include "myadc.h"
#include "bspUsart.h"
#include "MT6816.h"
#include "stdio.h"
#include "usbd_cdc_if.h"
#include "bspUSB.h"
#include <string.h>
#include "foc.h"
#include "math.h"
#define CURRENT_LIMIT_MA   2000.0f
PPDetect_t PoleDetect;
volatile uint8_t OverCurrentFlag = 0;
volatile uint8_t FOC_RunFlag = 0;
/*************************canopenNode 参数介绍*****************************
| 成员                | 你要不要配置 | 你的值              | 作用                          
| ------------------- | ------     | ---------------- | ------------------------           
| `CANHandle`         | 要         | `&hfdcan1`       | 指定使用哪个 FDCAN                  
| `HWInitFunction`    | 要         | `MX_FDCAN1_Init` | CANopen 通信复位时重新初始化 FDCAN  
| `timerHandle`       | 要         | `&htim6`         | CANopen 的 1ms 周期定时器           
| `desiredNodeID`     | 要         | `1`              | 希望使用的 CANopen Node-ID         
| `baudrate`   
| 要         | `500`            | CAN 波特率，单位按该移植层使用kbps   
| `activeNodeID`      | 不要       | 不填             | 初始化后协议栈实际使用的 Node-ID     
| `outStatusLEDGreen` | 不要       | 不填             | CANopen RUN LED 状态                
| `outStatusLEDRed`   | 不要       | 不填             | CANopen ERROR LED 状态              
| `canOpenStack`      | 不要       | 不填             | 内部 `CO_t *` 协议栈对象             
*************************************************************************************/
uint32_t adc1_value=0;
uint32_t adc2_value=0;
typedef union float_to_byte
{
	float f;
	uint8_t b;
}float_to_byte_t;
typedef union u16_to_byte
{
	float u16;
	uint8_t b;
}u16_to_byte_t;

typedef struct	FOC_Pram_type
{
	float_to_byte_t	 FOC_encoder_raw;
	float_to_byte_t  theta_e;
	float_to_byte_t  angle_error;
	float_to_byte_t  Id;
	float_to_byte_t  Iq;
	float_to_byte_t		motor_speed_rpm ;
	float_to_byte_t 	motor_speed_rpm_filt;

	//float_to_byte_t  aaa;
	char arr[4];
}FOC_Pram_t;
FOC_Pram_t FOC_Pram[254],FOC_Pram1[254];
uint8_t send_pos=0,save_pos=0,send_flag=0,sendArr_flag=0;
float a=0;
float angle_error;
float current_angle;
int add_Iflag=0;
char I_flag=0;
long pos=0;
void my_main(void)
{
    canopenNodeSTM32->timerHandle = &htim6;//初始化canopen协议层定时器
    canopenNodeSTM32->CANHandle = &hfdcan1;//初始化canopen协议层CAN句柄
    canopenNodeSTM32->HWInitFunction = MX_FDCAN1_Init;//设置canopen协议层硬件初始化函数
    canopenNodeSTM32->desiredNodeID = 0x01;//设置canopen协议层期望节点ID为0x01
    canopenNodeSTM32->baudrate = 500;//设置canopen协议层波特率为500Kbps
    //canopen_app_init(canopenNodeSTM32);//初始化canopen协议层
		/* ADC校准，只需要初始化时做一次 */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
		uart_printf("usart1 is OK!\r\n");
		//uart_send_periodic_task(&huart1);
		uart_printf("offset_a=%d,offset_b=%d\r\n",ADC_parm.offset_a,ADC_parm.offset_b);
		uart_send_periodic_task(&huart1);
		HAL_Delay(1);
		HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
		HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

		/* ADC2 slave 先启动 */
		HAL_ADCEx_InjectedStart_IT(&hadc2);

		/* ADC1 master 再启动 */
		HAL_ADCEx_InjectedStart_IT(&hadc1);
		HAL_TIM_Base_Start(&htim1);
		uint16_t MT6816_data=0;
		
		DRV8313_DISABLE();


		/*
		 * 等 ADC 零电流偏置校准完成
		 * 你 ADC_Init() 需要约 2000 次采样。
		 */
		HAL_Delay(500);



		OverCurrentFlag = 0;

		/*
		 * 清 PI积分量
		 */
		PI_Id.integral = 0.0f;
		PI_Iq.integral = 0.0f;
		FOC_UpdateElectricalAngle();//先取得一次正确角度
		FOC_SVPWM(0.0f, 0.0f);//PWM先输出零电压：
		FOC_PWM_Start();//开PWM
		DRV8313_ENABLE();//最后开驱动

		FOC_RunFlag = 1;//允许闭环
		uint8_t result_printed = 0;
		uint16_t angle = MT6816_ReadOneAngle();
		usb_print("angle=%u\r\n", angle);


char usb_flag=0;




while(1)
{
	
//	FOC_SetOpenLoopVector(a, 0.2f);
	//usb_print("FOC_encoder_raw=%d,%f \r\n",FOC_encoder_raw,theta_e);
//	a-=FOC_2PI/51600.0f;

//	if(a>FOC_2PI)
//		a+=FOC_2PI;
	if(send_flag&&sendArr_flag==1)
		{
			if(usb_flag==0)
				if(	CDC_Transmit_FS((uint8_t *)&FOC_Pram,sizeof(FOC_Pram)) == USBD_OK)
					send_flag=0;
		}
		else if(send_flag&&sendArr_flag==0)
		{
			if(usb_flag==0)
				if(	CDC_Transmit_FS((uint8_t *)&FOC_Pram1,sizeof(FOC_Pram1)) == USBD_OK)
					send_flag=0;

		}
		
		if(I_flag)
		{
			I_flag=0;
			
			pos += 16384*8;
		}
//	whil
//	usb_print("%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d,%f,%f,%f,%f,%f\r\n",
//		FOC_encoder_raw,
//    theta_e,
//    ADC_parm.I_a,
//    ADC_parm.I_b,
//    ADC_parm.I_c,
//    I_d,
//    I_q,
//		adc1_value,
//		adc2_value,
//		I_alpha,
//		I_beta,
//		current_angle	,
//		angle_error,
//		I_mag
//	);
}
}







/*******************************中断回调部分************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == canopenNodeSTM32->timerHandle->Instance)//1ms定时中断，canopen协议层用
    {
      // canopen_app_interrupt();
    }
}


void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
	static uint32_t SUM_A ,SUM_B=10;//求偏置
	static char InitOverFlag=0;
	static unsigned char add_i=0;

    if (hadc->Instance == ADC1) {
			
				adc1_value = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
				adc2_value = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
				/* 读取编码器，计算电角度 */
				FOC_UpdateElectricalAngle();
				FOC_SpeedCalculate(FOC_encoder_raw);
				FOC_POSCalculate(FOC_encoder_raw);
				//FOC_SpeedLoop(400);
					FOC_PosLoop(pos);
				if(!InitOverFlag)
				{
					ADC_parm.V_a=adc1_value;
					ADC_parm.V_b=adc2_value;
					InitOverFlag=ADC_Init(&ADC_parm);
				}
				else
				{
					ADC_parm.V_a=adc1_value-ADC_parm.offset_a;
					ADC_parm.V_b=adc2_value-ADC_parm.offset_b;
					// 转换为电流值
					CurrentCalculation(&ADC_parm);
					/* ==========================
             *  软件过流保护
             * ========================== */

            if((ADC_parm.I_a >  CURRENT_LIMIT_MA) ||
							 (ADC_parm.I_a < -CURRENT_LIMIT_MA) ||
							 (ADC_parm.I_b >  CURRENT_LIMIT_MA) ||
							 (ADC_parm.I_b < -CURRENT_LIMIT_MA) ||
							 (ADC_parm.I_c >  CURRENT_LIMIT_MA) ||
							 (ADC_parm.I_c < -CURRENT_LIMIT_MA))
						{
								FOC_RunFlag = 0;
								OverCurrentFlag = 1;

								DRV8313_DISABLE();
								FOC_PWM_Stop();

								return;
						}
						// 执行 FOC 运算
						

						/* Clarke */
						FOC_Clarke(ADC_parm.I_a,ADC_parm.I_b);

						/* Park */
						FOC_Park(I_alpha,I_beta,theta_e);
						
						if(FOC_RunFlag && !OverCurrentFlag)
						{
								
								FOC_CurrentLoop();
						}
						if(sendArr_flag==0)
						{
							current_angle = atan2f(I_beta, I_alpha);
							angle_error = current_angle - theta_e;
							while(angle_error > FOC_PI)
									angle_error -= FOC_2PI;
							while(angle_error < -FOC_PI)
									angle_error += FOC_2PI;
							
							FOC_Pram[add_i].FOC_encoder_raw.f	=(float)FOC_encoder_raw;
							FOC_Pram[add_i].angle_error.f			=(float)Iq_ref;
							FOC_Pram[add_i].motor_speed_rpm.f	=(float)error_V;
							FOC_Pram[add_i].motor_speed_rpm_filt.f=(float)motor_speed_rpm_filt;
							FOC_Pram[add_i].Id.f							=I_d;
							FOC_Pram[add_i].Iq.f							=I_q;
							FOC_Pram[add_i].theta_e.f					=(float)POS_PI.pos;
//							while(FOC_Pram[add_i].theta_e.f > FOC_PI)
//								FOC_Pram[add_i].theta_e.f -= FOC_2PI;

//							while(FOC_Pram[add_i].theta_e.f < -FOC_PI)
//								FOC_Pram[add_i].theta_e.f += FOC_2PI;
							FOC_Pram[add_i].arr[2]=0x80;
							FOC_Pram[add_i].arr[3]=0x7f;
						}
						else 
						{
							current_angle = atan2f(I_beta, I_alpha);
							angle_error = current_angle - theta_e;
							while(angle_error > FOC_PI)
									angle_error -= FOC_2PI;
							while(angle_error < -FOC_PI)
									angle_error += FOC_2PI;
					
							FOC_Pram1[add_i].FOC_encoder_raw.f	=(float)FOC_encoder_raw;
							FOC_Pram1[add_i].angle_error.f			=(float)Iq_ref;
							FOC_Pram1[add_i].motor_speed_rpm.f	=(float)error_V;
							FOC_Pram1[add_i].motor_speed_rpm_filt.f=(float)motor_speed_rpm_filt;
							FOC_Pram1[add_i].Id.f							=	I_d;
							FOC_Pram1[add_i].Iq.f							=I_q;
							FOC_Pram1[add_i].theta_e.f					=(float) POS_PI.pos;
//							while(FOC_Pram1[add_i].theta_e.f > FOC_PI)
//								FOC_Pram1[add_i].theta_e.f -= FOC_2PI;

//							while(FOC_Pram1[add_i].theta_e.f < -FOC_PI)
//								FOC_Pram1[add_i].theta_e.f += FOC_2PI;
							FOC_Pram1[add_i].arr[2]=0x80;
							FOC_Pram1[add_i].arr[3]=0x7f;
						}
						add_i++;
						if(add_i>=254)
						{
							add_Iflag++;
							if(add_Iflag>=200)
							{
								I_flag=1;
								add_Iflag=0;
							}
							send_flag=1;
							sendArr_flag=!sendArr_flag;
							add_i=0;
						}
				}
    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
		if (huart == &huart1)
		{
			uart_debug_tx_cplt_callback();
		}
		
//		else if (huart == &huart3)
//		{
//			// DMA发送完成
//     
//			uart_raw_tx_cplt_callback();
//		}
}
	



