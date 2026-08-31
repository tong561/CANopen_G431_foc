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

#define CURRENT_LIMIT_MA   1200.0f
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
		 *
		 * 你 ADC_Init() 需要约 2000 次采样。
		 */
		HAL_Delay(500);


		/*
		 * 开始前清状态
		 */
//OverCurrentFlag = 0;


	//		/*
	//		 * 开启驱动器
	//		 */
	//		DRV8313_ENABLE();


	//		/*
	//		 * 正式开始自动测极对数
	//		 */
	//		FOC_PolePairDetect_Start(&PoleDetect);

/*
 * 清状态
 */
OverCurrentFlag = 0;

/*
 * 清 PI
 */
PI_Id.integral = 0.0f;
PI_Iq.integral = 0.0f;

/*
 * 先取得一次正确角度
 */
FOC_UpdateElectricalAngle();

/*
 * PWM先输出零电压：
 * 三相都是50%
 */
FOC_SVPWM(0.0f, 0.0f);

/*
 * 开PWM
 */
FOC_PWM_Start();

/*
 * 最后开驱动
 */
DRV8313_ENABLE();

/*
 * 允许闭环
 */
FOC_RunFlag = 1;

		uint8_t result_printed = 0;

//    while(1)
//    {

//				/*
//				 * 等 ADC 零电流偏置校准完成
//				 *
//				 * 你 ADC_Init() 需要约 2000 次采样。
//				 */
//				HAL_Delay(500);
//				/*
//				 * 开始前清状态
//				 */
//				OverCurrentFlag = 0;
//				/*
//				 * 开启驱动器
//				 */
//				DRV8313_ENABLE();
//				/*
//				 * 正式开始自动测极对数
//				 */
//				FOC_PolePairDetect_Start(&PoleDetect);

//				uint8_t result_printed = 0;

				uint16_t angle = MT6816_ReadOneAngle();
				usb_print("angle=%u\r\n", angle);
//				DRV8313_ENABLE();

//FOC_PWM_Start();



//FOC_StablePointTest();
//while(1)
//{
//		


//   
//}

while(1)
{
//    /*
//     * 正常情况下不断运行检测状态机
//     */
//    if(!OverCurrentFlag)
//    {
//        FOC_PolePairDetect_Task(&PoleDetect);
//    }

//    /*
//     * 300mA 过流
//     */
//    if(OverCurrentFlag && !result_printed)
//    {
//        result_printed = 1;
//        DRV8313_DISABLE();
//        FOC_PWM_Stop();
//        usb_print( "OVER CURRENT > 300mA!\r\n");
//    }


//    /*
//     * 极对数检测成功
//     */
//    if((PoleDetect.state == PP_DETECT_DONE) && !result_printed)
//    {
//        result_printed = 1;
//        DRV8313_DISABLE();
//        FOC_PWM_Stop();
//			while(1)
//			{
//        usb_print(
//            "Pole Detect OK!\r\n"
//            "PolePairs=%d\r\n"
//            "Estimate=%.3f\r\n"
//            "MechCount=%ld\r\n"
//            "Direction=%d\r\n",

//            PoleDetect.pole_pairs,
//            PoleDetect.pole_pairs_float,
//            PoleDetect.mechanical_count_acc,
//            PoleDetect.direction
//        );
//			}
//    }


//    /*
//     * 检测失败
//     */
//    if((PoleDetect.state == PP_DETECT_ERROR) &&!result_printed)
//    {
//        result_printed = 1;

//        DRV8313_DISABLE();
//        FOC_PWM_Stop();

//        usb_print(
//            "Pole Detect ERROR!\r\n"
//            "Estimate=%.3f\r\n"
//            "MechCount=%ld\r\n",

//            PoleDetect.pole_pairs_float,
//            PoleDetect.mechanical_count_acc
//        );
//    }


//    /*
//     * 被过流等原因中止
//     */
//    if((PoleDetect.state == PP_DETECT_ABORT) &&
//       !result_printed)
//    {
//        result_printed = 1;

//        DRV8313_DISABLE();
//        FOC_PWM_Stop();

//        usb_print( "Pole Detect ABORT!\r\n" );
//    }

	//	uart_printf("%d,%d,%d,%d,%d\r\n",ADC_parm.offset_a,ADC_parm.offset_b,adc1_value,adc2_value,MT6816_data);
		//usb_print("%d,%d,%d,%d,%d,%f,%f,%f,%f,%f\r\n",ADC_parm.offset_a,ADC_parm.offset_b,adc1_value,adc2_value,PoleDetect.encoder_last,ADC_parm.I_a,ADC_parm.I_b,ADC_parm.I_c,I_d,I_q);
		usb_print(
    "%u,%.4f,%.1f,%.1f,%.1f,%.1f,%.1f\r\n",
   FOC_encoder_raw,
    theta_e,
    ADC_parm.I_a,
    ADC_parm.I_b,
    ADC_parm.I_c,
    I_d,
    I_q
);

//    uart_send_periodic_task(&huart1);
}

//				MT6816_data=MT6816_ReadOneAngle();
//				//uart_printf("%d,%d,%d,%d,%d\r\n",ADC_parm.offset_a,ADC_parm.offset_b,adc1_value,adc2_value,MT6816_data);
//				usb_print("%d,%d,%d,%d,%d,%f,%f,%f\r\n",ADC_parm.offset_a,ADC_parm.offset_b,adc1_value,adc2_value,MT6816_data,ADC_parm.I_a,ADC_parm.I_b,ADC_parm.I_c);
//		
//				uart_send_periodic_task(&huart1);
//        //canopen_app_process();

    //}

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

    if (hadc->Instance == ADC1) {
			
				adc1_value = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
				adc2_value = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
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
             * 300mA 软件过流保护
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
						/* 读取编码器，计算电角度 */
						FOC_UpdateElectricalAngle();

						/* Clarke */
						FOC_Clarke(
								ADC_parm.I_a,
								ADC_parm.I_b
						);

						/* Park */
						FOC_Park(
								I_alpha,
								I_beta,
								theta_e
						);
				if(FOC_RunFlag && !OverCurrentFlag)
{
    FOC_CurrentLoop();
}
				}
        // 执行 FOC 运算
				
        // ...
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
	



