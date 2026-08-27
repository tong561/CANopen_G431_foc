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
/*************************canopenNode 参数介绍*****************************
| 成员                | 你要不要配置 | 你的值              | 作用                          
| ------------------- | ------     | ---------------- | ------------------------           
| `CANHandle`         | 要         | `&hfdcan1`       | 指定使用哪个 FDCAN                  
| `HWInitFunction`    | 要         | `MX_FDCAN1_Init` | CANopen 通信复位时重新初始化 FDCAN  
| `timerHandle`       | 要         | `&htim6`         | CANopen 的 1ms 周期定时器           
| `desiredNodeID`     | 要         | `1`              | 希望使用的 CANopen Node-ID         
| `baudrate`          | 要         | `500`            | CAN 波特率，单位按该移植层使用kbps   
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
    while(1)
    {
				MT6816_data=MT6816_ReadOneAngle();
				//uart_printf("%d,%d,%d,%d,%d\r\n",ADC_parm.offset_a,ADC_parm.offset_b,adc1_value,adc2_value,MT6816_data);
				usb_print("%d,%d,%d,%d,%d\r\n",ADC_parm.offset_a,ADC_parm.offset_b,adc1_value,adc2_value,MT6816_data);
		
				uart_send_periodic_task(&huart1);
        //canopen_app_process();

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

    if (hadc->Instance == ADC1) {
			
				adc1_value = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
				adc2_value = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
				if(!InitOverFlag)
				{
					InitOverFlag=ADC_Init(&ADC_parm);
				}

				// 转换为电流值
				CurrentCalculation(&ADC_parm);
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
	



