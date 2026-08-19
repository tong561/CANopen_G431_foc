#include "main.h"
#include "my_main.h"
#include "tim.h"
#include "fdcan.h"
#include "CO_app_STM32.h"
/*************************canopenNode 参数介绍*****************************
| 成员                  | 你要不要配置 | 你的值              | 作用                       |
| ------------------- | ------ | ---------------- | ------------------------ |
| `CANHandle`         | ?      | `&hfdcan1`       | 指定使用哪个 FDCAN             |
| `HWInitFunction`    | ?      | `MX_FDCAN1_Init` | CANopen 通信复位时重新初始化 FDCAN |
| `timerHandle`       | ?      | `&htim6`         | CANopen 的 1ms 周期定时器      |
| `desiredNodeID`     | ?      | `1`              | 希望使用的 CANopen Node-ID    |
| `baudrate`          | ?      | `500`            | CAN 波特率，单位按该移植层使用 kbps   |
| `activeNodeID`      | ?      | 不填               | 初始化后协议栈实际使用的 Node-ID     |
| `outStatusLEDGreen` | ?      | 不填               | CANopen RUN LED 状态       |
| `outStatusLEDRed`   | ?      | 不填               | CANopen ERROR LED 状态     |
| `canOpenStack`      | ?      | 不填               | 内部 `CO_t *` 协议栈对象        |
*************************************************************************************/
void my_main(void)
{
    canopenNodeSTM32->timerHandle = &htim6;//初始化canopen协议层定时器

    canopenNodeSTM32->CANHandle = &hfdcan1;//初始化canopen协议层CAN句柄
    canopenNodeSTM32->HWInitFunction = MX_FDCAN1_Init;//设置canopen协议层硬件初始化函数
    canopenNodeSTM32->desiredNodeID = 0x01;//设置canopen协议层期望节点ID为0x01
    canopenNodeSTM32->baudrate = 500;//设置canopen协议层波特率为500Kbps
    canopen_app_init(canopenNodeSTM32);//初始化canopen协议层
    
    while(1)
    {
        canopen_app_process();
    }

}







/*******************************中断回调部分************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == canopenNodeSTM32->timerHandle->Instance)//1ms定时中断，canopen协议层用
    {
       canopen_app_interrupt();
    }
}




