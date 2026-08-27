#include "bspUSB.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include "main.h"
#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include "usbd_def.h"
#include "usbd_core.h"
#include "usb_device.h"
#include "usbd_cdc.h"
#include "bspUSB.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"

#include <stdio.h>
#include <stdarg.h>

#define USB_PRINT_BUF_SIZE    256

/* 必须是 static/global
 * CDC发送是异步的，不能使用局部数组后马上退出函数
 */
static uint8_t usb_print_buf[USB_PRINT_BUF_SIZE];

int usb_print(const char *fmt, ...)
{
    USBD_CDC_HandleTypeDef *hcdc;
    va_list args;
    int len;
    uint8_t ret;

    /* USB还没有完成枚举 */
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    {
        return -1;
    }

    /* CDC Class还没有初始化 */
    if (hUsbDeviceFS.pClassData == NULL)
    {
        return -2;
    }

    hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

    /* 上一包还没发完
       这里直接丢弃，绝对不要while死等 */
    if (hcdc->TxState != 0)
    {
        return -3;
    }

    va_start(args, fmt);

    len = vsnprintf((char *)usb_print_buf,
                    USB_PRINT_BUF_SIZE,
                    fmt,
                    args);

    va_end(args);

    if (len < 0)
    {
        return -4;
    }

    /* vsnprintf发生截断 */
    if (len >= USB_PRINT_BUF_SIZE)
    {
        len = USB_PRINT_BUF_SIZE - 1;
    }

    ret = CDC_Transmit_FS(usb_print_buf, (uint16_t)len);

    if (ret != USBD_OK)
    {
        return -5;
    }

    return len;
}
