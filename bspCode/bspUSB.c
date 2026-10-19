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

#include "foc.h"
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <limits.h>

#define USB_RX_SIZE 512U
#define USB_TOKEN_SIZE 48U
static uint8_t usb_rx[USB_RX_SIZE];
static volatile uint16_t usb_rx_head=0U,usb_rx_tail=0U;
static volatile uint8_t usb_rx_overflow=0U;
volatile int32_t USB_SpeedTargetRPM=0;
volatile int64_t USB_PositionTarget=0;
volatile uint8_t USB_ControlMode=0;

void USB_RxPush(const uint8_t *data, uint32_t length)
{
    uint32_t i;
    if(usb_rx_overflow)return;
    for(i=0;i<length;i++)
    {
        uint16_t next=(uint16_t)((usb_rx_head+1U)%USB_RX_SIZE);
        if(next==usb_rx_tail){usb_rx_overflow=1U;return;}
        usb_rx[usb_rx_head]=data[i];
        __DMB();
        usb_rx_head=next;
    }
}

static int USB_ParseGain(char *token, float *value)
{
    char *end;
    errno=0;
    *value=strtof(token,&end);
    if(end==token || errno==ERANGE || !isfinite(*value) || *value<0.0f)return 0;
    if(*end=='f' || *end=='F')end++;
    return *end=='\0';
}
static int USB_ParseSpeed(char *token, int32_t *value)
{
    char *end;
    long parsed;
    errno=0;
    parsed=strtol(token,&end,10);
    if(end==token || errno==ERANGE || parsed<INT32_MIN || parsed>INT32_MAX)return 0;
    if(*end=='D' || *end=='d')end++;
    if(*end!='\0')return 0;
    *value=(int32_t)parsed;
    return 1;
}

static int USB_ParsePosition(char *token, int64_t *value)
{ char *end; long long parsed; errno=0; parsed=strtoll(token,&end,10); if(end==token || errno==ERANGE)return 0; if(*end=='P' || *end=='p')end++; if(*end!='\0')return 0; *value=(int64_t)parsed; return 1; }

/* Every field ends in a comma; USB packet boundaries have no significance. */
void USB_CommandProcess(void)
{
    static char token[USB_TOKEN_SIZE];
    static uint16_t used=0U;
    static uint8_t mode=0U,field=0U,bad_token=0U;
    static float gains[3];
    if(usb_rx_overflow)
    {
        uint32_t mask=__get_PRIMASK();
        __disable_irq();
        usb_rx_tail=usb_rx_head;
        usb_rx_overflow=0U;
        __set_PRIMASK(mask);
        used=mode=field=bad_token=0U;
        return;
    }
    while(usb_rx_tail!=usb_rx_head)
    {
        char c=(char)usb_rx[usb_rx_tail];
        __DMB();
        usb_rx_tail=(uint16_t)((usb_rx_tail+1U)%USB_RX_SIZE);
        if(c=='\r' || c=='\n')
        {
            if(used || mode){used=mode=field=bad_token=0U;}
            continue;
        }
        if(c==' ' || c=='\t')continue;
        if(c!=',')
        {
            if(used<USB_TOKEN_SIZE-1U)token[used++]=c;
            else bad_token=1U;
            continue;
        }
        token[used]='\0';
        if(bad_token){mode=field=bad_token=used=0U;continue;}
        used=0U;
        if(strcmp(token,"SPID")==0){mode=1U;field=0U;continue;}
        if(strcmp(token,"IPID")==0){mode=2U;field=0U;continue;}
        if(strcmp(token,"steep")==0){mode=3U;field=0U;continue;}
        if(strcmp(token,"PPID")==0){mode=4U;field=0U;continue;}
        if(strcmp(token,"RPOS")==0){mode=5U;field=0U;continue;}
        if(strcmp(token,"APOS")==0){mode=6U;field=0U;continue;}
        if(mode==3U)
        {
            int32_t speed;
            if(USB_ParseSpeed(token,&speed)){uint32_t mask=__get_PRIMASK();__disable_irq();USB_SpeedTargetRPM=speed;if(USB_ControlMode) FOC_PositionTrajectoryReset();USB_ControlMode=0U;__set_PRIMASK(mask);}
            mode=field=0U;
        }
        else if(mode==4U || mode==5U || mode==6U)
        {
            if(mode==4U){ if(!USB_ParseGain(token,&gains[field])){mode=field=0U;continue;} if(++field==3U){FOC_SetPositionPID(gains[0],gains[1],gains[2]);mode=field=0U;} }
            else { int64_t position,base; uint32_t mask; if(!USB_ParsePosition(token,&position)){mode=field=0U;continue;} mask=__get_PRIMASK();__disable_irq();base=POS_PI.pos; if(mode==5U && ((position>0 && base>INT64_MAX-position)||(position<0 && base<INT64_MIN-position))){__set_PRIMASK(mask);mode=field=0U;continue;} USB_PositionTarget=(mode==5U)?base+position:position; if(!USB_ControlMode) FOC_PositionTrajectoryReset();USB_ControlMode=1U;__set_PRIMASK(mask);mode=field=0U; }
        }
        else if(mode==1U || mode==2U)
        {
            if(!USB_ParseGain(token,&gains[field])){mode=field=0U;continue;}
            field++;
            if(mode==1U && field==3U)
            {
                FOC_SetSpeedPID(gains[0],gains[1],gains[2]);
                mode=field=0U;
            }
            else if(mode==2U && field==2U)
            {
                FOC_SetCurrentPI(gains[0],gains[1]);
                mode=field=0U;
            }
        }
    }
}
