#ifndef _BSP_USART_H_
#define _BSP_USART_H_



#include "main.h"
#include "usart.h"

#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>


/* 普通 printf 调试输出 -> huart1 */
void uart_printf(const char *format, ...);


/* 二进制数据 -> huart1 */
void uart_debug_write_bytes(const void *data, uint16_t len);


/* 二进制 raw 数据 -> huart3 */
void uart_write_raw_bytes(const void *data, uint16_t len);


/* 周期发送任务 */
void uart_send_periodic_task(UART_HandleTypeDef *huart);


/* DMA 发送完成回调 */
void uart_debug_tx_cplt_callback(void);
void uart_raw_tx_cplt_callback(void);


/*
 * C 没有 template，所以用宏代替原来的：
 *
 * uart_send_raw(ia, ib, iq);
 */

#define UART_SEND_RAW_ONE(value) \
    uart_write_raw_bytes(&(value), sizeof(value))

#define UART_SEND_RAW2(a, b)        \
    do                              \
    {                               \
        UART_SEND_RAW_ONE(a);       \
        UART_SEND_RAW_ONE(b);       \
    } while (0)

#define UART_SEND_RAW3(a, b, c)     \
    do                              \
    {                               \
        UART_SEND_RAW_ONE(a);       \
        UART_SEND_RAW_ONE(b);       \
        UART_SEND_RAW_ONE(c);       \
    } while (0)

#define UART_SEND_RAW4(a, b, c, d)  \
    do                              \
    {                               \
        UART_SEND_RAW_ONE(a);       \
        UART_SEND_RAW_ONE(b);       \
        UART_SEND_RAW_ONE(c);       \
        UART_SEND_RAW_ONE(d);       \
    } while (0)










#endif
