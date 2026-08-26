#include "bspUsart.h"
#include <cstdarg>
#include <cstdio>


extern UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;


/* ============================================================
 * 配置
 * ============================================================ */

#define UART_DEBUG_BUF_SIZE     2048
#define UART_RAW_BUF_SIZE       1024
#define UART_TX_CHUNK_SIZE      512


/* ============================================================
 * 环形缓冲区
 * ============================================================ */

static uint8_t uart_debug_buf[UART_DEBUG_BUF_SIZE];
static uint8_t uart_raw_buf[UART_RAW_BUF_SIZE];

static uint8_t uart_debug_chunk[UART_TX_CHUNK_SIZE];
static uint8_t uart_raw_chunk[UART_TX_CHUNK_SIZE];


static volatile uint16_t uart_debug_head = 0;
static volatile uint16_t uart_debug_tail = 0;

static volatile uint16_t uart_raw_head = 0;
static volatile uint16_t uart_raw_tail = 0;


static volatile uint8_t uart_debug_busy = 0;
static volatile uint16_t uart_debug_sending_len = 0;

static volatile uint8_t uart_raw_busy = 0;
static volatile uint16_t uart_raw_sending_len = 0;


/* ============================================================
 * 环形缓冲区工具
 * ============================================================ */

static uint16_t uart_next_index(uint16_t index, uint16_t size)
{
    index++;

    if (index >= size)
    {
        index = 0;
    }

    return index;
}


static uint16_t uart_buf_count(
    volatile uint16_t head,
    volatile uint16_t tail,
    uint16_t size)
{
    if (head >= tail)
    {
        return head - tail;
    }

    return size - tail + head;
}


static void uart_buf_write(
    uint8_t *buf,
    volatile uint16_t *head,
    volatile uint16_t *tail,
    uint16_t size,
    const uint8_t *data,
    uint16_t len)
{
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        uint16_t next = uart_next_index(*head, size);

        /*
         * 缓冲区已满
         * 当前策略：后续数据直接丢弃
         */
        if (next == *tail)
        {
            break;
        }

        buf[*head] = data[i];

        *head = next;
    }
}


static void uart_buf_read_to_chunk(
    uint8_t *buf,
    volatile uint16_t tail,
    uint16_t size,
    uint8_t *chunk,
    uint16_t len)
{
    uint16_t index = tail;
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        chunk[i] = buf[index];

        index = uart_next_index(index, size);
    }
}


static void uart_buf_drop(
    volatile uint16_t *tail,
    uint16_t size,
    uint16_t len)
{
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        *tail = uart_next_index(*tail, size);
    }
}


/* ============================================================
 * printf
 *
 * 输出：
 * huart1
 * ============================================================ */

void uart_printf(const char *format, ...)
{
    char temp[UART_TX_CHUNK_SIZE];

    va_list args;

    int len;


    va_start(args, format);

    len = vsnprintf(
        temp,
        sizeof(temp),
        format,
        args
    );

    va_end(args);


    if (len <= 0)
    {
        return;
    }


    if (len >= (int)sizeof(temp))
    {
        len = sizeof(temp) - 1;
    }


    uart_buf_write(
        uart_debug_buf,
        &uart_debug_head,
        &uart_debug_tail,
        UART_DEBUG_BUF_SIZE,
        (const uint8_t *)temp,
        (uint16_t)len
    );
}


/* ============================================================
 * 二进制数据写入 debug 串口
 *
 * 输出：
 * huart1
 * ============================================================ */

void uart_debug_write_bytes(const void *data, uint16_t len)
{
    if ((data == NULL) || (len == 0))
    {
        return;
    }


    uart_buf_write(
        uart_debug_buf,
        &uart_debug_head,
        &uart_debug_tail,
        UART_DEBUG_BUF_SIZE,
        (const uint8_t *)data,
        len
    );
}


/* ============================================================
 * Raw 二进制数据
 *
 * 输出：
 * huart3
 * ============================================================ */

void uart_write_raw_bytes(const void *data, uint16_t len)
{
    if ((data == NULL) || (len == 0))
    {
        return;
    }


    uart_buf_write(
        uart_raw_buf,
        &uart_raw_head,
        &uart_raw_tail,
        UART_RAW_BUF_SIZE,
        (const uint8_t *)data,
        len
    );
}


/* ============================================================
 * huart1 DMA 发送
 * ============================================================ */

static void uart_debug_send_periodic_task(void)
{
    uint16_t count;
    uint16_t send_len;


    if (uart_debug_busy)
    {
        return;
    }


    count = uart_buf_count(
        uart_debug_head,
        uart_debug_tail,
        UART_DEBUG_BUF_SIZE
    );


    if (count == 0)
    {
        return;
    }


    send_len = count;


    if (send_len > UART_TX_CHUNK_SIZE)
    {
        send_len = UART_TX_CHUNK_SIZE;
    }


    uart_buf_read_to_chunk(
        uart_debug_buf,
        uart_debug_tail,
        UART_DEBUG_BUF_SIZE,
        uart_debug_chunk,
        send_len
    );


    uart_debug_busy = 1;
    uart_debug_sending_len = send_len;


    if (HAL_UART_Transmit_DMA(
            &huart1,
            uart_debug_chunk,
            send_len) != HAL_OK)
    {
        uart_debug_busy = 0;
        uart_debug_sending_len = 0;
    }
}


/* ============================================================
 * huart3 DMA 发送
 * ============================================================ */

static void uart_raw_send_periodic_task(void)
{
    uint16_t count;
    uint16_t send_len;


    if (uart_raw_busy)
    {
        return;
    }


    count = uart_buf_count(
        uart_raw_head,
        uart_raw_tail,
        UART_RAW_BUF_SIZE
    );


    if (count == 0)
    {
        return;
    }


    send_len = count;


    if (send_len > UART_TX_CHUNK_SIZE)
    {
        send_len = UART_TX_CHUNK_SIZE;
    }


    uart_buf_read_to_chunk(
        uart_raw_buf,
        uart_raw_tail,
        UART_RAW_BUF_SIZE,
        uart_raw_chunk,
        send_len
    );


    uart_raw_busy = 1;
    uart_raw_sending_len = send_len;


    if (HAL_UART_Transmit_DMA(
            &huart3,
            uart_raw_chunk,
            send_len) != HAL_OK)
    {
        uart_raw_busy = 0;
        uart_raw_sending_len = 0;
    }
}


/* ============================================================
 * 周期调用
 * ============================================================ */

void uart_send_periodic_task(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        uart_debug_send_periodic_task();
    }
    else if (huart == &huart3)
    {
        uart_raw_send_periodic_task();
    }
    else
    {
        uart_debug_send_periodic_task();
        uart_raw_send_periodic_task();
    }
}


/* ============================================================
 * huart1 DMA 完成
 * ============================================================ */

void uart_debug_tx_cplt_callback(void)
{
    uart_buf_drop(
        &uart_debug_tail,
        UART_DEBUG_BUF_SIZE,
        uart_debug_sending_len
    );


    uart_debug_sending_len = 0;

    uart_debug_busy = 0;
}


/* ============================================================
 * huart3 DMA 完成
 * ============================================================ */

void uart_raw_tx_cplt_callback(void)
{
    uart_buf_drop(
        &uart_raw_tail,
        UART_RAW_BUF_SIZE,
        uart_raw_sending_len
    );


    uart_raw_sending_len = 0;

    uart_raw_busy = 0;
}
