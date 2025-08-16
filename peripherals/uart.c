#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include "stdio.h"

#include "boards.h"
#include "uart.h"

#define USART1_RX_BUFFER_SZ     1

static UART_HandleTypeDef m_huart1_handle;
static uint8_t m_rx_buffer[USART1_RX_BUFFER_SZ];                  /* HAL库使用的串口接收缓冲 */

int usart1_init(void)
{
    m_huart1_handle.Instance        = USART1;               /* USART1 */
    m_huart1_handle.Init.BaudRate   = USART1_BAUDRATE;      /* 波特率 */
    m_huart1_handle.Init.WordLength = UART_WORDLENGTH_8B;   /* 字长为8位数据格式 */
    m_huart1_handle.Init.StopBits   = UART_STOPBITS_1;      /* 一个停止位 */
    m_huart1_handle.Init.Parity     = UART_PARITY_NONE;     /* 无奇偶校验位 */
    m_huart1_handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;  /* 无硬件流控 */
    m_huart1_handle.Init.Mode       = UART_MODE_TX_RX;      /* 收发模式 */
    HAL_UART_Init(&m_huart1_handle);                        /* HAL_UART_Init()会使能UART1 */
    
    /* 该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量 */
    HAL_UART_Receive_IT(&m_huart1_handle, (uint8_t *)m_rx_buffer, USART1_RX_BUFFER_SZ);

    return 0;
}

int usart1_tx(uint8_t *p_tx_data, uint16_t len)
{
    if(p_tx_data == NULL)
    {
        return -HAL_ERROR;
    }

    return HAL_UART_Transmit(&m_huart1_handle, p_tx_data, len, 10);    //超时先写个demo
}

int usart1_rx(uint8_t *p_rx_data, uint16_t len)
{
    if(p_rx_data == NULL)
    {
        return -HAL_ERROR;
    }

    return HAL_UART_Receive_r(&m_huart1_handle, p_rx_data, len, 10);     //超时先写个demo
}


void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&m_huart1_handle);
}

/**
 * @brief       Rx传输回调函数
 * @param       huart: UART句柄类型指针
 * @retval      无
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)             /* 如果是串口1 */
    {
        /* 接收完成后在开启写一次中断接收 */
        HAL_UART_Receive_IT(&m_huart1_handle, (uint8_t *)m_rx_buffer, USART1_RX_BUFFER_SZ);
    }
}