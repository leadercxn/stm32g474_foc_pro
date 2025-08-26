#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include "stdio.h"
#include "string.h"

#include "boards.h"
#include "uart.h"
#include "app_fifo.h"
#include "lib_error.h"
#include "app_timer.h"

#include "trace.h"

#define USART1_RX_TIMEOUT       6

static UART_HandleTypeDef m_huart1_handle;

static uint8_t      m_rx_data; // HAL库使用的串口接收缓冲
static app_fifo_t   m_usart1_rx_fifo;               // rx 的信息fifo
static uint8_t      m_usart1_rx_msg[256];           // fifo 长度
static uint8_t      m_is_rx1_done = 0;              // 假如使用bool类型，接收中断不受控，很奇怪！！！！先接收完成，再到超时中断

TIMER_DEF(m_rx1_timer);

static void usart1_timer_cb(void)
{
    m_is_rx1_done = 1;
}

int usart1_init(void)
{
    int    err_code = 0;

    m_huart1_handle.Instance        = USART1;               /* USART1 */
    m_huart1_handle.Init.BaudRate   = USART1_BAUDRATE;      /* 波特率 */
    m_huart1_handle.Init.WordLength = UART_WORDLENGTH_8B;   /* 字长为8位数据格式 */
    m_huart1_handle.Init.StopBits   = UART_STOPBITS_1;      /* 一个停止位 */
    m_huart1_handle.Init.Parity     = UART_PARITY_NONE;     /* 无奇偶校验位 */
    m_huart1_handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;  /* 无硬件流控 */
    m_huart1_handle.Init.Mode       = UART_MODE_TX_RX;      /* 收发模式 */
    HAL_UART_Init(&m_huart1_handle);                        /* HAL_UART_Init()会使能UART1 */
    
    /* 该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量 */
    HAL_UART_Receive_IT(&m_huart1_handle, (uint8_t *)&m_rx_data, sizeof(m_rx_data));

    err_code = app_fifo_init(&m_usart1_rx_fifo, m_usart1_rx_msg, sizeof(m_usart1_rx_msg));
    if(err_code != ENONE)
    {
        return err_code;
    }

    //定义一个接收超时定时器
    TIMER_CREATE(&m_rx1_timer, true, true, usart1_timer_cb);

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

int usart1_rx(uint8_t *p_rx_data)
{
    if(p_rx_data == NULL)
    {
        return -HAL_ERROR;
    }

    uint16_t len = 0;

    if(m_is_rx1_done == 1)
    {
        m_is_rx1_done = 0;

        if(p_rx_data)
        {
            len = fifo_length(&m_usart1_rx_fifo);
            app_fifo_gets(&m_usart1_rx_fifo, p_rx_data, len);
        }
    }

    return len;
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
        HAL_UART_Receive_IT(&m_huart1_handle, (uint8_t *)&m_rx_data, sizeof(m_rx_data));

        TIMER_STOP(m_rx1_timer);

        if(m_is_rx1_done == 1)                                  //前一帧数据还没读走，清走， 用 m_is_rx5_done 就进入不了，
        {
            m_is_rx1_done = 0;
            app_fifo_flush(&m_usart1_rx_fifo);
        }

        app_fifo_puts(&m_usart1_rx_fifo, &m_rx_data, 1);         // 接收到的数据入列

        TIMER_START(m_rx1_timer, USART1_RX_TIMEOUT);
    }
}

/**
 * 串口硬件IO初始化，内部调用
 */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef gpio_init_struct = {0};

  if(huart->Instance == USART1)
  {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio_init_struct.Pin        = USART1_TX_PIN | USART1_RX_PIN;
    gpio_init_struct.Mode       = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull       = GPIO_NOPULL;
    gpio_init_struct.Speed      = GPIO_SPEED_FREQ_LOW;
    gpio_init_struct.Alternate  = USART1_TX_AF;
  
    HAL_GPIO_Init(USART1_TX_PORT, &gpio_init_struct);

    HAL_NVIC_EnableIRQ(USART1_IRQn);                      /* 使能USART1中断通道 */
    HAL_NVIC_SetPriority(USART1_IRQn, 3, 3);              /* 抢占优先级3，子优先级3 */
  }
}

/**
 * 串口硬件IO恢复默认值，内部调用
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART1)
  {
    __HAL_RCC_USART1_CLK_DISABLE();

    HAL_GPIO_DeInit(USART1_TX_PORT, USART1_TX_PIN | USART1_RX_PIN);
  }

}

