#ifndef UART_H__
#define UART_H__

int usart1_init(void);
int usart1_tx(uint8_t *p_tx_data, uint16_t len);
int usart1_rx(uint8_t *p_rx_data);

int usart3_init(void);
int usart3_tx(uint8_t *p_tx_data, uint16_t len);
int usart3_rx(uint8_t *p_rx_data);
#endif
