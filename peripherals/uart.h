#ifndef UART_H__
#define UART_H__

int usart1_init(void);

int usart1_tx(uint8_t *p_tx_data, uint16_t len);
int usart1_rx(uint8_t *p_rx_data, uint16_t len);

#endif
