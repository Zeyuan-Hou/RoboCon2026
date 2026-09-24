#ifndef __HIPNUC_GYRO_H__
#define __HIPNUC_GYRO_H__

#include "hipnuc_dec.h"
#include "Types.h"
#include "usart.h"
#include "dma.h"


#define UART_RX_BUF_SIZE (1024)
#define LOG_STRING_SIZE (1024)

/* IMU stream read/control struct */
extern  hipnuc_raw_t hipnuc_raw ;


/* 0: no new data arrived, 1: new data arrived */
extern  uint8_t new_data_flag;

/* The char buffer used to show result */
extern  char log_buf[LOG_STRING_SIZE];

extern  uint8_t uart_rx_buf[UART_RX_BUF_SIZE];
extern  uint8_t dma_rx_buf[UART_RX_BUF_SIZE];
extern  uint16_t uart_rx_index; //接收数据计数器

extern void app_init(void);
extern void process_data(void);
extern void handle_usart_rx_idle(UART_HandleTypeDef *huart, uint16_t rx_size);

#endif /* __HIPNUC_GYRO_H__ */
