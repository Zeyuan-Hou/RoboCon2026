/**
  *****************************************************************************
  * @file    LWIP_UDP.h
  * @brief   UDP?????
  *****************************************************************************
  */

#ifndef __LWIP_UDP_H
#define __LWIP_UDP_H

#ifdef __cplusplus
extern "C" {
#endif

/* ????? */
#include "main.h"
#include "lwip/udp.h"
#include "global_declare.h"

/* ???? */
#define SERVER_PORT    8880  /* ???? */
#define REMOTE_PORT    8234  /* ???? */
////

/* ??????? */
#define UDP_RX_BUFFER_SIZE  1024
#define UDP_TX_BUFFER_SIZE  1024

/* ?????? */
extern uint8_t UDP_rx_data[UDP_RX_BUFFER_SIZE];
extern uint8_t UDP_tx_data[UDP_TX_BUFFER_SIZE];

///* ??????? */
//typedef struct {
//    uint32_t lwip_rec_cnt;
//    uint32_t lwip_send_cnt;
//} system_monitor_t;

//extern system_monitor_t system_monitor;

/* ???? */

//void udp_task_run(void);
void udp_task_init(void);
void Server_Receive_Callback(void *arg, struct udp_pcb *upcb, struct pbuf *p_rx_buf, 
                             const ip_addr_t *addr, u16_t port);
void udp_send_data(uint8_t *data, uint16_t len);
#ifdef __cplusplus
}
#endif

#endif /* __LWIP_UDP_H */
