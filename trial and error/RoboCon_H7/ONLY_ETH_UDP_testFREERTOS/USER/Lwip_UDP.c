/**
 *****************************************************************************
 * @file    LWIP_UDP.c
 * @brief   UDP????(???)
 *****************************************************************************
 */

#include "LWIP_UDP.h"
#include <string.h>
#include "lwip/udp.h"

/* data for send and receive */
uint8_t UDP_rx_data[UDP_RX_BUFFER_SIZE] = {0};
uint8_t UDP_tx_data[UDP_TX_BUFFER_SIZE] = {0};

/* remote address */
static struct udp_pcb *udpecho_pcb;
static err_t err;
static uint8_t REMOTE_ADDRESS[4] = {192, 168, 10, 3}; //

/* other vairables */
static struct udp_pcb *my_udp_pcb = NULL;
ip_addr_t DestIPaddr;

/* --------------------------- LWIP_UDP.c --------------------------- */

/**
 * @brief  call back when receive
 * WARNING: block functions will delay the time for receive, thus fps declines
 * WARNING: no "print",no "delay"
 */
void Server_Receive_Callback(void *arg, struct udp_pcb *upcb, struct pbuf *p,
                             const ip_addr_t *addr, u16_t port)
{
    if (p != NULL)
    {
        monitor.rate_cnt.udp_recv++;
        /* disable DCache */
        SCB_CleanInvalidateDCache();
        memset(UDP_rx_data, 0, UDP_RX_BUFFER_SIZE);
        uint16_t len = (p->tot_len > UDP_RX_BUFFER_SIZE) ? UDP_RX_BUFFER_SIZE : p->tot_len;
        pbuf_copy_partial(p, UDP_rx_data, len, 0);

        pbuf_free(p);

        udp_unpack();
    }
}

/**
 * @brief  init function in freertos
 */
void udp_task_init(void)
{
    if (my_udp_pcb != NULL)
        return;

    my_udp_pcb = udp_new();
    if (my_udp_pcb != NULL)
    {
        IP4_ADDR(&DestIPaddr, 192, 168, 10, 3); //
        err_t err = udp_bind(my_udp_pcb, IP_ADDR_ANY, SERVER_PORT);
        if (err == ERR_OK)
        {
            udp_recv(my_udp_pcb, Server_Receive_Callback, NULL);
        }
    }
}

void udp_send_data(uint8_t *data, uint16_t len)
{
    if (my_udp_pcb == NULL)
        return;

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_POOL);

    if (p != NULL)
    {
        pbuf_take(p, data, len);

        /* disable DCache */
        // SCB_CleanInvalidateDCache();//
        SCB_CleanDCache_by_Addr((uint32_t *)p->payload, p->len);
        // 两个都是清理dcache的函数。第一个操作整个L1cache
        // 第二个地址上更具有针对性
        // 如果不清除清除cache，这样读取到的一定是dma写入sram的数据；
        // 否则可能读到还未更新的cache内数据而产生什么问题

        udp_sendto(my_udp_pcb, p, &DestIPaddr, REMOTE_PORT);
        pbuf_free(p);

        monitor.rate_cnt.udp_send++;
    }
}
/******************************** END OF FILE ********************************/
