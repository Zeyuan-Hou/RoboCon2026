#include "LWIP_UDP_FreeRTOS.h"
#include <string.h>
#include "main.h"
#include "lwip/udp.h"
#include "FreeRTOS.h"
#include "task.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//#define UDP_SERVER_PORT 9000 /* define the UDP local connection port */
//#define UDP_CLIENT_PORT 9001 /* define the UDP remote connection port */

#define SERVER_PORT 9001 // 端口号
#define REMOTE_PORT 9000 // 端口号

uint8_t REMOTE_ADDRESS[4] = {192,168,2,2};
//uint8_t REMOTE_ADDRESS[4] = {169,254,239,188};

struct udp_pcb *udpecho_pcb;
 err_t err;

void udp_task_run(void) 
{
    struct pbuf *udp_tx_buf;

    /*创建一个pbuf数据包*/
   

    /* Create a new UDP control block  */
    udpecho_pcb = udp_new();

    if (udpecho_pcb)
    {
        /* Bind the upcb to the UDP_PORT port */
        /* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
        
        /* 绑定本地端口号 */
        err = udp_bind(udpecho_pcb, IP_ADDR_ANY, SERVER_PORT);
        
        /*设置远端连接信息*/
        IP4_ADDR(&udpecho_pcb->remote_ip, REMOTE_ADDRESS[0], REMOTE_ADDRESS[1], REMOTE_ADDRESS[2], REMOTE_ADDRESS[3]);
        udpecho_pcb->remote_port = REMOTE_PORT;
        
        if (err == ERR_OK)
        {
                /* Set a receive callback for the upcb */
                udp_recv(udpecho_pcb, Server_Receive_Callback, NULL);
        }
        else
        {
                udp_remove(udpecho_pcb);
        } 
    }

    while (1) 
    {
//        udp_tx_buf = pbuf_alloc(PBUF_TRANSPORT, sizeof(UDP_tx_data), PBUF_POOL);
//        pbuf_take(udp_tx_buf, UDP_tx_data, sizeof(UDP_tx_data));
//        udp_send(udpecho_pcb, udp_tx_buf);
//        pbuf_free(udp_tx_buf);
//				system_monitor.lwip_send_cnt++;
        vTaskDelay(1);
    }
}
void Server_Receive_Callback(void *arg, struct udp_pcb *upcb, struct pbuf *p_rx_buf, const ip_addr_t *addr, u16_t port)
{
    if(p_rx_buf->payload != NULL)
    {
				//数据接收
        memcpy(UDP_rx_data, p_rx_buf->payload, p_rx_buf->len);
			
				system_monitor.lwip_rec_cnt++;
			
        //绑定远程客户端IP地址
        udp_connect(upcb, addr, REMOTE_PORT);
       
				//定义发送数据的缓冲区，并申请空间，COPY数据到缓冲区（自己定义要发送的全局变量数据）
				struct pbuf *p_tx_buf;
				p_tx_buf = pbuf_alloc(PBUF_TRANSPORT, sizeof(UDP_tx_data), PBUF_POOL);
				pbuf_take(p_tx_buf, UDP_tx_data, sizeof(UDP_tx_data));
				udp_send(upcb, p_tx_buf);
				system_monitor.lwip_send_cnt++;
				//释放发送缓冲区,并释放UDP连接以便接受新的客户端
				pbuf_free(p_tx_buf);
        udp_disconnect(upcb);
        
    }
    //释放接收缓冲区
    pbuf_free(p_rx_buf);

}

