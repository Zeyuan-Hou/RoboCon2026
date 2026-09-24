#include "udp_comm.h"

#include "lwip/init.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "netif/etharp.h"
#include "robot.h"
#include "system_monitor.h"

#define UDP_SERVER_PORT 5001 /* define the UDP local connection port */
#define UDP_CLIENT_PORT 1234 /* define the UDP remote connection port */

void udp_echoserver_receive_callback(void *arg, struct udp_pcb *upcb,
                                     struct pbuf *p, const ip_addr_t *addr,
                                     u16_t port);

// static void udp_demo_callback(void *arg,
//                              struct udp_pcb *upcb,
//                              struct pbuf *p,
//                              const ip_addr_t *addr,
//                              u16_t port);

uint8_t udp_rx_data[100] = {0};
uint8_t udp_tx_data[] = "hello world!";

udp_receive_data_t udp_receive_data = {.header = {0xFE, 0xEE}};
udp_send_data_t udp_send_data = {.header = {0xFE, 0xEE}, .state = SYS_IDLE};
// udp_send_data_t udp_send_data = {0};

static char *recv_arg = "I get a data\n";
void udp_echoserver_init(void)
{
    struct udp_pcb *upcb;
    err_t err;

    /* Create a new UDP control block  */
    upcb = udp_new();

    if (upcb)
    {
        /* Bind the upcb to the UDP_PORT port */
        /* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
        err = udp_bind(upcb, IP_ADDR_ANY, UDP_SERVER_PORT);

        if (err == ERR_OK)
        {
            /* Set a receive callback for the upcb */
            udp_recv(upcb, udp_echoserver_receive_callback, recv_arg);
        }
        else
        {
            udp_remove(upcb);
        }
    }
}

void udp_motor_type2raw_motor_type(udp_motor_command_t *udp_motor_command,
                                   motor_send_data_t *raw_motor_send)
{
    // raw_motor_send->mode = udp_motor_command->state;
    raw_motor_send->K_P = udp_motor_command->K_P;
    raw_motor_send->K_W = udp_motor_command->K_W;
    raw_motor_send->Pos = udp_motor_command->Pos;
    raw_motor_send->W = udp_motor_command->W;
    raw_motor_send->T = udp_motor_command->T;
}

void raw_motor_type2udp_motor_type(udp_motor_feedback_t *udp_motor_feedback,
                                   motor_receive_data_t *raw_motor_receive)
{
    udp_motor_feedback->Pos = raw_motor_receive->Pos;
    udp_motor_feedback->W = raw_motor_receive->W;
    udp_motor_feedback->Acc = raw_motor_receive->Acc;
    udp_motor_feedback->T = raw_motor_receive->T;
    udp_motor_feedback->Temp = raw_motor_receive->Temp;
    udp_motor_feedback->MError = raw_motor_receive->Error;
    ;
}

_Bool success_receive_flag = 0;
void udp_echoserver_receive_callback(void *arg, struct udp_pcb *upcb,
                                     struct pbuf *p_rx_buf,
                                     const ip_addr_t *addr, u16_t port)
{
    /* 处理接收 */
    system_monitor.temp_rate[UDP_ENTER]++;
    // clear the flag
    success_receive_flag = 0;
    // get motor command from PC
    if (p_rx_buf->len == sizeof(udp_receive_data) &&
            *(uint8_t *)p_rx_buf->payload == udp_receive_data.header[0] &&
            *((uint8_t *)p_rx_buf->payload + 1) == udp_receive_data.header[1] ||
        0)
    {
        memcpy(&udp_receive_data, p_rx_buf->payload, p_rx_buf->len);
        if (crc32_core((uint8_t *)&udp_receive_data,
                       sizeof(udp_receive_data) / 4 - 1) ==
                udp_receive_data.check_digit ||
            0)
        {
            success_receive_flag = 1;
            system_monitor.temp_rate[UDP_REAL]++;

            // TODO: 根据接收设置电机控制指令
            // set motor control command
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    udp_motor_type2raw_motor_type(
                        &udp_receive_data.udp_motor_command[i * 3 + j],
                        &robot.leg[i].motor[j].command);
                }
            }

            //            // if error occered, upper computer can not change
            //            system state if (system_monitor.system_state !=
            //            SYS_ERROR)
            if (1) // disable error detect function
            {
                // switch between NORMAL(enable all motor) and IDLE(disable all
                // motor)
                system_monitor.system_state = udp_receive_data.state;
                udp_send_data.state = system_monitor.system_state;

                set_robot_mode(udp_receive_data.state);
            }
        }
    }

    /* Free the buffer */
    pbuf_free(p_rx_buf);

    /* 发送反馈 */
    // send motor feedback to PC
    if (success_receive_flag)
    {
        /* Connect to the remote client */

        // udp_connect(upcb, addr, UDP_CLIENT_PORT);
        struct pbuf *p_tx_buf;

        // TODO: update motor feedback data
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                raw_motor_type2udp_motor_type(
                    &udp_send_data.udp_motor_feedback[i * 3 + j],
                    &robot.leg[i].motor[j].feedback);
                udp_send_data.udp_motor_feedback[i * 3 + j].communication_rate =
                    robot.leg[i].motor[j].real_rate;
            }
        }

        // add crc
        udp_send_data.check_digit = crc32_core((uint8_t *)&udp_send_data,
                                               sizeof(udp_send_data) / 4 - 1);

        p_tx_buf = pbuf_alloc(PBUF_TRANSPORT, sizeof(udp_send_data), PBUF_POOL);

        pbuf_take(p_tx_buf, &udp_send_data, sizeof(udp_send_data));
        // udp_send(upcb, p_tx_buf);

        udp_sendto(upcb, p_tx_buf, addr, UDP_CLIENT_PORT);

        pbuf_free(p_tx_buf);

        // /* free the UDP connection, so we can accept new clients */
        //        udp_disconnect(upcb);
    }
}

// static void udp_demo_callback(void *arg,
//                              struct udp_pcb *upcb,
//                              struct pbuf *p,
//                              const ip_addr_t *addr,
//                              u16_t port)
//  {
////     struct pbuf *q = NULL;
//
////     pbuf_free(p);

////     q = pbuf_alloc(PBUF_TRANSPORT, strlen(reply)+1, PBUF_RAM);

////     memset(q->payload, 0 , q->len);
////     memcpy(q->payload, reply, strlen(reply));
//     udp_sendto(upcb, p , addr, port);
////     pbuf_free(q);
//        pbuf_free(p);
// }
