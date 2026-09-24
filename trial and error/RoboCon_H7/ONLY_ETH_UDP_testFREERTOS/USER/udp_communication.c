#include "udp_communication.h"

void udp_pack()
{
    UDP_tx_data[0] = 0x01;

    memcpy(&UDP_tx_data[1], &monitor.rate_fps.udp_recv, sizeof(uint16_t));
    memcpy(&UDP_tx_data[3], &monitor.rate_fps.udp_send, sizeof(uint16_t));

    UDP_tx_data[5] = 0x02;
}

void udp_unpack()
{
    if (UDP_rx_data[0] == 0x02 && UDP_rx_data[5] == 0x01)
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
    }
}
