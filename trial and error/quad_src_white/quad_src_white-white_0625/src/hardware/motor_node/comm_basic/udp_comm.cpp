/**
 * @file udp_comm.cpp
 * @brief UDP communication interface
 * @author Haoyu Wang (qrpucp@qq.com)
 * @date 2022-09-19
 *
 * @copyright Copyright (C) 2022.
 *
 */
/* related header files */
#include "udp_comm.h"

/* c system header files */

/* c++ standard library header files */

/* external project header files */

/* internal project header files */
// #include "basic_math.hpp"

/**
 * @brief 32-bit CRC function
 * @param[in] src data address
 * @param[in] len data length
 * @return 32-bit CRC value
 */


uint32_t crc32Core(volatile uint8_t *src, uint32_t len)
{
    auto *ptr = (uint32_t *)src;
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }
    return CRC32;
}

/**
 * @brief Construct a new Udp Comm:: Udp Comm object
 * @param[in] local_port UDP local port
 * @param[in] target_IP UDP target IP
 * @param[in] target_port UDP target port
 */
UdpComm::UdpComm(uint16_t local_port, const std::string &target_ip, uint16_t target_port)
    : local_port_(local_port), target_port_(target_port), target_ip_(target_ip)
{
    // set headers
    udp_receive_data_.header[0] = udp_send_data_.header[0] = 0xFE;
    udp_receive_data_.header[1] = udp_send_data_.header[1] = 0xEE;
}


 UdpComm::~UdpComm(void)
 {
    udp_send_data_.state = 0x00;
    setSendData(udp_send_data_);
    send();

 }

/**
 * @brief initialize UDP communication
 * @return whether the initialization is successful
 */
bool UdpComm::init(void)
{
    try
    {
        p_client_ = std::make_unique<udp_client_server::udp_client>(target_ip_, target_port_);
        p_server_ = std::make_unique<udp_client_server::udp_server>("0.0.0.0", local_port_);
    }
    catch (std::runtime_error &error)
    {
         std::cout<< "try to launch the program with super user permissions";
        return false;
    }
    return true;
}

/**
 * @brief receive data through UDP
 * @param[in] max_wait_ms the waiting time for non-blocking receive
 * @return if successfully receives data
 */
bool UdpComm::receive(int32_t max_wait_ms)
{
    int32_t receive_len;
    if (max_wait_ms < 0) // blocking
    {
        receive_len = p_server_->recv((char *)receive_buffer_, sizeof(receive_buffer_));
    }
    else // non-blocking
    {
        receive_len =
            p_server_->timed_recv((char *)receive_buffer_, sizeof(receive_buffer_), max_wait_ms);
    }
    if (receive_len == -1)
    {
        std::cout<<"[udp receive]recv len = -1 err"<<std::endl;
        return false; // non-blocking, no data to receive
    }
    else if (receive_len == sizeof(udp::ReceiveData) &&
             receive_buffer_[0] == udp_receive_data_.header[0] &&
             receive_buffer_[1] == udp_receive_data_.header[1]) // judge length and header
    {
        uint32_t check_digit = *(uint32_t *)&receive_buffer_[receive_len - 4];
        if (check_digit == crc32Core(receive_buffer_, receive_len / 4 - 1))
        {
            memcpy(&udp_receive_data_, receive_buffer_, receive_len);
            return true; // data received successfully
        }

        std::cout<<"[udp receive]crc err"<<std::endl;
        return false; // incorrect data received
    }
    else
    {
        std::cout<<receive_len<<std::endl;
        std::cout<<sizeof(udp::ReceiveData)<<std::endl;
        std::cout<<"[udp receive]lost data err"<<std::endl;
        return false; // lost data
    }
}

/**
 * @brief send data through UDP
 * @return if successfully sends data
 */
bool UdpComm::send(void)
{
    return p_client_->send((char *)&udp_send_data_, sizeof(udp_send_data_)) > 0;
}

/**
 * @brief set data to be sent
 * @param[in] data data address
 */
void UdpComm::setSendData(const udp::SendData &data)
{
    // do not copy head and crc digit, protocol depended
    memcpy((uint8_t *)&udp_send_data_ + 2, (uint8_t *)&data + 2, sizeof(udp_send_data_) - 6);
    // udp_send_data_ = data;
    // add crc
    udp_send_data_.check_digit =
        crc32Core((uint8_t *)&udp_send_data_, sizeof(udp_send_data_) / 4 - 1);
}

/**
 * @brief get the received data
 * @return the received data
 */
const udp::ReceiveData &UdpComm::getReceiveData(void) const
{
    return udp_receive_data_;
}



