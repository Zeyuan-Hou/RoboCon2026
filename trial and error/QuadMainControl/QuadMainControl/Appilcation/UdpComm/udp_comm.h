#ifndef __UDP_COMM_H__
#define __UDP_COMM_H__

#include "main.h"
#include "unitree_motor.h  "

#define LEG_NUM 4

// motor command
typedef struct {
    // uint8_t mode;
    float T;
    float W;
    float Pos;
    float K_P;
    float K_W;
} __attribute__((packed)) udp_motor_command_t;

// motor feedback
typedef struct {
    float Pos;
    float W;
    float Acc;
    float T;
    float Temp;
    uint32_t communication_rate;
    float MError;
} __attribute__((packed)) udp_motor_feedback_t;

// send to PC throught UDP
typedef struct {
    uint8_t header[2];
    uint8_t state;
    udp_motor_feedback_t udp_motor_feedback[3 * LEG_NUM];
    uint32_t check_digit;
} __attribute__((packed)) udp_send_data_t;

// receive from PC throught UDP
typedef struct {
    uint8_t header[2];
    uint8_t state;
    udp_motor_command_t udp_motor_command[3 * LEG_NUM];
    uint32_t check_digit;
} __attribute__((packed)) udp_receive_data_t;

extern void udp_echoserver_init(void);
extern udp_receive_data_t udp_receive_data;
extern udp_send_data_t udp_send_data;

#endif /* __UDP_COMM_H__ */
