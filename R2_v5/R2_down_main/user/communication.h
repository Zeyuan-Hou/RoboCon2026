#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_
#include "Type.h"
#include "usart.h"
#include "math_algorithm.h"
#include "tim.h"


#define CODE0 115
#define CODE1 229
#define CODEReset 0

#define LED_Count 15


//µÆ´øTIM
void WS2812_SET(uint8_t index,uint8_t r,uint8_t g,uint8_t b);
void WS2812_AllSet(uint8_t r,uint8_t g,uint8_t b);
void WS2812_Update(void);

//´®¿Ú
void Vision_Data_Deal(vision_Data_t *p_vision_data);
void usart_inner_send(void);
void usart_inner_receive(void);
void usart_all_ctrl(void);
void usart_all_ctrl_send(void);
float GET_YAW(float gyro_yaw,float radar_yaw,float gyro_w);
void choose_pid_yaw(void);
void vision_send_task(void);
#endif
