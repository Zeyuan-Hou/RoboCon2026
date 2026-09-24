#ifndef __SERVO_H
#define __SERVO_H

#include "ROBOT.h"
#include "math_algorithm.h"
#include "stdlib.h"
#include "usart.h"
#include "cmsis_os.h"

extern DMA_HandleTypeDef hdma_uart4_tx;

void servo_state_control(void);
void Get_Serve_Pos(void);
uint8_t servo_judge(void);
void servo_deal(void);

void Read_SCS(uint8_t ID, uint8_t Cmd, uint8_t Address, 
              uint8_t ReadSize, int8_t *buf);
void Write_Pos_SCS(uint8_t ID, Servo* Servo,  int8_t* buf);
#endif

