#ifndef __BOARD_COMMUNICATE_H__
#define __BOARD_COMMUNICATE_H__

#include "global_declare.h"
#include "remote_control.h"
#include "chassis.h"
#include "usart.h"

//是否完成任务标志位
extern uint8_t flag_up_step; 		//上台阶
extern uint8_t flag_down_step;   //下台阶
extern uint8_t flag_up_R1;       //上R1
//与另一辆车通讯
extern uint8_t flag_D_init;        //D初始化
extern uint8_t flag_D_ready_get_head;//D准备取武器头
extern uint8_t flag_D_get_left;    //D左臂取武器头
extern uint8_t flag_D_get_right;    //D右臂取武器头
extern uint8_t flag_D_get_block;   //D准备存S给的方块
extern uint8_t flag_D_3_init; 
 
extern uint8_t flag_S_init;             //S初始化
extern uint8_t flag_S_get_block_down;   //S取下层方块
extern uint8_t flag_S_get_block_up;     //S取上层方块
extern uint8_t flag_S_get_block_top;    //S取上上层方块
extern uint8_t flag_S_throw_block_back; //S向后扔方块
extern uint8_t flag_S_throw_block_front;//S向前扔方块
extern uint8_t flag_S_give_D;           //S给D方块
extern uint8_t flag_S_get_D_block;
extern uint8_t flag_S_put_block_middle; //S三区放中层方块
extern uint8_t flag_S_put_block_top;    //S三区放上层方块
extern bool flag_update_tx;

void Upper_Lower_Communication(void);
void Upper_Data_Recieve_Deal(void) ;
void updateTxData(void);
uint16_t crc16(uint8_t *data, uint16_t length);
#endif
