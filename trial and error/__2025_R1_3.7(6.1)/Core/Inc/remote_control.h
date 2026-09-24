#ifndef __REMOTE_CONTROL_H
#define __REMOTE_CONTROL_H

#include "stdint.h"
#include "ROBOT.h"
#include "path_algorithm.h"
#include "air_operated.h"
#include "bsp_can.h"
#include "algorithm.h"
#include "NRF24L01.h"
#include "pid.h"
// 中间位置常量（实际应用中应根据具体情况调整，标定方式：摇杆不动，通过debug  watch看一下摇杆位置值填入下面）
void parseDataPacket(const uint8_t Rx_Buf[32], ST_JS_VALUE *jsValue);
void CalculateVelocities(const ST_JS_VALUE *jsValue, ST_Nav *p_nav,
                         uint16_t ucGateX, int16_t ssXSpedLimit,
                         uint16_t ucGateY, int16_t ssYSpedLimit,
                         uint16_t ucGateW, int16_t ssWSpedLimit);
void RC_switch(ST_RC_CTRL *pst_rc_ctrl);
void Read_Key_Task(void);//读取键值改变Nav_State，分配任务
void Vision_RC(void);//读取视觉键值
void send_remote_control(void);

#endif


