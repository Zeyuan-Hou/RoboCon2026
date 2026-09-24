#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__

#include "global_declare.h"
#include "algorithm.h"
#include "NRF24L01.h"
#include "board_communicate.h"
void parseDataPacket(const uint8_t Rx_Buf[32], ST_JS_VALUE *jsValue);
void Send_To_RemoteControl(void);
void CalculateVelocities(const ST_JS_VALUE *jsValue, ST_Nav *p_nav,
                         uint16_t ucGateX, int16_t ssXSpedLimit,
                         uint16_t ucGateY, int16_t ssYSpedLimit,
                         uint16_t ucGateW, int16_t ssWSpedLimit);
void Deal_Key_State(const ST_JS_VALUE *jsValue);



#endif
