#ifndef REMOTE_CONTROL_TASK_H__
#define REMOTE_CONTROL_TASK_H__

#include "spi.h"
#include "algorithm.h"
#include "path_algorithm_spot.h"
#include "Location_Task.h"




#define LEFT_JS_X_MID 0x084C
#define LEFT_JS_X_MAX 0x0FDD
#define LEFT_JS_X_MIN 0x0010 
#define LEFT_JS_Y_MID 0x07F1 
#define LEFT_JS_Y_MAX 0x0F25
#define LEFT_JS_Y_MIN 0x0025
#define RIGHT_JS_MID 0x07DA 
#define RIGHT_JS_MIN 0x0005
#define RIGHT_JS_MAX 0x0FF6







void parseDataPacket(const uint8_t Tx_Buf[32], ST_JS_VALUE *Js_Value);

void CalculateVelocities(const ST_JS_VALUE *jsValue, ST_Nav *p_nav,
                         uint16_t ucGateX, int16_t ssXSpedLimit,
                         uint16_t ucGateY, int16_t ssYSpedLimit,
                         uint16_t ucGateW, int16_t ssWSpedLimit);
void pack_data(uint8_t buffer[]);
//void monitor(void);
#endif



