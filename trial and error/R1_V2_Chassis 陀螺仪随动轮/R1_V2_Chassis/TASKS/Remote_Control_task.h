#ifndef REMOTE_CONTROL_TASK_H__
#define REMOTE_CONTROL_TASK_H__

#include "spi.h"
#include "algorithm.h"
#include "path_algorithm_spot.h"
#include "Location_Task.h"




#define LEFT_JS_X_MID 1892
#define LEFT_JS_X_MAX 4076
#define LEFT_JS_X_MIN 4 
#define LEFT_JS_Y_MID 1960 
#define LEFT_JS_Y_MAX 4032
#define LEFT_JS_Y_MIN 5
#define RIGHT_JS_MID 1986 
#define RIGHT_JS_MIN 4
#define RIGHT_JS_MAX 4093







void parseDataPacket(const uint8_t Tx_Buf[32], ST_JS_VALUE *Js_Value);

void CalculateVelocities(const ST_JS_VALUE *jsValue, ST_Nav *p_nav,
                         uint16_t ucGateX, int16_t ssXSpedLimit,
                         uint16_t ucGateY, int16_t ssYSpedLimit,
                         uint16_t ucGateW, int16_t ssWSpedLimit);
void pack_data(uint8_t buffer[]);
//void monitor(void);
#endif



