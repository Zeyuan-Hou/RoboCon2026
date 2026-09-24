#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__

#include "global_declare.h"
#include "algorithm.h"
#include "gyro.h"
#include <math.h>

#define YOU_TRY 0
// #define LEFT_JS_X_MID 0x088E
// #define LEFT_JS_X_MAX 0x0FF9
// #define LEFT_JS_X_MIN 0x0002
// #define LEFT_JS_Y_MID 0x0801
// #define LEFT_JS_Y_MAX 0x0004
// #define LEFT_JS_Y_MIN 0x0FF9
// #define RIGHT_JS_MID 0x07F3
// #define RIGHT_JS_MIN 0x0004
// #define RIGHT_JS_MAX 0x0FFB

#define LEFT_JS_X_MID 0x0809
#define LEFT_JS_X_MAX 0x0FFC
#define LEFT_JS_X_MIN 0x004
#define LEFT_JS_Y_MID 0x076E
#define LEFT_JS_Y_MAX 0x0FFA
#define LEFT_JS_Y_MIN 0x004
#define RIGHT_JS_MID 0x08A3
#define RIGHT_JS_MIN 0x005
#define RIGHT_JS_MAX 0x0FFC

void parseDataPacket(const uint8_t Rx_Buf[32], ST_JS_VALUE *jsValue);
void CalculateVelocities(const ST_JS_VALUE *jsValue, ST_Nav *p_nav, uint16_t thres_velx, int16_t max_velx, uint16_t thres_vely, int16_t max_vely, uint16_t thres_velw, int16_t max_velw);
void pack_data(uint8_t buffer[]);

#endif
