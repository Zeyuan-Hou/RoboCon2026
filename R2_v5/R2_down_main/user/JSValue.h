#ifndef JSVALUE_H
#define JSVALUE_H


#include "stdint.h"
#include "Type.h"
#include "math_algorithm.h"
#include "up_down_step.h"
#include "path.h"

//遥控器摇杆值

#define LEFT_JS_X_MID 2245 
#define LEFT_JS_X_MAX 4090 
#define LEFT_JS_X_MIN 70 
#define LEFT_JS_Y_MID 1990 
#define LEFT_JS_Y_MAX 4082 
#define LEFT_JS_Y_MIN 4
#define RIGHT_JS_MID 2068 
#define RIGHT_JS_MIN 6
#define RIGHT_JS_MAX 4075

// 声明ST_JS_VALUE结构体
typedef struct
{
    uint16_t usJsKey;        // 独立+矩阵按键
    uint16_t usJsLeft_X;     // 左摇杆x方向
    uint16_t usJsLeft_Y;     // 左摇杆y方向
    uint16_t usJsRight_X;    // 右摇杆x方向
    uint16_t usJsRight_Y;    // 右摇杆y方向
} ST_JS_VALUE;

extern ST_JS_VALUE Js_Value;
extern uint8_t nRF24L01_RxBuf[9];
extern uint8_t remote_rec_uart[11];

// 声明解算函数
void parseDataPacket(const uint8_t Rx_Buf[9], ST_JS_VALUE *jsValue);
void CalculateVelocities(const ST_JS_VALUE *jsValue, ST_VECTOR *p_nav,
                         uint16_t ucGateX, int16_t ssXSpedLimit,
                         uint16_t ucGateY, int16_t ssYSpedLimit,
                         uint16_t ucGateW, int16_t ssWSpedLimit);

//遥控标志位，不用遥控器时保持清零
void remote_deal(uint8_t Rx_Buf[9],uint8_t remote_rec_uart[11]);
void adjust_area(void);
void remote_send_(void);
void check_fps(void);
//void path_1_remote(void)

#endif  
