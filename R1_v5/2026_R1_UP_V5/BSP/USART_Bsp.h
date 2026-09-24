#ifndef __USART_BSP_H__
#define __USART_BSP_H__
#include "Robot.h"

// Ò£¿ØÆ÷Ò¡¸Ë
#define LEFT_JS_X_MID 2010
#define LEFT_JS_X_MAX 4023
#define LEFT_JS_X_MIN 15//0x0FF6
#define LEFT_JS_Y_MID 1850
#define LEFT_JS_Y_MAX 3727
#define LEFT_JS_Y_MIN 10
#define RIGHT_JS_MID 1952
#define RIGHT_JS_MIN 1
#define RIGHT_JS_MAX 4029

void packDataToLower(u8* data);
u8 unpackDataFromLower(u8* data);

u8 unpackVisionData(u8* data);
void packVisionData(u8* data);

void parseDataPacket(const uint8_t Rx_Buf[9], ST_JS_VALUE *jsValue);
void CalculateVelocities(const ST_JS_VALUE *jsValue, ST_VEL *vel,
                         uint16_t ucGateX, int16_t ssXSpedLimit,
                         uint16_t ucGateY, int16_t ssYSpedLimit,
                         uint16_t ucGateW, int16_t ssWSpedLimit);
void search_KFS_information(void);
void DealKey(uint8_t key);
void DealKeyTemp(uint8_t key);
void packRemoteData(u8* data);
void packDataToIRModule(u8 *buf);
#endif
