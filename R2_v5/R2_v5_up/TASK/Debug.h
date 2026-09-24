#ifndef __DEBUG_H__
#define __DEBUG_H__

#include <math.h>
#include "Global_Variables.h"
#include "Algorithm.h"
#include "Robstride.h"

void J60_param_set(uint8_t mode);
void DM_param_set(uint8_t mode);
void A1_param_set(uint8_t mode);
void M3508_param_set(uint8_t mode);
void M2006_param_set(uint8_t mode);
void RS_param_set(uint8_t mode);

void Observation(void);
uint16_t FPS_Monitor(void);

#define CH_COUNT 50   // 通道数 即发送数据的个数
#pragma pack(push, 1) // 强制1字节对齐
typedef struct
{
    float ch_data[CH_COUNT];
    unsigned char tail[4];
} VOFA_DATA;
#pragma pack(pop) // 恢复默认对齐

extern VOFA_DATA vofa_data;
extern float vofa[50];
void VOFA_transmit_data(float *data, uint8_t num);
void VOFA_load_data(void);

uint8_t crc8_calc(uint8_t *data, uint8_t len);

#endif /* __DEBUG_H__ */
