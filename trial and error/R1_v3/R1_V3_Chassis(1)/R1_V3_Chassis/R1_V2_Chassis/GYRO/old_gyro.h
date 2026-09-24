#ifndef __OLD_GYRO_H__
#define __OLD_GYRO_H__

#include <string.h> 
#include "Types.h"



#define FRAME_HEAD1     0xAA
#define FRAME_HEAD2     0x55
#define FRAME_TAIL1     0x55
#define FRAME_TAIL2     0xAA
#define DATA_BYTE_CNT   12      // 3个float，固定12字节
#define FULL_FRAME_LEN  17      // 总帧长：2头+12数据+1校验+2尾 =17字节

/* USER CODE BEGIN PFP */
typedef __packed struct {
          float roll;         // 横滚角 (°)，绕X轴
    float pitch;        // 俯仰角 (°)，绕Y轴
    float yaw;          // 偏航角 (°)，绕Z轴（仅陀螺仪积分，无绝对参考）
} ADIS16470_FinalData;

extern ADIS16470_FinalData final_data;

uint8_t UART_Parse_Frame(uint8_t *buf, uint16_t len);



#endif
