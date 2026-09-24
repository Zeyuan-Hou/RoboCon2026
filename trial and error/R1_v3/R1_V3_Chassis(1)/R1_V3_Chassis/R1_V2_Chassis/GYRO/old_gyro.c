#include "old_gyro.h"

ADIS16470_FinalData final_data={0};

uint8_t UART_Parse_Frame(uint8_t *buf, uint16_t len)//放在接收DMA的中断中
{
    uint16_t pos = 0;
    // 循环查找完整帧
    while(pos <= len - FULL_FRAME_LEN)
    {
        // 1. 匹配帧头
        if(buf[pos] == FRAME_HEAD1 && buf[pos+1] == FRAME_HEAD2)
        {
            // 2. 匹配帧尾
            if(buf[pos+15] == FRAME_TAIL1 && buf[pos+16] == FRAME_TAIL2)
            {
                // 3. 校验和检查
                uint8_t check_sum = 0;
                for(int i=0; i<DATA_BYTE_CNT; i++)
                {
                    check_sum += buf[pos+2 + i];
                }
                if(check_sum == buf[pos+14])
                {
                    // 4. 拷贝解析后的数据到结构体
                    memcpy(&final_data, &buf[pos+2], DATA_BYTE_CNT);
                    return 1; // 解析成功
                }
            }
        }
        pos++;
    }
    return 0; // 解析失败
}
