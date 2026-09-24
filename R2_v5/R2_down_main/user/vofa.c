#include "vofa.h"
#include "string.h"
VOFA_DATA vofa_data;

float vofa[25]; 

const uint8_t FRAME_TAIL[4] = {0x00, 0x00, 0x80, 0x7F}; // 固定终止符

//使用justfloat协议将数据发送至vofa+上位机
void VOFA_transmit_data(float *data, uint8_t num)
{
	memcpy(vofa_data.ch_data, data, num * sizeof(float));
	
	
	// 明确填充剩余通道为0
    for(int i=num; i<CH_COUNT; i++) 
	{
        vofa_data.ch_data[i] = 0.0f; 
    }
	
	memcpy(vofa_data.tail, FRAME_TAIL, sizeof(FRAME_TAIL));
	
	
	HAL_UART_Transmit_DMA(&huart7,(uint8_t*)&vofa_data,sizeof(VOFA_DATA));//发送
}
