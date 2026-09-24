//VOFA这块的底层代码应该是比较完善的了，不需要再过多修改，关于应用相关的应该就是像写在freertos中的导航任务中的那一段
#include "vofa.h"
#include "string.h"
VOFA_DATA vofa_data;

float vofa[19]; 

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
	
	
	HAL_UART_Transmit_DMA(&huart6,(uint8_t*)&vofa_data,sizeof(VOFA_DATA));//发送
	system_monitor.VOFA_cnt++;
}
