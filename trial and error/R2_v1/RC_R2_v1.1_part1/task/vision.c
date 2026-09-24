#include "vision.h"
/****定位+规划*****/
// 帧头 坐标 目标位置 目标速度 路径完成标志位 帧尾
// 1 + 3*4 + 2*4 + 2*4 + 1 + 1 = 31
void Vision_Data_Deal(VISION_DATA_AUTO *p_vision_data)
{
	
	memcpy(&p_vision_data->id, &vision_rec[1], 1);
	memcpy(&p_vision_data->x1, &vision_rec[2], 4);
	memcpy(&p_vision_data->y1, &vision_rec[6], 4);
	memcpy(&p_vision_data->z1, &vision_rec[10], 4);
	memcpy(&p_vision_data->yaw1, &vision_rec[14], 4);
	
	if(flag_use_vision)
	{
			memcpy(&p_vision_data->judge, &vision_rec[18], 1);
			memcpy(&p_vision_data->x2, &vision_rec[19], 4);
			memcpy(&p_vision_data->y2, &vision_rec[23], 4);

			memcpy(&p_vision_data->next_id, &vision_rec[27], 1);
			memcpy(&p_vision_data->flag_123, &vision_rec[28], 1);
			memcpy(&p_vision_data->count_num, &vision_rec[29], 1);

			memcpy(&p_vision_data->extra_id1, &vision_rec[30], 1);
			memcpy(&p_vision_data->x3, &vision_rec[31], 4);
			memcpy(&p_vision_data->y3, &vision_rec[35], 4);

			memcpy(&p_vision_data->extra_id2, &vision_rec[39], 1);
			memcpy(&p_vision_data->x4, &vision_rec[40], 4);
			memcpy(&p_vision_data->y4, &vision_rec[44], 4);
			
			memcpy(&p_vision_data->I_max,&vision_rec[60], 1);
	}
	memcpy(&vision_judge,&vision_rec[48], 12);
	Vision_location(&nav);
	//	HAL_UART_Transmit_IT(&huart2,vision_tx,50);//发送
}
void Vision_Transmit(void)
{
	vision_tx[0] = 0x11;
	vision_tx[1] = flag_vision_update; // uint_8
	vision_tx[2] = 0x22;
	HAL_UART_Transmit_DMA(&huart5, vision_tx, 3); // 发送
}
