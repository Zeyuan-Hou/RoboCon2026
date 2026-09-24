#include "Vision.h"

 void Vision_Data_Deal(ST_VISION_DATA* p_vision_data)
 {
 	
 	memcpy(&p_vision_data->radar_y, &Radar_RxBuf[1], 4);
 	memcpy(&p_vision_data->radar_x, &Radar_RxBuf[5], 4);
	 p_vision_data->radar_x= -(p_vision_data->radar_x);
	 
 	memcpy(&p_vision_data->radar_z, &Radar_RxBuf[9], 4);	 
	 
 	memcpy(&p_vision_data->radar_yaw, &Radar_RxBuf[13], 4);
	 
	 
 	memcpy(&p_vision_data->x2, &Radar_RxBuf[17], 4);
 	memcpy(&p_vision_data->y2, &Radar_RxBuf[21], 4);
 	memcpy(&p_vision_data->yaw2, &Radar_RxBuf[25], 4);
	memcpy(&p_vision_data->yaw3, &Radar_RxBuf[29], 4);
	memcpy(&p_vision_data->distance, &Radar_RxBuf[33], 4);
	 
	memcpy(&uart1_tx_buffer[5], &Radar_RxBuf[17], 4);
	memcpy(&uart1_tx_buffer[9], &Radar_RxBuf[21], 4);
 	memcpy(&uart1_tx_buffer[13], &Radar_RxBuf[25], 4);
	memcpy(&uart1_tx_buffer[17], &Radar_RxBuf[29], 4);
	memcpy(&uart1_tx_buffer[21], &Radar_RxBuf[33], 4);


 	Vision_location(&nav);
 }
 
void Vision_location(ST_Nav *p_nav)
 {
 	if (location_filter.flag){
		LocationFilter();
		location_filter.flag = 0;
	}
	
// 	p_nav->auto_path.pos_pid.x.fpFB = stRobot.stPos.fpPosX;
// 	p_nav->auto_path.pos_pid.y.fpFB = stRobot.stPos.fpPosY;
// 	p_nav->auto_path.pos_pid.w.fpFB = stRobot.stPos.fpPosQ / 10.f;
  
 }

 void LocationFilter()
 {
	 
	  location_filter.x[0] = Vision_Data.radar_x;
		location_filter.y[0] = Vision_Data.radar_y;
	  location_filter.z[0] = Vision_Data.radar_z;
		location_filter.yaw[0] = Vision_Data.radar_yaw;
	  
		for (int i=4; i>0; i--){
			location_filter.x[i] = location_filter.x[i-1] + location_filter.lpf_k * (Vision_Data.radar_x - location_filter.x[i-1]);
			location_filter.y[i] = location_filter.y[i-1] + location_filter.lpf_k * (Vision_Data.radar_y - location_filter.y[i-1]);
			location_filter.z[i] = location_filter.z[i-1] + location_filter.lpf_k * (Vision_Data.radar_z - location_filter.z[i-1]);
			location_filter.yaw[i] = location_filter.yaw[i-1] + location_filter.lpf_k * (Vision_Data.radar_yaw - location_filter.yaw[i-1]);
		}
		 
	  float sum_x = 0.f, sum_y = 0.f, sum_z = 0.f,sum_yaw = 0.f;
		for (int i=0; i<5; i++){
			sum_x += location_filter.x[i];
			sum_y += location_filter.y[i];
			sum_z += location_filter.z[i];
			sum_yaw += location_filter.yaw[i];
		}
	 
			//坐标赋值
		test_stRobot.stPos.fpPosX =31.f+sum_x/ 5.f;
		test_stRobot.stPos.fpPosY =4282.f+sum_y / 5.f+3.f;
		test_stRobot.stPos.fpPosZ =sum_z/5.f-395.f; 
		test_stRobot.stPos.fpPosQ =sum_yaw / 5.f * 1800 / PI; // 转为0.1度，会连续跨圈


//不再在这里赋值了		
//	 	stRobot.stPos.fpPosX =48.f+sum_x / 5.f;
//		stRobot.stPos.fpPosY =4282.f+sum_y / 5.f;		
//		  stRobot.stPos.fpPosQ =sum_yaw / 5.f * 1800 / PI; // 转为0.1度
	    stRobot.stPos.fpPosZ =sum_z/5.f-395.f;
 	
 }
 void Vision_Transmit(void)
 {
 	vision_tx[0] = 0x11;
// 	vision_tx[1] = ; // uint_8
 	vision_tx[2] = 0x22;
 	HAL_UART_Transmit_DMA(&huart5, vision_tx, 3); // 发送
 }

 void Vision_Location(void)
{ 
	 	stRobot.stPos.fpPosX =test_stRobot.stPos.fpPosX+fpPosXOffset;
		stRobot.stPos.fpPosY =test_stRobot.stPos.fpPosY+fpPosYOffset;	
		stRobot.stPos.fpPosQ =test_stRobot.stPos.fpPosQ; 
}
