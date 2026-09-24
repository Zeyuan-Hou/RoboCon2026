#include "vision_data_deal.h"

void vision_deal(void)
{
	//视觉自瞄YAW角数据处理，顺时针改逆时针，弧度转角度
	Vision_Data.aim_chassis_yaw /= -RADIAN;
	
	//视觉其他YAW轴数据处理，弧度转角度
	Vision_Data.nav_pos_q /= RADIAN;;
	Vision_Data.joy_v_w /= RADIAN;
	Vision_Data.radar_q /= RADIAN;
	
	//视觉定位X、Y数据处理
	Vision_Data.radar_x *= 1000;
	Vision_Data.radar_y *= 1000;
	
	//视觉导航X、Y数据处理
	Vision_Data.nav_v_x *= 1000;
	Vision_Data.nav_v_y *= 1000;
	Vision_Data.nav_pos_x *= 1000;
	Vision_Data.nav_pos_y *= 1000;
	
	//视觉摇杆速度X、Y数据处理
	Vision_Data.joy_v_x *= 1000;
	Vision_Data.joy_v_y *= 1000;
	
}

