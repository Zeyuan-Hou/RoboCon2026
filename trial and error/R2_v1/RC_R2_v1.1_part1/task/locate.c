#include "locate.h"
#include "gyro.h"
void Vision_location(ST_Nav *p_nav)
{
	if (location_filter.flag){
		LocationFilter();
		location_filter.flag = 0;
	}
	p_nav->auto_path.pos_pid.x.fpFB = stRobot.stPos.fpPosX;
	p_nav->auto_path.pos_pid.y.fpFB = stRobot.stPos.fpPosY;
	p_nav->auto_path.pos_pid.z.fpFB = vision_data_recieve.z1;
	p_nav->auto_path.pos_pid.w.fpFB = stRobot.stPos.fpPosQ / 10.f;
	switch (vision_data_recieve.id)
	{
	case 1:
		vision_data_recieve.real_z = MEIHUA_1;
		break;
	case 2:
		vision_data_recieve.real_z = MEIHUA_2;
		break;
	case 3:
		vision_data_recieve.real_z = MEIHUA_3;
		break;
	case 4:
		vision_data_recieve.real_z = MEIHUA_4;
		break;
	case 5:
		vision_data_recieve.real_z = MEIHUA_5;
		break;
	case 6:
		vision_data_recieve.real_z = MEIHUA_6;
		break;
	case 7:
		vision_data_recieve.real_z = MEIHUA_7;
		break;
	case 8:
		vision_data_recieve.real_z = MEIHUA_8;
		break;
	case 9:
		vision_data_recieve.real_z = MEIHUA_9;
		break;
	case 10:
		vision_data_recieve.real_z = MEIHUA_10;
		break;
	case 11:
		vision_data_recieve.real_z = MEIHUA_11;
		break;
	case 12:
		vision_data_recieve.real_z = MEIHUA_12;
		break;
	}
	switch (vision_data_recieve.next_id)
	{
	case 1:
		vision_data_recieve.x2 = MEIHUA_1_x;
		vision_data_recieve.y2 = MEIHUA_1_y;
		vision_data_recieve.z2 = MEIHUA_1;
		break;
	case 2:
		vision_data_recieve.x2 = MEIHUA_2_x;
		vision_data_recieve.y2 = MEIHUA_2_y;
		vision_data_recieve.z2 = MEIHUA_2;
		break;
	case 3:
		vision_data_recieve.x2 = MEIHUA_3_x;
		vision_data_recieve.y2 = MEIHUA_3_y;
		vision_data_recieve.z2 = MEIHUA_3;
		break;
	case 4:
		vision_data_recieve.x2 = MEIHUA_4_x;
		vision_data_recieve.y2 = MEIHUA_4_y;
		vision_data_recieve.z2 = MEIHUA_4;
		break;
	case 5:
		vision_data_recieve.x2 = MEIHUA_5_x;
		vision_data_recieve.y2 = MEIHUA_5_y;
		vision_data_recieve.z2 = MEIHUA_5;
		break;
	case 6:
		vision_data_recieve.x2 = MEIHUA_6_x;
		vision_data_recieve.y2 = MEIHUA_6_y;
		vision_data_recieve.z2 = MEIHUA_6;
		break;
	case 7:
		vision_data_recieve.x2 = MEIHUA_7_x;
		vision_data_recieve.y2 = MEIHUA_7_y;
		vision_data_recieve.z2 = MEIHUA_7;
		break;
	case 8:
		vision_data_recieve.x2 = MEIHUA_8_x;
		vision_data_recieve.y2 = MEIHUA_8_y;
		vision_data_recieve.z2 = MEIHUA_8;
		break;
	case 9:
		vision_data_recieve.x2 = MEIHUA_9_x;
		vision_data_recieve.y2 = MEIHUA_9_y;
		vision_data_recieve.z2 = MEIHUA_9;
		break;
	case 10:
		vision_data_recieve.x2 = MEIHUA_10_x;
		vision_data_recieve.y2 = MEIHUA_10_y;
		vision_data_recieve.z2 = MEIHUA_10;
		break;
	case 11:
		vision_data_recieve.x2 = MEIHUA_11_x;
		vision_data_recieve.y2 = MEIHUA_11_y;
		vision_data_recieve.z2 = MEIHUA_11;
		break;
	case 12:
		vision_data_recieve.x2 = MEIHUA_12_x;
		vision_data_recieve.y2 = MEIHUA_12_y;
		vision_data_recieve.z2 = MEIHUA_12;
		break;
	}
	switch (vision_data_recieve.extra_id1)
	{
	case 1:
		vision_data_recieve.x3 = MEIHUA_1_x;
		vision_data_recieve.y3 = MEIHUA_1_y;
		vision_data_recieve.z3 = MEIHUA_1;
		break;
	case 2:
		vision_data_recieve.x3 = MEIHUA_2_x;
		vision_data_recieve.y3 = MEIHUA_2_y;
		vision_data_recieve.z3 = MEIHUA_2;
		break;
	case 3:
		vision_data_recieve.x3 = MEIHUA_3_x;
		vision_data_recieve.y3 = MEIHUA_3_y;
		vision_data_recieve.z3 = MEIHUA_3;
		break;
	case 4:
		vision_data_recieve.x3 = MEIHUA_4_x;
		vision_data_recieve.y3 = MEIHUA_4_y;
		vision_data_recieve.z3 = MEIHUA_4;
		break;
	case 5:
		vision_data_recieve.x3 = MEIHUA_5_x;
		vision_data_recieve.y3 = MEIHUA_5_y;
		vision_data_recieve.z3 = MEIHUA_5;
		break;
	case 6:
		vision_data_recieve.x3 = MEIHUA_6_x;
		vision_data_recieve.y3 = MEIHUA_6_y;
		vision_data_recieve.z3 = MEIHUA_6;
		break;
	case 7:
		vision_data_recieve.x3 = MEIHUA_7_x;
		vision_data_recieve.y3 = MEIHUA_7_y;
		vision_data_recieve.z3 = MEIHUA_7;
		break;
	case 8:
		vision_data_recieve.x3 = MEIHUA_8_x;
		vision_data_recieve.y3 = MEIHUA_8_y;
		vision_data_recieve.z3 = MEIHUA_8;
		break;
	case 9:
		vision_data_recieve.x3 = MEIHUA_9_x;
		vision_data_recieve.y3 = MEIHUA_9_y;
		vision_data_recieve.z3 = MEIHUA_9;
		break;
	case 10:
		vision_data_recieve.x3 = MEIHUA_10_x;
		vision_data_recieve.y3 = MEIHUA_10_y;
		vision_data_recieve.z3 = MEIHUA_10;
		break;
	case 11:
		vision_data_recieve.x3 = MEIHUA_11_x;
		vision_data_recieve.y3 = MEIHUA_11_y;
		vision_data_recieve.z3 = MEIHUA_11;
		break;
	case 12:
		vision_data_recieve.x3 = MEIHUA_12_x;
		vision_data_recieve.y3 = MEIHUA_12_y;
		vision_data_recieve.z3 = MEIHUA_12;
		break;
	}
	switch (vision_data_recieve.extra_id2)
	{
	case 1:
		vision_data_recieve.x4 = MEIHUA_1_x;
		vision_data_recieve.y4 = MEIHUA_1_y;
		vision_data_recieve.z4 = MEIHUA_1;
		break;
	case 2:
		vision_data_recieve.x4 = MEIHUA_2_x;
		vision_data_recieve.y4 = MEIHUA_2_y;
		vision_data_recieve.z4 = MEIHUA_2;
		break;
	case 3:
		vision_data_recieve.x4 = MEIHUA_3_x;
		vision_data_recieve.y4 = MEIHUA_3_y;
		vision_data_recieve.z4 = MEIHUA_3;
		break;
	case 4:
		vision_data_recieve.x4 = MEIHUA_4_x;
		vision_data_recieve.y4 = MEIHUA_4_y;
		vision_data_recieve.z4 = MEIHUA_4;
		break;
	case 5:
		vision_data_recieve.x4 = MEIHUA_5_x;
		vision_data_recieve.y4 = MEIHUA_5_y;
		vision_data_recieve.z4 = MEIHUA_5;
		break;
	case 6:
		vision_data_recieve.x4 = MEIHUA_6_x;
		vision_data_recieve.y4 = MEIHUA_6_y;
		vision_data_recieve.z4 = MEIHUA_6;
		break;
	case 7:
		vision_data_recieve.x4 = MEIHUA_7_x;
		vision_data_recieve.y4 = MEIHUA_7_y;
		vision_data_recieve.z4 = MEIHUA_7;
		break;
	case 8:
		vision_data_recieve.x4 = MEIHUA_8_x;
		vision_data_recieve.y4 = MEIHUA_8_y;
		vision_data_recieve.z4 = MEIHUA_8;
		break;
	case 9:
		vision_data_recieve.x4 = MEIHUA_9_x;
		vision_data_recieve.y4 = MEIHUA_9_y;
		vision_data_recieve.z4 = MEIHUA_9;
		break;
	case 10:
		vision_data_recieve.x4 = MEIHUA_10_x;
		vision_data_recieve.y4 = MEIHUA_10_y;
		vision_data_recieve.z4 = MEIHUA_10;
		break;
	case 11:
		vision_data_recieve.x4 = MEIHUA_11_x;
		vision_data_recieve.y4 = MEIHUA_11_y;
		vision_data_recieve.z4 = MEIHUA_11;
		break;
	case 12:
		vision_data_recieve.x4 = MEIHUA_12_x;
		vision_data_recieve.y4 = MEIHUA_12_y;
		vision_data_recieve.z4 = MEIHUA_12;
		break;
	}
}

float Lx1=276.93f,Ly1=309.93f,//DT35距车中心距离
	Lx2=1690,Ly2=500,//DT35坐标系到视觉坐标系偏移
	fpQ0=0,fpQ_rad;//DT35坐标系到视觉坐标系转角
void LocationFilter(){
//	location_filter.x[0] = vision_data_recieve.x1;
//	location_filter.y[0] = vision_data_recieve.y1;
//  location_filter.yaw[0] = vision_data_recieve.yaw1;

//	for (int i=1; i<5; i++){
//		location_filter.x[i] = location_filter.x[i-1] + location_filter.lpf_k * (vision_data_recieve.x1 - location_filter.x[i-1]);
//		location_filter.y[i] = location_filter.y[i-1] + location_filter.lpf_k * (vision_data_recieve.y1 - location_filter.y[i-1]);
//		location_filter.yaw[i] = location_filter.yaw[i-1] + location_filter.lpf_k * (vision_data_recieve.yaw1 - location_filter.yaw[i-1]);
//	}

	float sum_x = 0.f, sum_y = 0.f, sum_yaw = 0.f;
//	for (int i=0; i<5; i++){
//		sum_x += location_filter.x[i];
//		sum_y += location_filter.y[i];
//		sum_yaw += location_filter.yaw[i];
//	}
	
	sum_x = vision_data_recieve.x1 *5;
	sum_y = vision_data_recieve.y1 *5;
	sum_yaw = vision_data_recieve.yaw1 *5;
	
	

	//DT35位置
//	float fpQ = gyro_data.yaw*10;//接收雷达数据
//	if(fabs(fpQ-gyro.fpQ_Pre)>20){//错误判断
//		gyro.fpQ_Cur=gyro.fpQ_Pre;
//	}else{
//		gyro.fpQ_Cur=fpQ;//无错误时更新当前角度
//	}
//	gyro.fpQ_Pre=gyro.fpQ_Cur;//记录当前角度
	float fpQ = sum_yaw/5;//接收雷达数据
	//错误数据处理
	if(fabs(dt35_now.dt35_voltage_1-dt35_now.dt35_pre_voltage_1) > 100){
    dt35_now.x_suspect=1;
  }else{
    dt35_now.x_suspect=0;
  }
  dt35_now.dt35_pre_voltage_1 = dt35_now.dt35_voltage_1;
	if(fabs(dt35_now.dt35_voltage_2-dt35_now.dt35_pre_voltage_2) > 100){
		dt35_now.y_suspect=1;
  }else{
		dt35_now.y_suspect=0;
  }
  dt35_now.dt35_pre_voltage_2 = dt35_now.dt35_voltage_2;
	dt35_now.x_suspect=1;
	dt35_now.y_suspect=1;
	if(sum_x/5>600&&sum_x/5<1413){
		if(sum_y/5<-150&&sum_y/5>-600){
			if(sum_yaw/5<0.8f&&sum_yaw/5>-0.8f){
				dt35_now.x_suspect=0;
				dt35_now.y_suspect=0;
			}
		}
	}
	fpQ_rad=fpQ-18/1800*PI;
	//坐标解算
	dt35_now.dt35_robot_x=-(Lx1+dt35_now.dt35_voltage_1)*cos(fpQ_rad)+Lx2;
	dt35_now.dt35_robot_y=-(Ly1+dt35_now.dt35_voltage_2)*cos(fpQ_rad)+Ly2;
	
	//坐标赋值
//	stRobot.stPos.fpPosX =sum_x / 5.f;
//	stRobot.stPos.fpPosY =sum_y / 5.f;
//	stRobot.stPos.fpPosQ =sum_yaw / 5.f * 1800 / PI; // 转为0.1度
	if(sum_x>600){
		stRobot.stPos.fpPosX =dt35_now.x_suspect*sum_x / 5.f+(1-dt35_now.x_suspect)*dt35_now.dt35_robot_x;
	}else{
		stRobot.stPos.fpPosX =dt35_now.x_suspect*sum_x / 5.f - 100;
	}
	stRobot.stPos.fpPosY =dt35_now.y_suspect*sum_y / 5.f+(1-dt35_now.y_suspect)*dt35_now.dt35_robot_y;
	stRobot.stPos.fpPosQ =dt35_now.x_suspect*sum_yaw / 5.f * 1800 / PI+(1-dt35_now.x_suspect)*fpQ_rad* 1800 / PI; // 转为0.1度
}
