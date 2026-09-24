#include "JSValue.h"


//void path_1_remote(void)
//{		
//	static uint16_t delay_change_area= 0; //延迟切换到二区的计时器
//	static uint8_t  nav_inited = 0;
//	static uint8_t  prev_state = 0;
//	static uint8_t  num_header = 0;  //当前是第几个矛头
//		
//	static float temp_x[7], temp_y[7];  // 索引1~6使用
//		
//		if (path_state_1 != prev_state) {
//      prev_state = path_state_1;
//      nav_inited = 0;} /* 状态变化，重置标志 */ 	
//   
//	
//	switch (path_state_1)
//	{
//		case 0: //初始状态
//			delay_change_area = 0;
//			nav_inited = 0;
//			num_header = 0;
//			break;
//		
//		case 1://从当前位置到一个夹头点位，若上层机构反馈夹头失败，需要重新取头
//			num_header = 1;
//			area_one_target.target_rad = HEAD_RAD;
//			area_one_target.target_x = header_pos.header_1_x;
//			area_one_target.target_y = header_pos.header_1_y;
//		
//		if(!nav_inited){nav.nav_state = NAV_DT35;nav_inited = 1;}
//		
//			if (nav.nav_state == NAV_LOCK) {  /* 通知上层：夹头 */
//					inner_send[1] = 1;
//			if (inner_receive[1] == 1) {
//            /* 取头成功 → 去对接 */
//            inner_send[1] = 0;
//            path_state_1 = 7;
//        } else if (inner_receive[1] == 2) {
//            /* 取头失败 → 试下一个头 */
//            inner_send[1] = 0;
//            path_state_1 = 2;
//        } 
//			} 
//			break;

//		case 2://从当前位置一个夹头点位，上层机构反馈夹头失败,需要重新取头
//			num_header = 2;
//			area_one_target.target_rad = HEAD_RAD;
//			area_one_target.target_x = header_pos.header_2_x;
//			area_one_target.target_y = header_pos.header_2_y;
//		
//			if(!nav_inited){nav.nav_state = NAV_DT35;nav_inited = 1;}
//		
//			if (nav.nav_state == NAV_LOCK) { if (inner_send[1] != 1) inner_send[1] = 1; 
//			
//			if (inner_receive[1] == 1) { inner_send[1] = 0; path_state_1 = 7; }
//      else if (inner_receive[1] == 2) { inner_send[1] = 0; path_state_1 = 3; }
//			}
//			break;

//		case 3://从当前位置到一个夹头点位，上层机构反馈夹头失败,需要重新取头
//			num_header = 3;
//			area_one_target.target_rad = HEAD_RAD;
//			area_one_target.target_x = header_pos.header_3_x;
//			area_one_target.target_y = header_pos.header_3_y;			
//		
//			if(!nav_inited){nav.nav_state = NAV_DT35;nav_inited = 1;}
//		
//			if (nav.nav_state == NAV_LOCK) { if (inner_send[1] != 1) inner_send[1] = 1; 
//			
//			if (inner_receive[1] == 1) { inner_send[1] = 0; path_state_1 = 7; }
//      else if (inner_receive[1] == 2) { inner_send[1] = 0; path_state_1 = 4; }
//			}
//			break;

//		case 4://从当前位置到一个夹头点位，上层机构反馈夹头失败，需要重新取头
//			num_header = 4;
//			area_one_target.target_rad = HEAD_RAD;
//			area_one_target.target_x = header_pos.header_4_x;
//			area_one_target.target_y = header_pos.header_4_y;			
//		
//			if(!nav_inited){nav.nav_state = NAV_DT35;nav_inited = 1;}
//		
//			if (nav.nav_state == NAV_LOCK) { if (inner_send[1] != 1) inner_send[1] = 1; 
//			
//			if (inner_receive[1] == 1) { inner_send[1] = 0; path_state_1 = 7; }
//      else if (inner_receive[1] == 2) { inner_send[1] = 0; path_state_1 = 5; }
//			}
//			break;

//		case 5://从当前位置到一个夹头点位，上层机构反馈夹头失败，需要重新取头
//			num_header = 5;
//			area_one_target.target_rad = HEAD_RAD;
//			area_one_target.target_x = header_pos.header_5_x;
//			area_one_target.target_y = header_pos.header_5_y;			
//		
//			if(!nav_inited){nav.nav_state = NAV_DT35;nav_inited = 1;}
//		
//			if (nav.nav_state == NAV_LOCK) { if (inner_send[1] != 1) inner_send[1] = 1; 
//			
//			if (inner_receive[1] == 1) { inner_send[1] = 0; path_state_1 = 7; }
//      else if (inner_receive[1] == 2) { inner_send[1] = 0; path_state_1 = 6; } 
//			}
//			break;

//		case 6://从当前位置到一个夹头点位，上层机构反馈夹头失败，需要重新取头
//			num_header = 6;
//			area_one_target.target_rad = HEAD_RAD;
//			area_one_target.target_x = header_pos.header_6_x;
//			area_one_target.target_y = header_pos.header_6_y;		
//		
//			if(!nav_inited){nav.nav_state = NAV_DT35;nav_inited = 1;}
//		
//			if (nav.nav_state == NAV_LOCK) { if (inner_send[1] != 1) inner_send[1] = 1; 
//			
//			if (inner_receive[1] == 1) { inner_send[1] = 0; path_state_1 = 7; }
//      else if (inner_receive[1] == 2) { inner_send[1] = 0; path_state_1 = 7; } 
//			}
//			break;
//		case 7://转动180度（取完矛头转过来与R1对接）并走到对接位置 需要上层机构反馈夹取成功标志位
//			if(!nav_inited){nav.auto_path.number_point = 7;nav.nav_state = NAV_POINT_TO_POINT; flag_area_to_point = 2;nav_inited = 1;nav_reach_state = 3;}
//			if (nav.nav_state == NAV_ONE_AREA_LOCK) { nav_reach_state = 0; flag_area_to_point = 0;
//				
//			if (vision_data_recieve.aruco_detect_flag==1) {
//				
//				nav.auto_path.pos_pid.pid_x.fpE = 0;
//				nav.auto_path.pos_pid.pid_y.fpE = 0;
//				nav.auto_path.pos_pid.pid_w.fpE = 0;
//			
//			inner_send[1] = 3;path_state_1 = 8;} 
//				}
//			break;

//		case 8://R1二维码反馈对接成功，延迟200秒（默认1秒打开夹爪）
//			delay_change_area++;
//			if(delay_change_area>=200) {delay_change_area = 0;path_state_1 = 9;}
//			break;

//		case 9://给R1让路
//			if(!nav_inited){nav.auto_path.number_point = 8;nav.nav_state = NAV_POINT_TO_POINT; nav_inited = 1;nav_reach_state = 2;}
//			if (nav.nav_state == NAV_LOCK){delay_change_area++;if(delay_change_area>=2500){nav_reach_state = 0;path_state_1 = 10;}}
//			break;
//		
//		case 10://直接切到二区导航状态机
//		
//			break;
//		
//		case 11://一区先靠雷达导航到一个范围，用DT35确定六个点
//			if(!nav_inited){flag_area_to_point = 1; nav.auto_path.number_point = 32;nav.nav_state = NAV_POINT_TO_POINT; nav_inited = 1;nav_reach_state = 5;}
//			if (nav.nav_state == NAV_LOCK){flag_area_to_point = 0;nav_reach_state = 0;path_state_1 = num_header+1;nav_reach_state = 0;}
//			break;
//			
//		case 12://武器头甩丢了，跑回重试区
//	
//			break;
//			
//		case 13://先根据视觉判断取头顺序
//			//判断有无漏输
//		
//			if(vision_data_recieve.header_num_1+vision_data_recieve.header_num_2+vision_data_recieve.header_num_3+vision_data_recieve.header_num_4+vision_data_recieve.header_num_5+vision_data_recieve.header_num_6==21)
//			{
//				temp_x[1] = HEAD_1_X;
//				temp_y[1] = HEAD_1_Y;

//				temp_x[2] = HEAD_2_X;
//				temp_y[2] = HEAD_2_Y;

//				temp_x[3] = HEAD_3_X;
//				temp_y[3] = HEAD_3_Y;

//				temp_x[4] = HEAD_4_X;
//				temp_y[4] = HEAD_4_Y;

//				temp_x[5] = HEAD_5_X;
//				temp_y[5] = HEAD_5_Y;

//				temp_x[6] = HEAD_6_X;
//				temp_y[6] = HEAD_6_Y;
//				
//				header_pos.header_1_x = temp_x[vision_data_recieve.header_num_1];
//        header_pos.header_1_y = temp_y[vision_data_recieve.header_num_1];
//				
//        header_pos.header_2_x = temp_x[vision_data_recieve.header_num_2];
//        header_pos.header_2_y = temp_y[vision_data_recieve.header_num_2];
//        
//        header_pos.header_3_x = temp_x[vision_data_recieve.header_num_3];
//        header_pos.header_3_y = temp_y[vision_data_recieve.header_num_3];
//        
//        header_pos.header_4_x = temp_x[vision_data_recieve.header_num_4];
//        header_pos.header_4_y = temp_y[vision_data_recieve.header_num_4];
//        
//        header_pos.header_5_x = temp_x[vision_data_recieve.header_num_5];
//        header_pos.header_5_y = temp_y[vision_data_recieve.header_num_5];
//        
//        header_pos.header_6_x = temp_x[vision_data_recieve.header_num_6];
//        header_pos.header_6_y = temp_y[vision_data_recieve.header_num_6];
//			
//			}else {
//			
//				header_pos.header_1_x = HEAD_1_X;
//        header_pos.header_1_y = HEAD_1_Y;
//				
//        header_pos.header_2_x = HEAD_2_X;
//        header_pos.header_2_y = HEAD_2_Y;
//        
//        header_pos.header_3_x = HEAD_3_X;
//        header_pos.header_3_y = HEAD_3_Y;
//        
//        header_pos.header_4_x = HEAD_4_X;
//        header_pos.header_4_y = HEAD_4_Y;
//        
//        header_pos.header_5_x = HEAD_5_X;
//        header_pos.header_5_y = HEAD_5_Y;
//				
//        header_pos.header_6_x = HEAD_6_X;
//        header_pos.header_6_y = HEAD_6_Y;			
//			}
//		
//			path_state_1 = 11;
//			
//			break;

//		default:
//			break;
//	}
//		}


void adjust_area(void)
{
	static uint8_t num_state = 1;	
	static uint8_t prev_state = 0;

    if (prev_state != remote_state) {
        prev_state = remote_state;
		}
		
	switch(remote_state)
	{
		case 0://初始化
			break;
		
		case 1:  //等待遥控信息
		if(Js_Value.usJsKey==10){remote_state = num_state + 1;}
		if(Js_Value.usJsKey==11){remote_state = num_state - 1;}		
			break;		
	
		case 2://启动区到夹头
			
		num_state = 2;
			break;			
	
		case 3:
			
		num_state = 3;
			break;		

		case 4:
			
		num_state = 4;
			break;		

		case 5:
			
		num_state = 5;
			break;		

		case 6:
			
		num_state = 6;		
			break;				
	}
}



void check_fps(void)
{
	if(monitor.rate_fps.lift_201<=800){monitor_error.lift_201=0;}else {monitor_error.lift_201=1;}
	if(monitor.rate_fps.lift_202<=800){monitor_error.lift_202=0;}else {monitor_error.lift_202=1;}
	if(monitor.rate_fps.lift_203<=800){monitor_error.lift_203=0;}else {monitor_error.lift_203=1;}
	if(monitor.rate_fps.lift_204<=800){monitor_error.lift_204=0;}else {monitor_error.lift_204=1;}
	
	if(monitor.rate_fps.wheel_201<=800){monitor_error.wheel_201=0;}else {monitor_error.wheel_201=1;}
	if(monitor.rate_fps.wheel_202<=800){monitor_error.wheel_202=0;}else {monitor_error.wheel_202=1;}
	if(monitor.rate_fps.wheel_203<=800){monitor_error.wheel_203=0;}else {monitor_error.wheel_203=1;}
	if(monitor.rate_fps.wheel_204<=800){monitor_error.wheel_204=0;}else {monitor_error.wheel_204=1;}
	
	if(monitor.rate_fps.DJI_201<=800){monitor_error.DJI_201=0;}else {monitor_error.DJI_201=1;}
	if(monitor.rate_fps.DJI_202<=800){monitor_error.DJI_202=0;}else {monitor_error.DJI_202=1;}	
	
	if(monitor.rate_fps.vision<=220){monitor_error.vision=0;}else {monitor_error.vision=1;}		
	
}




void remote_send_(void)
{
	remote_send	[0] = 0x0A;
	remote_send [24] = 0x0B;

  memcpy(&remote_send[1],  &robot_pos.fpPosX, 4);  // 占字节1-4
  memcpy(&remote_send[5],  &robot_pos.fpPosY, 4);  // 占字节5-8
  memcpy(&remote_send[9],  &robot_pos.fpPosQ, 4);  // 占字节9-12	
	
	//手操导航状态
	remote_send[13] = 1;
	
	check_fps();
	uint8_t error_lift_DJI = monitor_error.lift_201 + monitor_error.lift_202 * 2 + monitor_error.lift_203 * 4 + monitor_error.lift_204 * 8 + monitor_error.DJI_201 * 16 + monitor_error.DJI_202 * 32; 
	uint8_t error_wheel_vision = monitor_error.wheel_201 + monitor_error.wheel_202 * 2 + monitor_error.wheel_203 * 4 + monitor_error.wheel_204 * 8 + monitor_error.vision * 16;
	
	remote_send[14] = error_lift_DJI;		
	remote_send[15] = error_wheel_vision;
	
	remote_send[16] = inner_receive[7];
	remote_send[17] = inner_receive[8];
	remote_send[18] = 1;
	remote_send[19] = 1;
	remote_send[20] = 1;
	remote_send[21] = 1;
	remote_send[22] = 1;
	remote_send[23] = 1;
	
	
	HAL_UART_Transmit_DMA(&huart2,remote_send,sizeof(remote_send));

}






void remote_deal(uint8_t Rx_Buf[9],uint8_t remote_rec_uart[11])
{
		
	size_t remote_len = 11;
	for(uint8_t a = 0;a<remote_len;a++){
		if(remote_rec_uart[a] == 0xAA && remote_rec_uart[(a+remote_len-1)%remote_len] == 0xBB)
		{	
			
       uint8_t seg1_len = 10 - a;   // 从 a+1 到索引10 的字节数
       if (seg1_len > 0)
           memcpy(&Rx_Buf[0], &remote_rec_uart[a + 1], seg1_len);
			
       uint8_t seg2_len = a;        // 从索引0 到索引 a-1 的字节数
       if (seg2_len > 0)
           memcpy(&Rx_Buf[seg1_len], &remote_rec_uart[0], seg2_len);			 
				
			break;
		}
	}
	parseDataPacket(Rx_Buf,&Js_Value);

}


//定义结构体
ST_JS_VALUE Js_Value;
uint8_t remote_rec_uart[11] = {0};
uint8_t nRF24L01_RxBuf[9]={0};//接收数据缓存
// 定义解算函数
void parseDataPacket(const uint8_t Rx_Buf[9], ST_JS_VALUE *jsValue) {
 
    // 摇杆赋值
    jsValue->usJsLeft_X = (Rx_Buf[0] << 8) | Rx_Buf[1];  // 左摇杆X值    
    jsValue->usJsLeft_Y = (Rx_Buf[2] << 8) | Rx_Buf[3];  // 左摇杆Y值    
    jsValue->usJsRight_X = (Rx_Buf[4] << 8) | Rx_Buf[5]; // 右摇杆X值    
    jsValue->usJsRight_Y = (Rx_Buf[6] << 8) | Rx_Buf[7]; // 右摇杆Y值    

    // 按键赋值
    jsValue->usJsKey = Rx_Buf[8];
	 if(ctrl_flag.remote_flag == 1&&up_down_state==0&&nav.nav_state == NAV_REMOTE)
	 {CalculateVelocities(jsValue,&nav.expect_robot_global_velt,ucGateX,ssXSpedLimit,ucGateY,ssYSpedLimit,ucGateW,ssWSpedLimit);}
	
		switch (jsValue->usJsKey)	
		{
		
			case 1:
			ctrl_flag.remote_flag = 1; 
			ctrl_flag.chassis_flag = 1;
			ctrl_flag.dji_flag = 1;
				break;
			
			case 2:
//			nav.nav_state = NAV_REMOTE;
				break;
			
			case 3://登200
//			remote_up_down = 1;
				break;
			
			case 4://登400
//			remote_up_down = 2;
				break;
			
			case 5://下200
//			remote_up_down = 3;
				break;
			
			case 6://下400
//			remote_up_down = 4;
				break;
			
			case 7://起立等待合体
//			remote_up_down = 5;
				break;
			
			case 8://机械臂正
//			inner_send[3] = 2;
				break;			
			
			case 9://机械臂左
//			inner_send[3] = 1;
				break;

			case 10://机械臂右
//			inner_send[3] = 3;
				break;
			
			case 11://机械臂低200
//			inner_send[2] = 1;
				break;
			
			case 12://机械臂高200
//			inner_send[2] = 2;
				break;
			
			case 13://机械臂高400
//			inner_send[2] = 3;				
				break;
			
			case 14://机械臂存
				
				break;
			
			case 15://机械臂持
				
				break;
			
			case 16:		
			memset(&ctrl_flag,0,sizeof(ctrl_flag));
				break;
			
			default :
				break;
		
		}
			
		if(ctrl_flag.remote_flag == 1){CalculateVelocities(jsValue,&nav.expect_robot_global_velt,1000,5000,1000,5000,300,200);}
}





/*******************************************************************************************
函数名称：CalculateVelocities
函数功能：基于手柄摇杆值计算机器人的线速度（X和Y方向）及角速度，并应用低通滤波器进行平滑处理。
输入：   1. jsValue 指向包含手柄摇杆读数的ST_JS_VALUE结构体指针。
          2. velocities 指向用于存储计算得到的速度值的ST_VELT结构体指针。
          3. ucGateX X方向上的阈值，只有超过此阈值才产生运动。
          4. ssXSpedLimit X方向上的最大速度限制（单位mm/s）。
          5. ucGateY Y方向上的阈值，只有超过此阈值才产生运动。
          6. ssYSpedLimit Y方向上的最大速度限制（单位mm/s）。
          7. ucGateW 角速度方向上的阈值，只有超过此阈值才产生旋转。
          8. ssWSpedLimit 角速度方向上的最大速度限制（单位0.1度/s）。
输出：   1. velocities 结构体中的成员变量被更新为计算得到的X方向速度（fpVx）、Y方向速度（fpVy）和角速度（fpW）。
备注：     1. 计算过程首先检查是否超过了设定的阈值，如果超过，则按照比例缩放至最大速度限制。
          2. 使用一阶低通滤波器对速度值进行平滑处理，以减少由于手柄输入抖动或用户操作不精确带来的快速加减速。
          3. 注意坐标系转换：左摇杆X轴对应的是fpVx，左摇杆Y轴对应的是fpVy,但要加一个负号，因为摇杆值Y方向从上到下是从0到4096，X方向从右到左是0到4096。
          4. 右摇杆X控制角速度，其值经过特定的比例缩放后赋给fpW。
*******************************************************************************************/


void CalculateVelocities(const ST_JS_VALUE *jsValue, ST_VECTOR *V_fact,
                         uint16_t ucGateX, int16_t ssXSpedLimit,
                         uint16_t ucGateY, int16_t ssYSpedLimit,
                         uint16_t ucGateW, int16_t ssWSpedLimit)
{
    const float smooth = 20; // 平滑等级，一阶低通滤波器截止频率
	
    static ST_LPF FJx = {0, 0, 0, smooth, 0.002f};     
    static ST_LPF FJy = {0, 0, 0, smooth, 0.002f}; 
    static ST_LPF FJw= {0, 0, 0, smooth, 0.002f}; 
   // 计算左摇杆X方向的速度 (fpVy)
    float Vx = jsValue->usJsLeft_X - LEFT_JS_X_MID;
    if (fabs(Vx) < ucGateX) {
        FJx.in = 0;
    } 
		else {
			if(Vx > 0)
			{
				FJx.in = (Vx- ucGateX) * ssXSpedLimit /(LEFT_JS_X_MAX-LEFT_JS_X_MID );
			}
			else if (Vx < 0)
			{
				FJx.in = (Vx + ucGateX) * ssXSpedLimit /(LEFT_JS_X_MID-LEFT_JS_X_MIN );
			}
    }

    // 计算左摇杆Y方向的速度 (fpVx)

    float Vy = jsValue->usJsLeft_Y - LEFT_JS_Y_MID;

			if (fabs(Vy) < ucGateY) 
			{
					FJy.in = 0;
			} 
			else {
					if (Vy > 0) {
							// Y轴正向偏移
							FJy.in = (Vy - ucGateY) * ssYSpedLimit / (LEFT_JS_Y_MAX - LEFT_JS_Y_MID);
					} 
					else if (Vy < 0) {
							// Y轴负向偏移
							FJy.in = (Vy + ucGateY) * ssYSpedLimit / (LEFT_JS_Y_MID - LEFT_JS_Y_MIN);
					}
			}


// 计算右摇杆X方向的角速度 (fpW)
		float Vw = jsValue->usJsRight_X - RIGHT_JS_MID;
		if (fabs(Vw) < ucGateW) {
				FJw.in = 0; // 在死区内，设置为0
		} 
		else {
				if (Vw > 0) 
				{
						// X轴正向偏移（假设向右为正）
						FJw.in = (Vw - ucGateW) * ssWSpedLimit / (RIGHT_JS_MAX - RIGHT_JS_MID);
				} 
				else if (Vw < 0) 
				{
						// X轴负向偏移（假设向左为负）
						FJw.in = (Vw + ucGateW) * ssWSpedLimit / (RIGHT_JS_MID - RIGHT_JS_MIN);
				}
		}
    // 应用低通滤波
    LpFilter(&FJx);
    LpFilter(&FJy);
    LpFilter(&FJw);

 V_fact->fpX  = FJx.out * test_remote_x; // 摇杆值X方向从右到左是从0到4096。
 V_fact->fpY  = -FJy.out * test_remote_y; // 摇杆值Y方向从上到下是从0到4096。
 V_fact->fpW  = -FJw.out*  RADIAN; //  右摇杆X控制角速度
}
