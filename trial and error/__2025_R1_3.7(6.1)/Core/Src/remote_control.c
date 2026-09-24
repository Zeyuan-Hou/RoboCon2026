#include "remote_control.h"
#include <math.h>    

/*******************************************************************************************
函数名称：parseDataPacket
函数功能：解析来自遥控器的数据包，并将解析后的数据填充到遥控器结构体ST_JS_VALUE中。
          数据包包含摇杆的位置和按键（36合一的矩阵键盘按键和8个独立按键）信息。
输入：   1. Tx_Buf （SPI通讯的存储数组）指向包含32个元素的uint8_t数组，每个元素代表一个字节的数据。
输出：   1. jsValue 指向ST_JS_VALUE结构体的指针，用于存储解析后的摇杆值和按键控制信息。
备注：   1. 数据包的前8个元素用于存储ADC采样值，其中0, 1: 左摇杆X方向；2, 3: 左摇杆Y方向；4, 5: 右摇杆X方向；6, 7: 右摇杆Y方向。
         2. 数据包的第8个元素表示矩阵键盘的值（范围0到36），第9-16个元素表示独立按键的值。
         3. ADC采样值是12位的，需要从两个连续的字节中提取高8位和低8位来重组完整的采样值。
         4. 解析过程中会考虑数据包中的所有相关信息，并将其适当地映射到目标结构体中。
*******************************************************************************************/


void parseDataPacket(const uint8_t Rx_Buf[32], ST_JS_VALUE *jsValue) {
    
    // ADC采样值处理（组装） 
    jsValue->usJsLeft_X= (Rx_Buf[0] << 8) | Rx_Buf[1]; // 组装左摇杆X方向值    左X
    jsValue->usJsLeft_Y= (Rx_Buf[2] << 8) | Rx_Buf[3]; // 组装左摇杆Y方向值    左Y
    jsValue->usJsRight_X = (Rx_Buf[4] << 8) | Rx_Buf[5]; // 组装右摇杆X方向值    右X
    jsValue->usJsRight_Y = (Rx_Buf[6] << 8) | Rx_Buf[7]; // 组装右摇杆Y方向值    右Y

    
    // 矩阵键盘值赋值
    jsValue->usJsKey = Rx_Buf[8];

    // 独立按键值赋值
    for (int i = 0; i < 8; ++i) {
        jsValue->indepen_usJsKey[i] = Rx_Buf[9 + i];
    }
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


void CalculateVelocities(const ST_JS_VALUE *jsValue, ST_Nav *p_nav,
                         uint16_t ucGateX, int16_t ssXSpedLimit,
                         uint16_t ucGateY, int16_t ssYSpedLimit,
                         uint16_t ucGateW, int16_t ssWSpedLimit)
{
    const float smooth = 20; // 平滑等级，一阶低通滤波器截止频率
    static ST_LPF FJx = {0, 0, 0, smooth, 0.002f}; // 需要pre_out，故需要设为静态变量
    static ST_LPF FJy = {0, 0, 0, smooth, 0.002f};
    static ST_LPF FJw = {0, 0, 0, smooth, 0.002f};

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


    p_nav->auto_path.basic_velt.fpVx = FJx.out; // 摇杆值X方向从右到左是从0到4096。
    p_nav->auto_path.basic_velt.fpVy = -FJy.out; // 摇杆值Y方向从上到下是从0到4096。
    p_nav->auto_path.basic_velt.fpW = -FJw.out ; //  右摇杆X控制角速度
}

/*-------------------------------------------------------------------------------------------------
函数功能：双遥控器的断连切换，优先选择使用硬件遥控器
-------------------------------------------------------------------------------------------------*/
void RC_switch(ST_RC_CTRL *pst_rc_ctrl)
{
	if(pst_rc_ctrl->normal_rc_status == DISCONNECT && pst_rc_ctrl->vision_rc_status == DISCONNECT)//两个遥控器都断连，直接全锁住
	{
		pst_rc_ctrl->rc_mode = ALL_LOCKED;
		
		Task_Choice = CHOOSE_RESET_MODE;//遥控器全部断连后全车reset
		Reset_Task_State = OPENED;
	}
	
	
	if(pst_rc_ctrl->rc_mode == NORMAL_RC_OPENED)//硬件遥控器模式下
	{
		if(pst_rc_ctrl->normal_rc_status == DISCONNECT && pst_rc_ctrl->vision_rc_status == CONNECT)//硬件遥控器断连，但视觉遥控器连着
		{
			pst_rc_ctrl->rc_mode = VISION_RC_OPENED;//自动切换到视觉遥控器
			
			Task_Choice = CHOOSE_RESET_MODE;//遥控器切换时全车reset
			Reset_Task_State = OPENED;
		}
	}
	else if(pst_rc_ctrl->rc_mode == VISION_RC_OPENED)//视觉遥控器模式下
	{
		if(pst_rc_ctrl->normal_rc_status == CONNECT && pst_rc_ctrl->vision_rc_status == DISCONNECT)//视觉遥控器断连，但硬件遥控器连着
		{
			pst_rc_ctrl->rc_mode = NORMAL_RC_OPENED;//自动切换到硬件遥控器
			
			Task_Choice = CHOOSE_RESET_MODE;//遥控器切换时全车reset
			Reset_Task_State = OPENED;
		}
	}
	else//全锁住情况下，自动重连(由于全断连的那次就会reset，因此这里重连不需要reset了)
	{
		if(pst_rc_ctrl->normal_rc_status == CONNECT)//此时硬件遥控器连着
			pst_rc_ctrl->rc_mode = NORMAL_RC_OPENED;//自动切换到硬件遥控器
		else if(pst_rc_ctrl->vision_rc_status == CONNECT)//硬件遥控器断连，但视觉遥控器连着
			pst_rc_ctrl->rc_mode = VISION_RC_OPENED;//自动切换到视觉遥控器
	}
}


u8 flag_debounce;
int delay_sum;
void Read_Key_Task(void)
{
		if(Manual_Task_State != LOCKED)
		{
			if(Js_Value.indepen_usJsKey[0] == 39)
			{
				
			}
			if(Js_Value.indepen_usJsKey[1] == 40)
			{
				
			}
			if(Js_Value.indepen_usJsKey[2] == 41)
			{
				
			}
			if(Js_Value.indepen_usJsKey[3] == 42)//J60使能
			{
				ctrl_motor(&hcan2,0x41,&J60_Motor_Ctrl,0);
			}
			if(Js_Value.indepen_usJsKey[4] == 43)//手操放球
			{
				Load_flag = 1;
				Load_State = 0;
			}
			if(Js_Value.indepen_usJsKey[5] == 44)//手操运球，按一次运一次
			{
				Drib_flag = 1;
				Dribble_State = MOTOR_SPEED_UP;
			}
			if(Js_Value.indepen_usJsKey[6] == 45)//手操接球
			{
				Rec_Ball_flag = 1;
				Rec_Ball_State = J60_LIFT;
			}
			if(Js_Value.indepen_usJsKey[7] == 46)//手操发射
			{
				uart1_eft.num[4] = 1;
			}
		}
		if(Js_Value.usJsKey!=0)
		{
			flag_debounce = 1;
		}
		if(flag_debounce == 1)
		{
			if(delay_sum == 0)
			{
				switch(Js_Value.usJsKey)
				{
					case 1://保留一个卸力键，拿来保命
						Reset_Task_State = LOCKED;
						Manual_Task_State = LOCKED;
						Vision_Task_State = LOCKED;
						ShootChal_Task_State = LOCKED;
						DribChal_Task_State = LOCKED;
						nav.nav_state=NAV_OFF;
						break;
					case 2://进入reset任务
						Task_Choice = CHOOSE_RESET_MODE;
						Reset_Task_State = OPENED;
						break;
					case 3://进入普通手操任务
						Task_Choice = CHOOSE_MANUAL_MODE;
						Manual_Task_State = OPENED;
						Manual_Mode = NORMAL_MANUAL;
						break;
					case 4://进入全局手操任务
						Task_Choice = CHOOSE_MANUAL_MODE;
						Manual_Task_State = OPENED;
						Manual_Mode = GLOBA_MANUAL;
						flag_global_manual = 1;
						break;
					case 5://进入自瞄手操任务
						Task_Choice = CHOOSE_MANUAL_MODE;
						Manual_Task_State = OPENED;
						Manual_Mode = AIM_MANUAL;
						break;
					case 6://进入投篮挑战赛任务
	//					Task_Choice = CHOOSE_SHOOTCHAL_MODE;
	//					ShootChal_Task_State = OPENED;
	//					Shoot_Challenge = SHOOT_PATH1;
						break;
					case 7://进入运球挑战赛任务
	//					Task_Choice = CHOOSE_DRIBCHAL_MODE;
	//					DribChal_Task_State = OPENED;
	//					Dribble_Challenge = DRIB_EXIT;
						break;
					case 8://进入视觉自动导航
						Task_Choice = CHOOSE_VISION_MODE;
						Vision_Task_State = OPENED;
						break;
					case 9:
						uart1_efr.num[0] = 1;
						break;
					case 10:
						break;
					case 11:
						break;
					case 12:
						break;
					case 13:
						break;
					case 14:
						break;
					case 15:
						break;
					case 33:
						if(RC_Ctrl.vision_rc_status == CONNECT)
						{
							RC_Ctrl.rc_mode = VISION_RC_OPENED;//进入视觉遥控器，同时封锁底盘的任务
							Task_Choice = CHOOSE_RESET_MODE;
							Reset_Task_State = OPENED;
						}
						break;
					case 34:
						break;
					case 35:
						uart1_eft.num[5] = 1;
						break;
					default :
						break;
				}
			}
			delay_sum++;
	}
	if(delay_sum > 120)
	{
		delay_sum = 0;
		flag_debounce = 0;
	}
	}

void Vision_RC(void)
{
	if(Vision_Data.botton_hat_left == 1)//Reset任务
	{
		Task_Choice = CHOOSE_RESET_MODE;
		Reset_Task_State = OPENED;
	}
	if(Vision_Data.botton_hat_up == 1)//视觉普通手操任务
	{
		Task_Choice = CHOOSE_MANUAL_MODE;
		Manual_Task_State = OPENED;
		Manual_Mode = VISION_NORMAL_MANUAL;
	}
	if(Vision_Data.botton_hat_right == 1)//视觉自瞄手操任务
	{
		Task_Choice = CHOOSE_MANUAL_MODE;
		Manual_Task_State = OPENED;
		Manual_Mode = VISION_AIM_MANUAL;
	}
	if(Vision_Data.botton_hat_down == 1)//进入视觉路径
	{
		Task_Choice = CHOOSE_VISION_MODE;
		Vision_Task_State = OPENED;
	}
	
	if(Manual_Task_State != LOCKED)
	{
		if(Vision_Data.botton_left_shoulder == 1)//视觉手操运球
		{
			Drib_flag = 1;
			Dribble_State = MOTOR_SPEED_UP;
		}
		if(Vision_Data.botton_right_shoulder == 1)//视觉手操发射
		{
			uart1_eft.num[4] = 1;
		}
		if(Vision_Data.botton_a == 1)//视觉手操接球
		{
			Rec_Ball_flag = 1;
			Rec_Ball_State = J60_LIFT;
		}
		if(Vision_Data.botton_b == 1)//视觉手操放球
		{
			Load_flag = 1;
			Load_State = 0;
		}
		if(Vision_Data.botton_x == 1)
		{
			
		}
		if(Vision_Data.botton_y == 1)
		{
			
		}
	}
	if(Vision_Data.botton_minus == 1)//底盘卸力，所有任务封锁
	{
		Reset_Task_State = LOCKED;
		Manual_Task_State = LOCKED;
		Vision_Task_State = LOCKED;
		ShootChal_Task_State = LOCKED;
		DribChal_Task_State = LOCKED;
		nav.nav_state=NAV_OFF;
	}
	if(Vision_Data.botton_plus == 1)
	{
		if(RC_Ctrl.normal_rc_status == CONNECT)
		{
			RC_Ctrl.rc_mode = NORMAL_RC_OPENED;//切换到硬件遥控器，同时封锁底盘的任务
			Task_Choice = CHOOSE_RESET_MODE;
			Reset_Task_State = OPENED;
		}
	}
}
void send_remote_control(void)
{
	int16_t robot_x,robot_y,robot_w;
	int16_t fw_x,fw_y,fw_w;
	int16_t distance,aim_yaw;
	int16_t aim_force,aim_pitch;
	int16_t real_vel1,real_vel2,real_vel3;
	int16_t lift,real_pitch;
	
	robot_x = (int16_t)(stRobot.stPos.fpPosX);
	robot_y = (int16_t)(stRobot.stPos.fpPosY);
	robot_w = (int16_t)(stRobot.stPos.fpPosQ);
	
	fw_x = (int16_t)(stRobot.stPos.fpPosX);
	fw_y = (int16_t)(stRobot.stPos.fpPosY);
	fw_w = (int16_t)(stRobot.stPos.fpPosQ);
	
	distance = Auto_Aim_Data.aim_distance *1000;
	aim_yaw = Auto_Aim_Data.aim_yaw *10;
	aim_force = Auto_Aim_Data.aim_speed;
	aim_pitch = Auto_Aim_Data.aim_pitch;
		
	real_vel1 = (int16_t)(Upboard_Data.FrictionWheel_vel[0]);
	real_vel2 = (int16_t)(Upboard_Data.FrictionWheel_vel[1]);
	real_vel3 = (int16_t)(Upboard_Data.FrictionWheel_vel[2]);
	
	lift = (int16_t)(Upboard_Data.Lift_Flag);
	real_pitch = (int16_t)(Upboard_Data.Real_Pitch);
	
	
	if(get_bit(Upboard_Data.Up_Motor_State,0))
		set_bit1(&nRF24L01_ack_pay.Ack_Buf[1],0);
	else 
		set_bit0(&nRF24L01_ack_pay.Ack_Buf[1],0);
	
	if(get_bit(Upboard_Data.Up_Motor_State,1))
		set_bit1(&nRF24L01_ack_pay.Ack_Buf[1],1);
	else 
		set_bit0(&nRF24L01_ack_pay.Ack_Buf[1],1);
	
	if(get_bit(Upboard_Data.Up_Motor_State,2))
		set_bit1(&nRF24L01_ack_pay.Ack_Buf[1],2);
	else 
		set_bit0(&nRF24L01_ack_pay.Ack_Buf[1],2);
	
	if(get_bit(Upboard_Data.Up_Motor_State,3))
		set_bit1(&nRF24L01_ack_pay.Ack_Buf[1],3);
	else 
		set_bit0(&nRF24L01_ack_pay.Ack_Buf[1],3);
	
	if(get_bit(Upboard_Data.Up_Motor_State,4))
		set_bit1(&nRF24L01_ack_pay.Ack_Buf[1],4);
	else 
		set_bit0(&nRF24L01_ack_pay.Ack_Buf[1],4);
	
	if(get_bit(Upboard_Data.Up_Motor_State,5))
		set_bit1(&nRF24L01_ack_pay.Ack_Buf[1],5);
	else 
		set_bit0(&nRF24L01_ack_pay.Ack_Buf[1],5);
	
	if(get_bit(Upboard_Data.Up_Motor_State,6))
		set_bit1(&nRF24L01_ack_pay.Ack_Buf[1],6);
	else 
		set_bit0(&nRF24L01_ack_pay.Ack_Buf[1],6);
	
	if(get_bit(Upboard_Data.Up_Motor_State,7))
		set_bit1(&nRF24L01_ack_pay.Ack_Buf[1],7);
	else 
		set_bit0(&nRF24L01_ack_pay.Ack_Buf[1],7);
	
	nRF24L01_ack_pay.Ack_Buf[2] = robot_x>>8;
	nRF24L01_ack_pay.Ack_Buf[3] = robot_x &0xff;
	
	nRF24L01_ack_pay.Ack_Buf[4] = robot_y>>8;
	nRF24L01_ack_pay.Ack_Buf[5] = robot_y &0xff;
	
	nRF24L01_ack_pay.Ack_Buf[6] = robot_w>>8;
	nRF24L01_ack_pay.Ack_Buf[7] = robot_w &0xff;
	
	nRF24L01_ack_pay.Ack_Buf[8] = fw_x>>8;
	nRF24L01_ack_pay.Ack_Buf[9] = fw_x &0xff;
	
	nRF24L01_ack_pay.Ack_Buf[10] = fw_y>>8;
	nRF24L01_ack_pay.Ack_Buf[11] = fw_y &0xff;
	
	nRF24L01_ack_pay.Ack_Buf[12] = fw_w>>8;
	nRF24L01_ack_pay.Ack_Buf[13] = fw_w &0xff;
	
	nRF24L01_ack_pay.Ack_Buf[14] = distance>>8;
	nRF24L01_ack_pay.Ack_Buf[15] = distance &0xff;
	
	nRF24L01_ack_pay.Ack_Buf[16] = aim_yaw>>8;
	nRF24L01_ack_pay.Ack_Buf[17] = aim_yaw &0xff;
	
	nRF24L01_ack_pay.Ack_Buf[18] = aim_force>>8;
	nRF24L01_ack_pay.Ack_Buf[19] = aim_force &0xff;
	
	nRF24L01_ack_pay.Ack_Buf[20] = aim_pitch>>8;
	nRF24L01_ack_pay.Ack_Buf[21] = aim_pitch &0xff;
	
	nRF24L01_ack_pay.Ack_Buf[22] = real_vel1>>8;
	nRF24L01_ack_pay.Ack_Buf[23] = real_vel1 &0xff;
	
	nRF24L01_ack_pay.Ack_Buf[24] = real_vel2>>8;
	nRF24L01_ack_pay.Ack_Buf[25] = real_vel2 &0xff;
	
	nRF24L01_ack_pay.Ack_Buf[26] = real_vel3>>8;
	nRF24L01_ack_pay.Ack_Buf[27] = real_vel3 &0xff;
	
	nRF24L01_ack_pay.Ack_Buf[28] = lift>>8;
	nRF24L01_ack_pay.Ack_Buf[29] = lift &0xff;
	
	nRF24L01_ack_pay.Ack_Buf[30] = real_pitch>>8;
	nRF24L01_ack_pay.Ack_Buf[31] = real_pitch &0xff;
}



