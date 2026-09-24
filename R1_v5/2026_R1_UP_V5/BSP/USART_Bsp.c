#include "USART_Bsp.h"
#include "string.h"
#include "MathAlgorithm.h"
#include "tim.h"
#include "StateMachine.h"
//Within board communication
void packDataToLower(u8* data){
	data[0]=0x0a;
	data[22]=0x0b;
	memcpy(data+1,&remote_vel.vel_x,4);
	memcpy(data+5,&remote_vel.vel_y,4);
	memcpy(data+9,&remote_vel.w,4);
	memcpy(data+13,&nav_target,4);
	data[17]=chassis_action;
	data[18]=cmf;
	data[19]=ii_robot.KFS_IDs[0];
	data[20]=ii_robot.KFS_IDs[1];
	data[21]=ii_robot.KFS_IDs[2];
}
u8 nav_cplt_refresh_flag=0,get_KFS_allowed_flag=0;
int16_t pos_x,pos_y,pos_yaw;
u8 unpackDataFromLower(u8* data){
	if(data[0]!=0x0b||data[17]!=0x0a){
		return 0;
	}else{
		memcpy(&nav_progress,data+1,4);
		if(nav_progress>1.5f){
			if(nav_cplt_refresh_flag==0){
				nav_cplt=1;
				nav_cplt_refresh_flag=1;
			}
		}else{
			nav_cplt_refresh_flag=0;
		}
		ii_robot.move_in_flag=data[10];
		memcpy(&nav_leaved_time,data+6,4);
		downFpsError=data[5];
		memcpy(&pos_x,data+11,2);
		memcpy(&pos_y,data+13,2);
		memcpy(&pos_yaw,data+15,2);
//		memcpy(&chassis_vel_rec1,data+11,4);
//		memcpy(&chassis_vel_rec2,data+15,4);
		return 1;
	}
}

//Vision
u8 unpackVisionData(u8* data){
	if(data[0]!=0x66||data[23]!=0x99){
		return 0;
	}else{
		memcpy(&robotPos.pos_x,data+1,4);
		memcpy(&robotPos.pos_y,data+5,4);
		memcpy(&robotPos.Q,data+9,4);
		memcpy(&deltaPos.pos_y,data+13,4);
		return 1;
	}
}

void packVisionData(u8* data){
	data[0]=0x11;
	data[1]=QD_show_final_2;
	data[2]=0x22;
}

//Remote
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
void parseDataPacket(const uint8_t Rx_Buf[9], ST_JS_VALUE *jsValue)
{
    static uint8_t RC_Key_pre_value = 0; // 保存上一次按键值
    jsValue->usJsLeft_X = (Rx_Buf[0] << 8) | Rx_Buf[1];  // 左摇杆X值
    jsValue->usJsLeft_Y = (Rx_Buf[2] << 8) | Rx_Buf[3];  // 左摇杆Y值
    jsValue->usJsRight_X = (Rx_Buf[4] << 8) | Rx_Buf[5]; // 右摇杆X值
    jsValue->usJsRight_Y = (Rx_Buf[6] << 8) | Rx_Buf[7]; // 右摇杆Y值
		if((jsValue->usJsLeft_X+jsValue->usJsLeft_Y+jsValue->usJsRight_X+jsValue->usJsRight_Y)==0){
			jsValue->usJsLeft_X = LEFT_JS_X_MID;
			jsValue->usJsLeft_Y = LEFT_JS_Y_MID;
			jsValue->usJsRight_X = RIGHT_JS_MID;
			jsValue->usJsRight_Y = 2000;
			jsValue->usJsKey = 0;
			return;
		}
    // 按键赋值
    // jsValue->usJsKey = Rx_Buf[8];

    // 只接收一次按键值，防止状态被不断更新
    // 缺点是不能处理连续按两次同一按键的情况
    if (Rx_Buf[8] == RC_Key_pre_value)
    {
        jsValue->usJsKey = 0;
    }
    else
    {
        jsValue->usJsKey = Rx_Buf[8];
        RC_Key_pre_value = jsValue->usJsKey;
    }
		

}



/*******************************************************************************************
函数名称：CalculateVelocities
函数功能：基于手柄摇杆值计算机器人的线速度（X和Y方向）及角速度，并应用低通滤波器进行平滑处理。
输入：   1. jsValue 指向包含手柄摇杆读数的ST_JS_VALUE结构体指针。
          2. velocities 指向用于存储计算得到的速度值的ST_VELT结构体指针。
          3. ucGateX X方向上的阈值，只有超过此阈值才产生运动。
          4. ssXSpedLimit X方向上的最大速度限制（单位mm/s）。
          5. ucGateY Y方向上的速度阈值，只有超过此阈值才产生运动。
          6. ssYSpedLimit Y方向上的最大速度限制（单位mm/s）。
          7. ucGateW 角速度方向上的阈值，只有超过此阈值才产生旋转。
          8. ssWSpedLimit 角速度方向上的最大速度限制（单位0.1度/s）。
输出：   1. velocities 结构体中的成员变量被更新为计算得到的X方向速度（fpVx）、Y方向速度（fpVy）和角速度（fpW）。
备注：     1. 计算过程首先检查是否超过了设定的阈值，如果超过，则按照比例缩放至最大速度限制。
          2. 使用一阶低通滤波器对速度值进行平滑处理，以减少由于手柄输入抖动或用户操作不精确带来的快速加减速。
          3. 注意坐标系转换：左摇杆X轴对应的是fpVx，左摇杆Y轴对应的是fpVy,但要加一个负号，因为摇杆值Y方向从上到下是从0到4096，X方向从右到左是0到4096。
          4. 右摇杆X控制角速度，其值经过特定的比例缩放后赋给fpW。
*******************************************************************************************/
void CalculateVelocities(const ST_JS_VALUE *jsValue, ST_VEL *vel,
                         uint16_t ucGateX, int16_t ssXSpedLimit,
                         uint16_t ucGateY, int16_t ssYSpedLimit,
                         uint16_t ucGateW, int16_t ssWSpedLimit)
{
    const float smooth = 2.5f;                       // 一阶低通滤波器截止频率，单位 Hz
    static ST_LPF FJx = {0, 0, 0, smooth, 0.004f}; // 需要pre_out，故需要设为静态变量
    static ST_LPF FJy = {0, 0, 0, smooth, 0.004f};
    static ST_LPF FJw = {0, 0, 0, smooth, 0.004f};

    // 计算左摇杆X方向的速度 (fpVy)
    float Vx = jsValue->usJsLeft_X - LEFT_JS_X_MID;
    if (fabs(Vx) < ucGateX)
    {
        FJx.in = 0;
    }
    else
    {
        if (Vx > 0)
        {
            FJx.in = (Vx - ucGateX) * ssXSpedLimit / (LEFT_JS_X_MAX - LEFT_JS_X_MID);
        }
        else if (Vx < 0)
        {
            FJx.in = (Vx + ucGateX) * ssXSpedLimit / (LEFT_JS_X_MID - LEFT_JS_X_MIN);
        }
    }

    // 计算左摇杆Y方向的速度 (fpVx)

    float Vy = jsValue->usJsLeft_Y - LEFT_JS_Y_MID;
    if (fabs(Vy) < ucGateY)
    {
        FJy.in = 0;
    }
    else
    {
        if (Vy > 0)
        {
            // Y轴正向偏移
            FJy.in = (Vy - ucGateY) * ssYSpedLimit / (LEFT_JS_Y_MAX - LEFT_JS_Y_MID);
        }
        else if (Vy < 0)
        {
            // Y轴负向偏移
            FJy.in = (Vy + ucGateY) * ssYSpedLimit / (LEFT_JS_Y_MID - LEFT_JS_Y_MIN);
        }
    }

    // 计算右摇杆X方向的角速度 (fpW)
    float Vw = jsValue->usJsRight_X - RIGHT_JS_MID;
    if (fabs(Vw) < ucGateW)
    {
        FJw.in = 0; // 在死区内，设置为0
    }
    else
    {
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

    vel->vel_x = FJx.out;  // 摇杆值X方向从右到左是从0到4096。
    vel->vel_y = -FJy.out; // 摇杆值Y方向从上到下是从0到4096。
    vel->w = -FJw.out;  //  右摇杆X控制角速度
}

/*******************************************************************************************

*******************************************************************************************/
u8 manual_mode=1,III_mode=0,half_manual=0;
u8 preKFS=0,input_index=0;
u8 R2_map[12]={0};
void DealKeyTemp(u8 key){
	if(manual_mode){
		switch(key){
			case 1:
				if(part_flag==10||part_flag==0){
					part_flag=9;
					ii_robot.KFS_IDs[0]=0;
					ii_robot.KFS_IDs[1]=0;
					ii_robot.KFS_IDs[2]=0;
					input_index=0;
				}else if(part_flag==9){
					part_flag=10;
				}
				break;
			case 31:
				if(part_flag!=1&&part_flag!=3){
					chassis_action=1;
				}else{
					weapon_player_action=MOVE_WEAPON_OUT;
				}
				break;
			case 32:
				chassis_action=7;
//				use_KFS_orientaton=!use_KFS_orientaton;
				break;
			case 2:
				manual_mode=1;
				if(chassis_action!=2){
					chassis_action=2;
				}else{
					chassis_action=6;
				}
				break;
			case 3:
				part_flag=0;
				part_step=0;
				break;
			case 4:
				if(part_flag==2||part_flag==3){
					half_manual=0;
				}
				part_flag=1;
				part_step=11;//blank
				chassis_action=2;
				break;
			case 5:
				if(part_flag==3){
					half_manual=0;
				}
				part_flag=2;
				part_step=11;//blank
				chassis_action=2;
				break;
			case 6:
				if(part_flag==1){
					half_manual=0;
				}
				part_flag=3;
				part_step=11;//blank
				chassis_action=2;
				break;
			case 7:
				if(part_flag==1){
					if(half_manual){
#ifdef main_match						
						part_flag=1;
						part_step=10;
						weapon_num=1;
						complete_weapon=0;
#endif
#ifdef adaptive_training						
						nav_target=0x1001;
						chassis_action=3;
#endif						
					}
					weapon_player_action=GET_WEAPON;
				}else if(part_flag==2){
					get_KFS_height=0;
					KFS_master_action=GET_KFS;
				}else if(part_flag==3){
					get_KFS_height=0;
					KFS_master_action=GET_KFS;
#ifdef adaptive_training
					chassis_action=3;
					nav_target=0x3041;
#endif					
				}
				break;
			case 8:
				if(part_flag==1){
					if(half_manual){
#ifdef main_match						
						part_flag=1;
						part_step=10;
						weapon_num=2;
						complete_weapon=0;
#endif
#ifdef adaptive_training						
						nav_target=0x1002;
						chassis_action=3;
#endif		
					}
					weapon_player_action=GET_WEAPON;
				}else if(part_flag==2){
					get_KFS_height=1;
					KFS_master_action=GET_KFS;
				}else if(part_flag==3){
					if(half_manual){
						part_step=30;
					}
				}
				break;
			case 9:
				if(part_flag==1){
					if(half_manual){
#ifdef main_match						
						part_flag=1;
						part_step=10;
						weapon_num=3;
						complete_weapon=0;
#endif
#ifdef adaptive_training						
						nav_target=0x1003;
						chassis_action=3;
#endif		
					}
					weapon_player_action=GET_WEAPON;
				}else if(part_flag==2){
					get_KFS_height=2;
					KFS_master_action=GET_KFS;
				}else if(part_flag==3){
					
				}
				break;
			case 10:
				if(part_flag==1){
					if(half_manual){
#ifdef main_match						
						part_flag=1;
						part_step=10;
						weapon_num=4;
						complete_weapon=0;
#endif
#ifdef adaptive_training						
						nav_target=0x1004;
						chassis_action=3;
#endif		
					}
					weapon_player_action=GET_WEAPON;
				}else if(part_flag==2){
					get_KFS_height=3;
					KFS_master_action=GET_KFS;
				}else if(part_flag==3){
					III_mode=!III_mode;
				}
				break;
			case 14:
				if(part_flag==1){
					if(half_manual){
						
							chassis_action=3;
							nav_target=0x1005;
						
					}
					weapon_player_action=STORE_WEAPON;
				}else if(part_flag==2){
					store_KFS_orientation=STORE_AND_GET_STORED_LEFT;
					KFS_master_action=STORE_KFS;
				}else if(part_flag==3){
					weapon_player_action=USE_WEAPON;
				}
				break;
			case 18:
				if(part_flag==1){
					weapon_player_action=COMBINE_WEAPON;
					if(weapon_player_action==COMBINE_WEAPON){
						chassis_action=13;
					}
				}else if(part_flag==2){
					store_KFS_orientation=STORE_AND_GET_STORED_RIGHT;
					KFS_master_action=STORE_KFS;
				}else if(part_flag==3){
					KFS_master_action=STORE_KFS;
				}
				break;
			case 22:
				if(part_flag==1){
					QD_show_1=1;//组装结束
				}else if(part_flag==2){
					store_KFS_orientation=STORE_AND_GET_STORED_LEFT;
					KFS_master_action=GET_STORED_KFS;
				}else if(part_flag==3){
					KFS_master_action=GET_STORED_KFS;
					delay_timer_flag=5;
				}
				break;
			case 26:
				if(part_flag==2){
					store_KFS_orientation=STORE_AND_GET_STORED_RIGHT;
					KFS_master_action=GET_STORED_KFS;
				}else if(part_flag==3){
					if(half_manual&&pos_y>9550){
						part_step=31;
					}else{
						weapon_player_action=USE_WEAPON;
					}
				}else if(part_flag==1){
					weapon_player_action=USE_WEAPON;
				}
				break;
			case 29:
				half_manual=!half_manual;
				chassis_action=2;
				if(part_flag==2){
					part_step=1;
				}else if(part_flag==1){
					part_step=9;
					timer=0;
				}
				break;
			case 11:
				if(part_flag==2){
					if(half_manual){
						preKFS=1;
						ii_robot.now_KFS_ID=1;
						if(pos_y<9550){
							nav_target=0x2010;
							chassis_action=3;
						}
						get_KFS_height=2;
#ifdef main_match						
						part_step=0;
#endif						
					}
				}else if(part_flag==3&&half_manual){
					if(III_mode==III_DEFAULT){
						nav_target=PUT_KFS_IN_FIRST_COLUMN;
					}else{
						nav_target=MIRRORED_PUT_KFS_IN_FIRST_COLUMN;
					}
					chassis_action=3;
				}else if(part_flag==9){
					ii_robot.KFS_IDs[input_index]=1;
					input_index++;
				}else if(part_flag==1){
					switch(R2_map[0]){
						case 0:
							R2_map[0]=1;
							break;
						case 1:
							R2_map[0]=2;
							break;
						case 2:
							R2_map[0]=3;
							break;
						case 3:
							R2_map[0]=0;
							break;
						default:
							break;
					}
				}
				break;
			case 12:
				if(part_flag==2){
					if(half_manual){
						preKFS=2;
						ii_robot.now_KFS_ID=2;
						if(pos_y<9550){
							nav_target=0x2002;
							chassis_action=3;
						}
						get_KFS_height=1;
#ifdef main_match						
						part_step=0;
#endif	
					}
				}else if(part_flag==3&&half_manual){
					if(III_mode==III_DEFAULT){
						nav_target=PUT_KFS_IN_SECOND_COLUMN;
					}else{
						nav_target=MIRRORED_PUT_KFS_IN_SECOND_COLUMN;
					}
					chassis_action=3;
				}else if(part_flag==9){
					ii_robot.KFS_IDs[input_index]=2;
					input_index++;
				}else if(part_flag==1){
					switch(R2_map[1]){
						case 0:
							R2_map[1]=1;
							break;
						case 1:
							R2_map[1]=2;
							break;
						case 2:
							R2_map[1]=3;
							break;
						case 3:
							R2_map[1]=0;
							break;
						default:
							break;
					}
				}
				break;
			case 13:
				if(part_flag==2){
					if(half_manual){
						preKFS=3;
						ii_robot.now_KFS_ID=3;
						if(pos_y<9550){
							nav_target=0x2014;
							chassis_action=3;
						}
						get_KFS_height=2;
#ifdef main_match						
						part_step=0;
#endif	
					}
				}else if(part_flag==3&&half_manual){
					if(III_mode==III_DEFAULT){
						nav_target=PUT_KFS_IN_THIRD_COLUMN;
					}else{
						nav_target=MIRRORED_PUT_KFS_IN_THIRD_COLUMN;
					}
					chassis_action=3;
				}else if(part_flag==9){
					ii_robot.KFS_IDs[input_index]=3;
					input_index++;
				}else if(part_flag==1){
					switch(R2_map[2]){
						case 0:
							R2_map[2]=1;
							break;
						case 1:
							R2_map[2]=2;
							break;
						case 2:
							R2_map[2]=3;
							break;
						case 3:
							R2_map[2]=0;
							break;
						default:
							break;
					}
				}
				break;
			case 15:
				if(part_flag==2){
					if(half_manual){
						preKFS=4;
						ii_robot.now_KFS_ID=4;
						if(pos_y<9550){
							nav_target=0x2020;
							chassis_action=3;
						}
						get_KFS_height=1;
#ifdef main_match						
						part_step=0;
#endif	
					}
				}else if(part_flag==3&&half_manual){
					if(pos_y>9550){
						if(III_mode==III_MIRRORED){
							nav_target=MIRRORED_PUSH_PLATFORM_FOR_SECOND_COLUMN_R2;
						}
						chassis_action=3;
					}
				}else if(part_flag==9){
					ii_robot.KFS_IDs[input_index]=4;
					input_index++;
				}else if(part_flag==1){
					switch(R2_map[3]){
						case 0:
							R2_map[3]=1;
							break;
						case 1:
							R2_map[3]=2;
							break;
						case 2:
							R2_map[3]=3;
							break;
						case 3:
							R2_map[3]=0;
							break;
						default:
							break;
					}
				}
				break;
			case 16:
				if(part_flag==2){
					if(half_manual){
						if(input_index){
							part_step=20;
						}else{
							if(III_mode==0){
								nav_target=PUT_KFS_IN_SECOND_COLUMN;
							}else{
//								nav_target=0x3004;
							}
							complete_KFS=0;
							KFS_master_action=USE_KFS;
							chassis_action=3;
						}
					}
				}else if(part_flag==3&&half_manual){
					if(pos_y>9550){
						if(III_mode==III_DEFAULT){
							nav_target=PUSH_PLATFORM_FOR_FIRST_COLUMN_R2;
						}else{
							nav_target=MIRRORED_PUSH_PLATFORM_FOR_THIRD_COLUMN_R2;
						}
						chassis_action=3;
					}
				}else if(part_flag==1){
					switch(R2_map[4]){
						case 0:
							R2_map[4]=1;
							break;
						case 1:
							R2_map[4]=2;
							break;
						case 2:
							R2_map[4]=3;
							break;
						case 3:
							R2_map[4]=0;
							break;
						default:
							break;
					}
				}
				break;
			case 17:
				if(part_flag==2){
					if(half_manual){
						preKFS=6;
						ii_robot.now_KFS_ID=6;
						if(pos_y<9550){
							nav_target=0x2024;
							chassis_action=3;
						}
						get_KFS_height=3;
#ifdef main_match						
						part_step=0;
#endif	
					}
				}else if(part_flag==3&&half_manual){
					if(pos_y>9550){
						if(III_mode==III_DEFAULT){
							nav_target=PUSH_PLATFORM_FOR_SECOND_COLUMN_R2;
						}
						chassis_action=3;
					}
				}else if(part_flag==9){
					ii_robot.KFS_IDs[input_index]=6;
					input_index++;
				}else if(part_flag==1){
					switch(R2_map[5]){
						case 0:
							R2_map[5]=1;
							break;
						case 1:
							R2_map[5]=2;
							break;
						case 2:
							R2_map[5]=3;
							break;
						case 3:
							R2_map[5]=0;
							break;
						default:
							break;
					}
				}
				break;
			case 19:
				if(part_flag==2){
					if(half_manual){
						preKFS=7;
						ii_robot.now_KFS_ID=7;
						if(pos_y<9550){
							nav_target=0x2030;
							chassis_action=3;
						}
						get_KFS_height=2;
#ifdef main_match						
						part_step=0;
#endif	
					}
				}else if(part_flag==3){
					QD_show_1=4;//二维码：去左列放块
				}else if(part_flag==9){
					ii_robot.KFS_IDs[input_index]=7;
					input_index++;
				}else if(part_flag==1){
					switch(R2_map[6]){
						case 0:
							R2_map[6]=1;
							break;
						case 1:
							R2_map[6]=2;
							break;
						case 2:
							R2_map[6]=3;
							break;
						case 3:
							R2_map[6]=0;
							break;
						default:
							break;
					}
				}
				break;
			case 20:
				if(part_flag==2){
					if(half_manual){
						KFS_master_action=GET_KFS;
					}
				}else if(part_flag==3){
					QD_show_1=6;//二维码：去中列放块
				}else if(part_flag==1){
					switch(R2_map[7]){
						case 0:
							R2_map[7]=1;
							break;
						case 1:
							R2_map[7]=2;
							break;
						case 2:
							R2_map[7]=3;
							break;
						case 3:
							R2_map[7]=0;
							break;
						default:
							break;
					}
				}
				break;
			case 21:
				if(part_flag==2){
					if(half_manual){
						preKFS=9;
						ii_robot.now_KFS_ID=9;
						if(pos_y<9550){
							nav_target=0x2034;
							chassis_action=3;
						}
						get_KFS_height=2;
#ifdef main_match						
						part_step=0;
#endif	
					}
				}else if(part_flag==3){
					QD_show_1=7;//二维码：去右列放块
				}else if(part_flag==9){
					ii_robot.KFS_IDs[input_index]=9;
					input_index++;
				}else if(part_flag==1){
					switch(R2_map[8]){
						case 0:
							R2_map[8]=1;
							break;
						case 1:
							R2_map[8]=2;
							break;
						case 2:
							R2_map[8]=3;
							break;
						case 3:
							R2_map[8]=0;
							break;
						default:
							break;
					}
				}
				break;
			case 23:
				if(part_flag==2){
					if(pos_y<9550){
						if(half_manual){
							switch(preKFS){
								case 3:
								case 6:
								case 9:
								case 12:
								case 11:
									nav_target=0x2051;
									break;
								default:
									nav_target=0x2040;
									break;
							}
							chassis_action=3;
						}
						get_KFS_height=1;
#ifdef main_match						
						part_step=0;
#endif	
						preKFS=10;
						ii_robot.now_KFS_ID=10;
					}
				}else if(part_flag==3){
					if(QD_show_1!=12){
						QD_show_1=12;//二维码：合体完成
					}else{
						QD_show_1=20;//二维码：R2升起
					}
				}else if(part_flag==9){
					ii_robot.KFS_IDs[input_index]=10;
					input_index++;
				}else if(part_flag==1){
					switch(R2_map[9]){
						case 0:
							R2_map[9]=1;
							break;
						case 1:
							R2_map[9]=2;
							break;
						case 2:
							R2_map[9]=3;
							break;
						case 3:
							R2_map[9]=0;
							break;
						default:
							break;
					}
				}
				break;
			case 24:
				if(part_flag==2){
					if(half_manual){
						preKFS=11;
						ii_robot.now_KFS_ID=11;
						if(pos_y<9550){
							nav_target=0x2052;
							chassis_action=3;
						}
						get_KFS_height=2;
#ifdef main_match						
						part_step=0;
#endif	
					}
				}else if(part_flag==3){
					if(QD_show_1==13){
						QD_show_1=15;//二维码：开始递块
					}else if(QD_show_1==9){
						delay_timer_flag=1;
						complete_KFS=1;
					}else{
						QD_show_1=13;//二维码：完成递块
					}
				}else if(part_flag==9){
					ii_robot.KFS_IDs[input_index]=11;
					input_index++;
				}else if(part_flag==1){
					switch(R2_map[10]){
						case 0:
							R2_map[10]=1;
							break;
						case 1:
							R2_map[10]=2;
							break;
						case 2:
							R2_map[10]=3;
							break;
						case 3:
							R2_map[10]=0;
							break;
						default:
							break;
					}
				}
				break;
			case 25:
				if(part_flag==2){
					if(pos_y<9550){
						if(half_manual){
							switch(preKFS){
								case 1:
								case 4:
								case 7:
								case 10:
								case 11:
									nav_target=0x2053;
									break;
								default:
									nav_target=0x2044;
									break;
							}
							chassis_action=3;
						}
						get_KFS_height=1;
#ifdef main_match						
						part_step=0;
#endif	
						preKFS=12;
						ii_robot.now_KFS_ID=12;
					}
				}else if(part_flag==3){
					if(QD_show_1!=21){
						QD_show_1=21;//二维码：打断
					}else{
						QD_show_1=14;//二维码：放KFS
					}
				}else if(part_flag==9){
					ii_robot.KFS_IDs[input_index]=12;
					input_index++;
				}else if(part_flag==1){
					switch(R2_map[11]){
						case 0:
							R2_map[11]=1;
							break;
						case 1:
							R2_map[11]=2;
							break;
						case 2:
							R2_map[11]=3;
							break;
						case 3:
							R2_map[11]=0;
							break;
						default:
							break;
					}
				}
				break;
			case 27:
				if(III_mode==III_DEFAULT){
					use_KFS_orientaton=USE_KFS_LEFT_OF_R1;
					KFS_master_action=USE_KFS;
				}else if(III_mode==III_MIRRORED){
					use_KFS_orientaton=USE_KFS_RIGHT_OF_R1;
					KFS_master_action=USE_KFS;
				}
				break;
			case 28:
					if(III_mode==III_DEFAULT){
					use_KFS_orientaton=HANDOVER_KFS_AT_RIGHT_OF_R2;
					KFS_master_action=HANDOVER_KFS;
				}else if(III_mode==III_MIRRORED){
					use_KFS_orientaton=HANDOVER_KFS_AT_LEFT_OF_R2;
					KFS_master_action=HANDOVER_KFS;
				}
				break;
			case 30:
				if(part_flag==3){
					if(platform_state==IN||platform_state==STAND_BY){
						platform_action=MOVEOUT;
						if(half_manual){
							delay_timer_flag=6;
						}
					}else{
						platform_action=MOVEIN;
					}
				}
				break;
			case 33:
				complete_weapon=!complete_weapon;
				chassis_action=2;
				break;
			case 34:
				complete_KFS=!complete_KFS;
				break;
			default:
				break;
		}
	}else{
		switch(key){
			case 2:
				manual_mode=1;
				chassis_action=2;
				break;
			case 3:
				part_flag=0;
				break;
			case 4:
				part_flag=1;
				part_step=0;
				break;
			case 5:
				part_flag=2;
				part_step=20;
				break;
			case 6:
				part_flag=3;
				part_flag=0;
				break;
			case 7:
				if(part_flag==1){
					weapon_num=1;
					part_step=10;
				}
				break;
			case 8:
				if(part_flag==1){
					weapon_num=2;
					part_step=10;
				}
				break;
			case 9:
				if(part_flag==1){
					weapon_num=3;
					part_step=10;
				}
				break;
			case 10:
				if(part_flag==1){
					weapon_num=4;
					part_step=10;
				}
				break;
			case 11:
				if(part_flag==3){
					if(III_mode==0){
						nav_target=0x3003;
					}else{
						nav_target=0x3004;
					}
					chassis_action=3;
				}
				break;
			case 12:
				if(part_flag==3){
					if(III_mode==0){
						nav_target=0x3002;
					}else{
						nav_target=0x3005;
					}
					chassis_action=3;
				}
				break;
			case 13:
				if(part_flag==3){
					if(III_mode==0){
						nav_target=0x3003;
					}else{
						nav_target=0x3006;
					}
					chassis_action=3;
				}
				break;
			case 15:
				if(part_flag==3){
					if(III_mode==0){
						nav_target=0x3011;
					}else{
						nav_target=0x3014;
					}
					chassis_action=3;
				}
				break;
			case 16:
				if(part_flag==3){
					if(III_mode==0){
						nav_target=0x3012;
					}else{
						nav_target=0x3015;
					}
					chassis_action=3;
				}
				break;
			case 17:
				if(part_flag==3){
					if(III_mode==0){
						nav_target=0x3013;
					}else{
						nav_target=0x3016;
					}
					chassis_action=3;
				}
				break;
			case 23:
				if(part_flag==2){
					if(half_manual){
						nav_target=0x2040;
						chassis_action=3;
						get_KFS_height=1;
					}
				}else if(part_flag==3){
					QD_show_1=12;//二维码：合体完成
				}
				break;
			case 24:
				if(part_flag==2){
					if(half_manual){
						nav_target=0x2052;
						chassis_action=3;
						get_KFS_height=2;
					}
				}else if(part_flag==3){
					QD_show_1=13;//二维码：递块成功
				}
				break;
			case 25:
				if(part_flag==2){
					if(half_manual){
						nav_target=0x2044;
						chassis_action=3;
						get_KFS_height=1;
					}
				}else if(part_flag==3){
					QD_show_1=14;//二维码：可以放高层
				}
				break;
			case 22:
				if(part_flag==3){
					store_KFS_orientation=!III_mode;
					KFS_master_action=GET_STORED_KFS;
				}
				break;
			case 26:
				if(part_flag==3){
					III_mode=!III_mode;
				}
				break;
			case 27:
				if(part_flag==3){
					use_KFS_orientaton=!III_mode;
					KFS_master_action=USE_KFS;
				}
				break;
			case 28:
				if(part_flag==3){
					use_KFS_orientaton=!III_mode;
					KFS_master_action=HANDOVER_KFS;
				}
				break;
			case 29:
				chassis_action=2;
				part_step=11;//blank
				half_manual=1;
			case 30:
				if(part_flag==3){
					if(platform_state==IN||platform_state==STAND_BY){
						platform_action=MOVEOUT;
					}else{
						platform_action=MOVEIN;
					}
				}
				break;
			case 33:
				complete_weapon=!complete_weapon;
				break;
			case 34:
				complete_KFS=!complete_KFS;
				break;
			default:
				break;
		}
	}
}

void search_KFS_information(void){
	switch(ii_robot.now_KFS_ID){
		case 1:
			ii_robot.KFS_height=2;
			break;
		case 2:
			ii_robot.KFS_height=1;
			break;
		case 3:
			ii_robot.KFS_height=2;
			break;
		case 4:
			ii_robot.KFS_height=1;
			break;
		case 6:
			ii_robot.KFS_height=3;
			break;
		case 7:
			ii_robot.KFS_height=2;
			break;
		case 9:
			ii_robot.KFS_height=2;
			break;
		case 10:
			ii_robot.KFS_height=1;
			break;
		case 11:
			ii_robot.KFS_height=2;
			break;
		case 12:
			ii_robot.KFS_height=1;
			break;
		default:
			break;
	}
	if(ii_robot.R1_KFS_cnt==0){
		ii_robot.KFS_store=0;
	}else if(ii_robot.R1_KFS_cnt==1||ii_robot.R1_KFS_cnt==2){
		ii_robot.KFS_store=1;
	}else{
		ii_robot.KFS_store=0;
	}
}

void packRemoteData(u8* data){
		data[0]=0x0A;
		data[1]=0;
		for (u8 i = 0; i <= 4; i++) {
        if (systemMonitor.error[i]) {
            data[1] |= (1 << i); 
        }
    }
		for (u8 i = 6; i <= 8; i++) {
				if (systemMonitor.error[i]) {
						data[1] |= (1 << (i - 1)); 
				}
		}
		data[2]=downFpsError;
		data[3]=(part_flag << 4)|(chassis_action & 0x0F);
		data[4]=(half_manual << 4) | (manual_mode & 0x0F);
//		int16_t int_x=(int16_t) robotPos.pos_x;
//		memcpy(data+5,&int_x,2*sizeof(uint8_t));
//		int16_t int_y=(int16_t) robotPos.pos_y;
//		memcpy(data+7,&int_y,2*sizeof(uint8_t));
//		int16_t int_q=(int16_t) robotPos.Q;
//		memcpy(data+9,&int_q,2*sizeof(uint8_t));
		memcpy(data+5,&pos_x,2);
		memcpy(data+7,&pos_y,2);
		memcpy(data+9,&pos_yaw,2);
		data[11]=(weapon_cplt << 4) | (KFS_cplt & 0x0F);
		switch(QD_show_1){
			case 0:
				QD_show_final_1=0;
				break;
			case 1:
				QD_show_final_1=1;
				break;
			case 4:
				QD_show_final_1=2;
				break;
			case 6:
				QD_show_final_1=3;
				break;
			case 7:
				QD_show_final_1=4;
				break;
			case 12:
				QD_show_final_1=5;
				break;
			case 13:
				QD_show_final_1=6;
				break;
			case 14:
				QD_show_final_1=7;
				break;
			case 15:
				QD_show_final_1=9;
				break;
			case 20:
				QD_show_final_1=10;
				break;
			case 21:
				QD_show_final_1=11;
				break;
		}
		data[12]=(get_KFS_height << 4) | (QD_show_final_1 & 0x0F);
		data[13]=ii_robot.KFS_IDs[0];
		data[14]=ii_robot.KFS_IDs[1];
		data[15]=ii_robot.KFS_IDs[2];
		pack_2bit_data_simple(R2_map,data+16);
		data[19]=III_mode;
		data[23]=0x0B;
}

void packDataToIRModule(u8 *buf){
	buf[0]=0x55;
	buf[1]=0xaa;
	
	buf[2]=0x51;
	pack_2bit_data_simple(R2_map,buf+3);
	
	buf[6]=0x66;
//	buf[7]=1;
	buf[8]=0x55;
}
void unpackDataFromIRModule(u8 *buf){
	return;
}
