#include "communication.h"

/****视觉定位*****/
void Vision_Data_Deal(vision_Data_t *p_vision_data)
{
		
		//雷达位姿
    memcpy(&p_vision_data->radar_x, &vision_rec[1], 4);
    memcpy(&p_vision_data->radar_y, &vision_rec[5], 4);
    memcpy(&p_vision_data->radar_z, &vision_rec[9], 4);
    memcpy(&p_vision_data->radar_yaw, &vision_rec[13], 4);
	
	  // 入口点序号、路径编号 
    memcpy(&p_vision_data->entry_kfs_id, &vision_rec[17], 1);
    memcpy(&p_vision_data->path_number,  &vision_rec[18], 1);
	
	 //状态数组
    memcpy(&p_vision_data->state_1, &vision_rec[19], 1);
    memcpy(&p_vision_data->state_2, &vision_rec[20], 1);
    memcpy(&p_vision_data->state_3, &vision_rec[21], 1);
    memcpy(&p_vision_data->state_4, &vision_rec[22], 1);
		
	//四个台阶上取KFS位置
	  memcpy(&p_vision_data->s1_x, &vision_rec[23],  4);
    memcpy(&p_vision_data->s1_y, &vision_rec[27],  4);
    memcpy(&p_vision_data->s1_side_left_x, &vision_rec[31],  4);
    memcpy(&p_vision_data->s1_side_left_y, &vision_rec[35],  4);
    memcpy(&p_vision_data->s1_side_right_x, &vision_rec[39],  4);
    memcpy(&p_vision_data->s1_side_right_y, &vision_rec[43],  4);
    
	  memcpy(&p_vision_data->s2_x, &vision_rec[47],  4);
    memcpy(&p_vision_data->s2_y, &vision_rec[51],  4);
    memcpy(&p_vision_data->s2_side_left_x, &vision_rec[55],  4);
    memcpy(&p_vision_data->s2_side_left_y, &vision_rec[59],  4);
    memcpy(&p_vision_data->s2_side_right_x, &vision_rec[63],  4);
    memcpy(&p_vision_data->s2_side_right_y, &vision_rec[67],  4);
    
	  memcpy(&p_vision_data->s3_x, &vision_rec[71],  4);
    memcpy(&p_vision_data->s3_y, &vision_rec[75],  4);
    memcpy(&p_vision_data->s3_side_left_x, &vision_rec[79],  4);
    memcpy(&p_vision_data->s3_side_left_y, &vision_rec[83],  4);
    memcpy(&p_vision_data->s3_side_right_x, &vision_rec[87],  4);
    memcpy(&p_vision_data->s3_side_right_y, &vision_rec[91],  4);
    
	  memcpy(&p_vision_data->s4_x, &vision_rec[95],  4);
    memcpy(&p_vision_data->s4_y, &vision_rec[99],  4);
    memcpy(&p_vision_data->s4_side_left_x, &vision_rec[103],  4);
    memcpy(&p_vision_data->s4_side_left_y, &vision_rec[107],  4);
    memcpy(&p_vision_data->s4_side_right_x, &vision_rec[111],  4);
    memcpy(&p_vision_data->s4_side_right_y, &vision_rec[115],  4);
	
		
    //四个台阶中心的坐标（锚点）
    memcpy(&p_vision_data->s1_center_x, &vision_rec[119], 4);
    memcpy(&p_vision_data->s1_center_y, &vision_rec[123], 4);
		
    memcpy(&p_vision_data->s2_center_x, &vision_rec[127], 4);
    memcpy(&p_vision_data->s2_center_y, &vision_rec[131], 4);
		
    memcpy(&p_vision_data->s3_center_x, &vision_rec[135], 4);
    memcpy(&p_vision_data->s3_center_y, &vision_rec[139], 4);
		
    memcpy(&p_vision_data->s4_center_x, &vision_rec[143], 4);
    memcpy(&p_vision_data->s4_center_y, &vision_rec[147], 4);		
		
		//二维码识别
	  memcpy(&p_vision_data->aruco_detect_flag, &vision_rec[151], 1);
		
		//要取KFS总数
		memcpy(&p_vision_data->KFS_number, &vision_rec[152], 1);
		
		//一区夹头顺序
		memcpy(&p_vision_data->header_num_1, &vision_rec[153], 1);
		memcpy(&p_vision_data->header_num_2, &vision_rec[154], 1);
		memcpy(&p_vision_data->header_num_3, &vision_rec[155], 1);
		memcpy(&p_vision_data->header_num_4, &vision_rec[156], 1);
		memcpy(&p_vision_data->header_num_5, &vision_rec[157], 1);
		memcpy(&p_vision_data->header_num_6, &vision_rec[158], 1);
		
		//重试的四个台阶状态
		memcpy(&p_vision_data->state_1_repeat, &vision_rec[159], 1);
		memcpy(&p_vision_data->state_2_repeat, &vision_rec[160], 1);
		memcpy(&p_vision_data->state_3_repeat, &vision_rec[161], 1);
		memcpy(&p_vision_data->state_4_repeat, &vision_rec[162], 1);		
		
		memcpy(&p_vision_data->arcuo_left_or_right,&vision_rec[163],1);
		
		memcpy(&p_vision_data->R1_two,&vision_rec[164],1);
		
		
		robot_pos.fpPosX =   p_vision_data->radar_x;
		robot_pos.fpPosY =   p_vision_data->radar_y;
		robot_pos.fpPosZ = p_vision_data->radar_z;
//	robot_pos.fpPosQ = ConvertAngle(p_vision_data->radar_yaw);
		
		nav.auto_path.pos_pid.pid_x.fpFB = robot_pos.fpPosX;
		nav.auto_path.pos_pid.pid_y.fpFB = robot_pos.fpPosY;		
		nav.auto_path.pos_pid.pid_w.fpFB = robot_pos.fpPosQ;
}


//视觉二维码 vision_data_recieve.aruco_detect_flag
//	1 对接成功
//	2 三区1列
//	3 三区2列
//	4 三区3列
//	5 合体成功收腿
//	6 可以从R1上拿走KFS
//	7 放KFS
//	8 从R1上传递KFS
//  9	直接合体
//  10 强制打断







void choose_pid_yaw(void)
{
	switch(gyro_or_radar)
	{
		case 0: //用陀螺仪
		point_only.pid_w.fpKp = 2.5f;point_only.pid_w.fpUpMax = 8.0f;
		point_2_area.pid_w.fpKp = 2.5f;point_2_area.pid_w.fpUpMax = 8.0f;
		NAV_LOCK_POS_PID.pid_w.fpKp = 2.3f;NAV_LOCK_POS_PID.pid_w.fpUpMax = 8.0f;		
		NAV_DT35_PID.pid_w.fpKp = 2.5f;NAV_DT35_PID.pid_w.fpUpMax = 8.0f;		
			break;
		
		case 1: //雷达
		point_only.pid_w.fpKp = 1.55f;point_only.pid_w.fpUpMax = 8.0f;
		point_2_area.pid_w.fpKp = 1.55f;point_2_area.pid_w.fpUpMax = 8.0f;
		NAV_LOCK_POS_PID.pid_w.fpKp = 1.5f;NAV_LOCK_POS_PID.pid_w.fpUpMax = 8.0f;		
		NAV_DT35_PID.pid_w.fpKp =1.6f;NAV_DT35_PID.pid_w.fpUpMax = 8.0f;	
			break;
		
		default:
			break;
	}
}




//视觉发送，发1视觉会传数据
void vision_send_task(void)
{
	vision_send[0] = 0x66;
	vision_send[1] = 1;
	vision_send[2] = 0x99;
	HAL_UART_Transmit_DMA(&huart6,vision_send,sizeof(vision_send));
}



//陀螺仪雷达融合获取角度（大多数时候相信陀螺仪，死掉会用雷达）

float GET_YAW(float gyro_yaw,float radar_yaw,float gyro_w)
{ 
	static uint8_t start_delay_200 = 0; //上电等200ms再开始
	static uint8_t inited = 0; //初始化一次
	static uint8_t stuck_cnt = 0;  //陀螺仪损坏计数
	static float prev_gyro = 0; //上一次陀螺仪角度
	static float pos_yaw = 0;  //真正用到导航里的YAW角
	static uint8_t gyro_dead_flag = 0; //陀螺仪死掉了
	static float gyro_offset = 0;  //陀螺仪纠偏量
	static float gyro_real = 0;  //陀螺仪加上纠偏之后的数据

  static uint16_t check_cnt = 0;        //  检测间隔计数
  static float last_diff = 0;           //  上次记录的差值
  static uint8_t need_fix = 0;          //  需要修正
	
	
    if(!inited) {
        start_delay_200++;
        if(radar_yaw != 0) {
            pos_yaw = radar_yaw;  // 延时期间先用雷达
        }
        if(start_delay_200 >= 200) {  // 200ms到了
            if(radar_yaw != 0) {
                gyro_offset = radar_yaw - gyro_yaw;
            }
            gyro_real = gyro_yaw + gyro_offset;
            prev_gyro = gyro_real;
            pos_yaw = gyro_real;
            inited = 1;
						start_delay_200 = 0;
        }
        return pos_yaw;
    }		
	
	
	gyro_real = gyro_yaw + gyro_offset;

   // 每隔一段时间检测差值
    check_cnt++;
    if(check_cnt >= 200 && radar_yaw != 0) {
        check_cnt = 0;
        last_diff = fabs(radar_yaw - gyro_real);
        if(last_diff > 0.03f) {  // 差值大于0.03rad需要修正
            need_fix = 1;
        }
    }

		
    if(need_fix && fabs(gyro_w) < 0.3f && radar_yaw != 0) {
        gyro_offset = radar_yaw - gyro_yaw;
        gyro_real = gyro_yaw + gyro_offset;
        need_fix = 0;
    }
		
		
		if(!gyro_dead_flag&&inited) //陀螺仪正常工作
	{
		
			// === 陀螺仪处理 ===
			if(gyro_real == prev_gyro) 
			{
        stuck_cnt++;
				if(stuck_cnt > 10) {gyro_dead_flag = 1;}// 判死刑
			} else {
								stuck_cnt = 0;
								pos_yaw = gyro_real;
								gyro_or_radar = 0;
							}		
		
	 prev_gyro = gyro_real;
	}else if(gyro_dead_flag&&inited)
		{
			pos_yaw = radar_yaw;
			gyro_or_radar = 1;
		  need_fix = 0;    
		}
	
	 return pos_yaw;
}


//板间通信通讯协议 send
//帧头帧尾  inner_send[0] 0xAA   inner_send[7] 0xBB
//总任务  inner_send[1]  0占位 1夹头 2 （空）  3对接完成  8站在地面放中层KFS  9站在R1上面取背后KFS  10站在R1放高层 11站在R1取R1KFS 12（崇武探幽恢复夹爪） 13上下400前控制机械臂姿态  14上三区改机械臂姿态 100(占位)  17R1喷气，R2可以收回机械臂  19三区重试   20初始化 200（用来解锁）
//											 15 //站起来给机械臂发改姿态    
//二区取块高度 inner_send[2] （相对于轮子，存在三种） 0初始化 1低200 2高200 3高400
//二区取块方向 inner_send[3] （相对于车身正面） 0初始化 1左 2正 3右
//二区取块动作 inner_send[4] 0初始  1扔块（向后扔） 2存块 3手持（只有取最后一个KFS才会手持）

//inner_send[5] 取块动作  0初始化 1做出准备取kfs动作 2取二区kfs 3取三区地面kfs（九宫藏宝） 
//inner_send[6] R1在R2左侧发1  在R2右侧发2

void usart_inner_send(void)
{
	inner_send[0] = 0xAA;
	
	inner_send[7] = 0xBB;		
	HAL_UART_Transmit_DMA(&huart4,inner_send,sizeof(inner_send));
}





//板间通信通讯协议 receive
//帧头帧尾  inner_receive[0] 0xBB  inner_receive[17] 0xAA
// inner_receive[1]  0初始化 1取头成功 2取头失败(底盘需要前往下一个武器头位置) 3夹住了但是甩掉
//inner_receive[2] 0初始化 1正在执行动作 2执行完成 3未执行完成，但底盘可移动
//inner_receive[3] 0初始化 1正在放置KFS 2放置完成
//inner_receive[4] 0初始化 1正在取R1KFS 2取R1KFS完成
//inner_receive[5] 0初始化 1正在执行准备动作 2准备动作执行完成
//inner_receive[6] 0初始化 1正在执行机械臂变姿态动作 2机械臂变姿态动作执行完成
//inner_receive[7] 上层帧率
//inner_receive[8] 上层帧率

//inner_receive[9] -- inner_receive[12]  陀螺仪角度（度）
//inner_receive[13] -- inner_receive[16]  陀螺仪角速度

void usart_inner_receive(void)
{
	size_t inner_len = sizeof(uart4_receive);
	for(uint8_t a = 0;a<inner_len;a++){
		if(uart4_receive[a] == 0xBB && uart4_receive[(a+inner_len-1)%inner_len] == 0xAA)
		{	memcpy(&inner_receive,&uart4_receive[a],inner_len-a);
			memcpy(&inner_receive[inner_len-a],&uart4_receive,a);
			memcpy(&yaw_gyro,&inner_receive[9],4);
			memcpy(&gyro_w,&inner_receive[13],4);
			if(fabs(yaw_gyro)<=1800)
			{yaw_gyro_rad = ConvertAngle(yaw_gyro * RADIAN);}
			else{yaw_gyro_rad=0;}
			break;
		}
	}
}



//总通信逻辑
//R2启动，导航到第一个夹头位置，令inner_send[1] = 1 ，{若inner_receive[1] = 1，则导航到对接位置，令inner_send[1] = 2}，
//{若inner_receive[1] = 2，先令inner_send[1] = 0，后导航到下一个取头位置，令inner_send[1] = 1，直到inner_receive[1] = 1导航到对接位置，令inner_send[1] = 2}

//R2收到R1二维码显示对接成功后，令inner_send[1] = 3，延时1s等待夹爪打开，给R1让路，等两秒R1先进二区，延时结束后直接导航到第一个取块点，若不需要入口取块那么直接导航到规划的那条路径
//取块时，inner_send[2]，inner_send[3]，inner_send[4]先赋值，最后令inner_send[1] = 4，若inner_receive[2] = 2，先令inner_send[1] = 0，
//再导航到下一个取块点或登下一个台阶，先给inner_send[2]，inner_send[3]，inner_send[4]赋值，再令inner_send[1] = 4

//下二区之后直接导航到三区入口上坡，到达识别R1二维码的位置到了之后做一个判断，若识别二维码获取放在哪一个中层那么按要求放置，若在3s内没有识别到那么直接放中间，先是导航到目标方块位置，然后令inner_send[1] = 5，
//若inner_receive[3] = 2，那么先令inner_send[1] = 0，判断：若R2存有一个KFS，直接令inner_send[1] = 6，若没有保持inner_send[1] = 0
//导航到登R1位置站起，R1将平台伸出，收到二维码获取到R1完全伸出后，收腿的同时，判断：若手里持有一个那么直接令inner_send[1] = 7，若没有那么直接令inner_send[1] = 8，若vision_data_recieve.aruco_detect_flag=6，那么直接令inner_send[1] = 0



//硬件键盘，输入对应按键控制动作

void usart_all_ctrl(void)
{
		size_t key_len = 4;
	for(uint8_t b = 0;b<key_len;b++){
		if(uart_ctrl_receive[b] == 0xAA && uart_ctrl_receive[(b+key_len-1)%key_len] == 0xBB)
		{	memcpy(&key_receive,&uart_ctrl_receive[b],key_len-b);
			memcpy(&key_receive[key_len-b],&uart_ctrl_receive,b);
			monitor.rate_cnt.remote++;
			break;
		}
	}
}


//硬件键盘发送，控制状态灯

void usart_all_ctrl_send(void)
{
	key_send[0] = 0xAA;
	key_send[4] = 0xBB;	
	HAL_UART_Transmit_DMA(&huart3,key_send,sizeof(key_send));
}



//灯带
uint8_t color[LED_Count][3];

void WS2812_SET(uint8_t index,uint8_t r,uint8_t g,uint8_t b)
{
        color[index][0]=r;
        color[index][1]=g;
        color[index][2]=b;
}

void WS2812_AllSet(uint8_t r,uint8_t g,uint8_t b)
{
        for(int i=0;i<LED_Count;i++)
        {
                WS2812_SET(i,r,g,b);
        }
}


void WS2812_Update(void)
{
        static uint16_t data[LED_Count*3*8+1];
        for(int i=0;i<LED_Count;i++)
        {
                uint8_t r=color[i][0];
                uint8_t g=color[i][1];
                uint8_t b=color[i][2];
                for(int j=0;j<8;j++)
                {
                        data[24*i+j]=(r&(0x80>>j))?CODE1:CODE0;
                        data[24*i+j+8]=(g&(0x80>>j))?CODE1:CODE0;
                        data[24*i+j+16]=(b&(0x80>>j))?CODE1:CODE0;
                }        
        }
        data[LED_Count*3*8]=CODEReset;
        HAL_TIM_PWM_Stop_DMA(&htim1,TIM_CHANNEL_3);
        __HAL_TIM_SetCounter(&htim1,0);
        HAL_TIM_PWM_Start_DMA(&htim1,TIM_CHANNEL_3,(uint32_t*)data,sizeof(data)/sizeof(uint16_t));
}

