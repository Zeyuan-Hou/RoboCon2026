#include "up_down_step.h"

//本文件只有上下台阶状态机，总状态机跟导航结合，放在了path.c里

void GO1_pid_change(void)
{
	static uint8_t wait_timer = 0;
	
	switch(GO1_state_change)
	{
		case 0:
		wait_timer++;
		if(wait_timer>=200){
		cmd_1.K_P = 2.8;
		cmd_1.K_W = 0.035;
		cmd_2.K_P = 2.8;
		cmd_2.K_W = 0.035;
		cmd_3.K_P = 2.8;
		cmd_3.K_W = 0.035;
		cmd_4.K_P = 2.8;
		cmd_4.K_W = 0.035;
		up_down_state = 5;
		}
			break;

		case 1://常态（收腿）	
		GO1_td_1.r = 7500;
		GO1_td_2.r = 7500;
		GO1_td_3.r = 7500;
		GO1_td_4.r = 7500;
		
		cmd_1.T = 0;
		cmd_2.T = 0;
		cmd_3.T = 0;		
		cmd_4.T = 0;
			break;

		case 2://站立
		GO1_td_1.r = 650;
		GO1_td_2.r = 650;
		GO1_td_3.r = 650;
		GO1_td_4.r = 650;
		cmd_1.T =  0.64;
		cmd_2.T = -0.75;
		cmd_3.T =  0.75;
		cmd_4.T = -0.64;
			break;

		case 3://下台阶后，整车落地
		GO1_td_1.r = 420;
		GO1_td_2.r = 420;
		GO1_td_3.r = 420;
		GO1_td_4.r = 420;
		cmd_1.T = 0;
		cmd_2.T = 0;		
		cmd_3.T = 0;		
		cmd_4.T = 0;
			break;
		
		case 4://不设置重力前馈，状态机运行中设置，用于四个腿不全都需要重力前馈的情况
		GO1_td_1.r = 7500;
		GO1_td_2.r = 7500;
		GO1_td_3.r = 7500;
		GO1_td_4.r = 7500;
			break;		
		
		case 5://空状态
			break;

		default :
			break;
	}
}


void up_logic_200(void)
{
	static uint8_t Timer_wait_200 = 0;
	static uint8_t lift_first_flag = 0;
	static uint8_t flag_key_change = 0;
	
	static uint8_t Timer_delay_air = 0;  //气缸延时
		
	switch(up_state_200)
	{
	
		case 0://初始
			break;
		
		case 1://切换GO1参数到上台阶 R2站立 辅助轮弹出
			if(travel_switch_mode[0]==0){flag_key_change=1;}
			dji_run.flag_2006_V = 1;
			if(lift_first_flag == 0)
			{lift_state = 1;lift_first_flag = 1;}
			test_lift(2.95);	
			if(lift_state==0)
			{AirOperaterCtrlBuf[4] = 1;
				up_state_200 = 2;}
	
			break;
		
		case 2://2006驱动轮子向前跑
		if(travel_switch_mode[0]==0){flag_key_change=1;}
		nav.expect_robot_global_velt.fpY = -600;
		if(flag_key_change)
		{if(travel_switch_mode[0]==1){up_state_200 = 3;}}//前方行程开关被遮挡
			break;
		
		case 3://切换GO1参数 收前腿 四个全向轮开始转动
			nav.expect_robot_global_velt.fpY = -280;
			GO1_state_change = 4;
			cmd_1.T = 0;
			cmd_4.T = 0;		
			cmd_1.Target_Delta_Deg = -0.35;
			cmd_4.Target_Delta_Deg = 0.35;		
			if(travel_switch_mode[1]==1){lift_first_flag = 0;nav.expect_robot_global_velt.fpY = -850;up_state_200 = 4;}
			break;
		
		case 4:
		Timer_delay_air++;
		if(Timer_delay_air>=200){AirOperaterCtrlBuf[4] = 0;}
		if(travel_switch_mode[4]==1){nav.expect_robot_global_velt.fpY = -260;}
		
		if(travel_switch_mode[2]==1)	//后面两个全向轮已登上
		{nav.expect_robot_global_velt.fpY = -260;up_state_200 = 5;}
		
			break;

		case 5://切换GO1参数 收腿 调整底盘速度（慢速）
			AirOperaterCtrlBuf[4] = 0;
			GO1_state_change = 4;		
			cmd_1.T = 0;
			cmd_4.T = 0;		
			cmd_2.T = 0;
			cmd_3.T = 0;		
			GO1_fold();	
		
		if(travel_switch_mode[3]==1||fabs(cmd_2.target_td-0.35)<=0.02||fabs(cmd_3.target_td+0.35)<=0.02)//最后面的行程开关被遮挡	
			{	
				nav.expect_robot_global_velt.fpY = -1000;
				Timer_wait_200++;
				if(Timer_wait_200>=80)
				{up_state_200 = 6;}//这段让底盘以原速度跑，过程更流畅
			}		
			break;
		
		
		case 6:// 登200完成
		Timer_delay_air = 0;
		flag_key_change = 0;
		nav.expect_robot_global_velt.fpY = 0;
		dji_run.flag_2006_V = 0;
		AirOperaterCtrlBuf[4] = 0;		
		GO1_state_change = 1;
		lift_first_flag = 0;
		GO1_fold();
		Timer_wait_200 = 0;
		up_state_200 = 0;						
			break;
		
		
		default:
			break;
	}
}




void up_logic_400(void)
{
	static uint8_t Timer_wait_400 = 0;
	static uint8_t lift_first_flag = 0;
	static uint8_t flag_key_change = 0;

	static uint16_t Timer_delay_air = 0;  //气缸延时
	
	switch(up_state_400)
	{
	
		case 0://初始
			break;
		
		case 1://切换GO1参数到上台阶 R2站立 辅助轮弹出
			if(travel_switch_mode[0]==0){flag_key_change = 1;}
			
			dji_run.flag_2006_V = 1;
			if(lift_first_flag == 0)
			{lift_state = 1;lift_first_flag = 1;}
			test_lift(5.75);	
			if(lift_state==0)
			{AirOperaterCtrlBuf[4] = 1;
				up_state_400 = 2;}
	
			break;
		
		case 2://2006驱动轮子向前跑
			if(travel_switch_mode[0]==0){flag_key_change = 1;}
		
				nav.expect_robot_global_velt.fpY = -300;
if(flag_key_change){if(travel_switch_mode[0]==1){	up_state_400 = 3;}} //前方行程开关被遮挡
			break;
		
		case 3://切换GO1参数 收前腿 四个全向轮开始转动
			GO1_state_change = 4;
			cmd_1.T = 0;
			cmd_4.T = 0;		
			cmd_1.Target_Delta_Deg = -0.35;
			cmd_4.Target_Delta_Deg = 0.35;		
			nav.expect_robot_global_velt.fpY = -280;
			if(travel_switch_mode[1]==1){lift_first_flag = 0;	up_state_400 = 4;nav.expect_robot_global_velt.fpY = -600;}//行程开关检测到前两个腿已经到达台阶上
		
			break;
		
		case 4:
		Timer_delay_air++;
		if(Timer_delay_air>=500){AirOperaterCtrlBuf[4] = 0;}
			
		if(travel_switch_mode[4]==1){nav.expect_robot_global_velt.fpY = -400;}		
		
		if(travel_switch_mode[2]==1)	//后面两个全向轮已登上
		{nav.expect_robot_global_velt.fpY = -200;up_state_400 = 5;}
		
			break;

		case 5://切换GO1参数 收腿 调整底盘速度（慢速）
			AirOperaterCtrlBuf[4] = 0;
			GO1_state_change = 4;		
			cmd_1.T = 0;
			cmd_4.T = 0;		
			cmd_2.T = 0;
			cmd_3.T = 0;		
			GO1_fold();	
		
		if(travel_switch_mode[3]==1||fabs(cmd_2.target_td-0.35)<=0.02||fabs(cmd_3.target_td+0.35)<=0.02)//最后面的行程开关被遮挡	
			{	
				nav.expect_robot_global_velt.fpY = -1000;
				Timer_wait_400++;
				if(Timer_wait_400>=80)
				{up_state_400 = 6;} //这段让底盘以原速度跑，过程更流畅
			}		
			break;
		
		
		case 6:// 登400完成
		Timer_delay_air = 0;
		flag_key_change=0;
		nav.expect_robot_global_velt.fpY = 0;
		dji_run.flag_2006_V = 0;
		AirOperaterCtrlBuf[4] = 0;		
		GO1_state_change = 1;
		lift_first_flag = 0;
		GO1_fold();			
		Timer_wait_400 = 0;
		up_state_400 = 0;						
			break;
		
		
		default:
			break;
	}

}




void down_logic_200(void)
{
	static uint8_t timer_wait_200 = 0;
	static uint8_t timer_wait_key_200 = 0;
	switch(down_state_200)
	{
	
		case 0://初始
			break;
		
		case 1://底盘前进（快速）  辅助轮弹出		后腿放下让两个2006轮子接触地面，并转动
			dji_run.flag_2006_V = 1;
			GO1_state_change = 4;			
			cmd_2.Target_Delta_Deg = 0;
			cmd_3.Target_Delta_Deg = 0;		
			nav.expect_robot_global_velt.fpY = -810;
			AirOperaterCtrlBuf[5] = 1;
			if(travel_switch_mode[1]==0) //行程开关无遮挡，代表前腿可以放下
			{	nav.expect_robot_global_velt.fpY = -280;down_state_200 = 2;}
			break;
		
		case 2://前腿放下（接触到地面后给重力前馈）	 	
			cmd_1.Target_Delta_Deg = 2.9;
			cmd_4.Target_Delta_Deg = -2.9;
		if(fabs(cmd_1.target_td - 2.9f)<=0.05f||fabs(cmd_4.target_td + 2.9f)<=0.05f)
		{
			cmd_1.T = 0.64;
			cmd_4.T = -0.64;			
			nav.expect_robot_global_velt.fpY = -1200;
		}		
		if(travel_switch_mode[4]==0){	nav.expect_robot_global_velt.fpY = -620;down_state_200 = 3;}
			break;
		
		case 3:
		if(travel_switch_mode[3]==0)//行程开关无遮挡，后腿可以落下
		{	timer_wait_key_200++;if(timer_wait_key_200>=30){down_state_200 = 4;}}		
			break;
		
		case 4://后腿落下
			cmd_2.Target_Delta_Deg = -2.9;
			cmd_3.Target_Delta_Deg = 2.9;	
		if(fabs(cmd_3.target_td - 2.9f)<=0.05f||fabs(cmd_2.target_td + 2.9f)<=0.05f)//反馈判断接近地面		
		{
			cmd_2.T = -0.75;
			cmd_3.T = 0.75;		
			down_state_200 = 5;
		}			
			break;

		case 5://收起辅助轮，调整PID准备降落
		timer_wait_200++;
		if(timer_wait_200>=20){AirOperaterCtrlBuf[5] = 0;}
		if(timer_wait_200>=60) //降落
		{
			GO1_state_change = 2;
			nav.expect_robot_global_velt.fpY = 0;
			GO1_fold();
		}
		if(fabs(cmd_1.target_td+0.35f)<=0.1f||fabs(cmd_2.target_td-0.35f)<=0.1f||fabs(cmd_3.target_td+0.35f)<=0.1||fabs(cmd_4.target_td-0.35f)<=0.1f)
		{down_state_200 = 6;}
			break;
		
		case 6:
			dji_run.flag_2006_V = 0;
			AirOperaterCtrlBuf[5] = 0;
			GO1_state_change = 1;
			nav.expect_robot_global_velt.fpY = 0;
			timer_wait_200 = 0;
			timer_wait_key_200 = 0;
			GO1_fold();
			down_state_200 = 0;				
			break;
		
		default:
			break;
	}
}




void down_logic_400(void)
{
	static uint8_t timer_wait_400 = 0;
	static uint8_t timer_wait_key_400 = 0;
	
	switch(down_state_400)
	{
	
		case 0://初始
			break;
		
		case 1://底盘前进（快速）  辅助轮弹出		后腿放下让两个2006轮子接触地面，并转动
			dji_run.flag_2006_V = 1;
			GO1_state_change = 4;			
			cmd_2.Target_Delta_Deg = 0;
			cmd_3.Target_Delta_Deg = 0;		
			nav.expect_robot_global_velt.fpY = -500;
			AirOperaterCtrlBuf[5] = 1;
			if(travel_switch_mode[1]==0) //行程开关无遮挡，代表前腿可以放下
			{nav.expect_robot_global_velt.fpY = -210;	down_state_400 = 2;}
			break;
		
		case 2://前腿放下（接触到地面后给重力前馈）	 	
			cmd_1.Target_Delta_Deg = 5.7;
			cmd_4.Target_Delta_Deg = -5.7;
		if(fabs(cmd_1.target_td - 5.7f)<=0.05f||fabs(cmd_4.target_td + 5.7f)<=0.05f)
		{
			cmd_1.T = 0.64;
			cmd_4.T = -0.64;		
			nav.expect_robot_global_velt.fpY = -900;
		}		
		if(travel_switch_mode[4]==0){nav.expect_robot_global_velt.fpY = -400;down_state_400 = 3;}
			
			break;
		
		case 3:
		if(travel_switch_mode[3]==0)//行程开关无遮挡，后腿可以落下
		{nav.expect_robot_global_velt.fpY = -330;timer_wait_key_400++;if(timer_wait_key_400>=50){down_state_400 = 4;}}		
			break;
		
		case 4://后腿落下
			cmd_2.Target_Delta_Deg = -5.7;
			cmd_3.Target_Delta_Deg = 5.7;	
		if(fabs(cmd_3.target_td - 5.7f)<=0.05f||fabs(cmd_2.target_td + 5.7f)<=0.05f)//反馈判断接近地面		
		{
			cmd_2.T = -0.75;
			cmd_3.T = 0.75;		
			down_state_400 = 5;
		}			
			break;

		case 5://收起辅助轮，调整PID准备降落
		timer_wait_400++;
		if(timer_wait_400>=50){AirOperaterCtrlBuf[5] = 0;}
		if(timer_wait_400>=100) //降落
		{
			GO1_state_change = 2;
			nav.expect_robot_global_velt.fpY = 0;
			GO1_fold();
		}
		if(fabs(cmd_1.target_td+0.35f)<=0.1f||fabs(cmd_2.target_td-0.35f)<=0.1f||fabs(cmd_3.target_td+0.35f)<=0.1f||fabs(cmd_4.target_td-0.35f)<=0.1f)
		{down_state_400 = 6;}
			break;
		
		case 6:
			dji_run.flag_2006_V = 0;
			AirOperaterCtrlBuf[5] = 0;
			GO1_state_change = 1;
			nav.expect_robot_global_velt.fpY = 0;
			timer_wait_400 = 0;
			timer_wait_key_400 = 0;
			GO1_fold();
			down_state_400 = 0;					
			break;
		
		default:
			break;
	}

}


void combine(void)
{
	static uint8_t flag_first = 0;
	static uint16_t Timer_delay = 0;
	switch(combine_state)
	{
		case 0://初始化
			break;
		
		case 1://R2站立
			if(flag_first==0){lift_state = 1;flag_first = 1; }
			test_lift(6.5);	
			if(lift_state==0){combine_state = 2;}
				
			break;
	
		case 2://等待视觉信号(延迟几百ms收腿)
			if(vision_data_recieve.aruco_detect_flag==5)
			{combine_state = 3;}
		
			break;
		
		case 3://检测到登上R1后收腿，但是不会全部收回
			GO1_state_change = 3;
			Timer_delay++;
		if(Timer_delay>=1200)
		{		cmd_1.T = 0;
				cmd_2.T = 0;		
				cmd_3.T = 0;		
				cmd_4.T = 0;
				cmd_1.Target_Delta_Deg = 0.5;
				cmd_2.Target_Delta_Deg = -0.5;
				cmd_3.Target_Delta_Deg = 0.5;
				cmd_4.Target_Delta_Deg = -0.5;		
		if(fabs(cmd_1.target_td - cmd_1.Target_Delta_Deg)<=0.1||fabs(cmd_2.target_td - cmd_2.Target_Delta_Deg)<=0.1||fabs(cmd_3.target_td - cmd_3.Target_Delta_Deg)<=0.1||fabs(cmd_4.target_td - cmd_4.Target_Delta_Deg)<=0.1)
		{combine_state = 4;}
		}
			break;
		
		case 4:
			Timer_delay = 0;
			flag_first = 0;
		  combine_state = 0;
			up_down_state = 0;
			break;
	}
}

void up_down_logic(void)
{
//上电的时候四个升降轮着地，需要先收起
	static uint8_t flag_begin_lift = 0;
	static uint8_t flag_first = 0;
	
	switch (up_down_state)
	{
		case 0: //无上下台阶动作
			flag_first = 0;
		if(remote_up_down==1){up_down_state = 1;}
		else if(remote_up_down==2){up_down_state = 2;}
		else if(remote_up_down==3){up_down_state = 3;}
		else if(remote_up_down==4){up_down_state = 4;}
		else if(remote_up_down==5){up_down_state = 6;}
		
			break;
		
		case 1: //上200
			if(flag_first==0)
			{up_state_200 = 1;flag_first = 1;}		
			up_logic_200();	
			if(up_state_200 == 0)
    {
			 remote_up_down = 0;
       flag_first = 0;
       up_down_state = 0;
    }
			break;

		case 2://上400
			if(flag_first==0)
			{up_state_400 = 1;flag_first = 1;}
			up_logic_400();			
		if(up_state_400 == 0)
    {
			 remote_up_down = 0;
       flag_first = 0;
       up_down_state = 0;
    }
			break;

		case 3://下200
		if(flag_first==0)
			{down_state_200 = 1;flag_first = 1;}		
			down_logic_200();		
		if(down_state_200 == 0)
    {
			 remote_up_down = 0;
       flag_first = 0;
       up_down_state = 0;
    }			
			break;

		case 4://下400
		if(flag_first==0)
			{down_state_400 = 1;flag_first = 1;}		
			down_logic_400();		
		if(down_state_400 == 0)
    {
			 remote_up_down = 0;
       flag_first = 0;
       up_down_state = 0;
    }						
			break;		
		
		case 5://四条腿初始化
	if(flag_begin_lift == 0){
		GO1_td_1.r = 1500;
		GO1_td_2.r = 1500;
		GO1_td_3.r = 1500;
		GO1_td_4.r = 1500;
		cmd_1.Target_Delta_Deg = -0.35;
		cmd_2.Target_Delta_Deg = 0.35;
		cmd_3.Target_Delta_Deg = -0.35;
		cmd_4.Target_Delta_Deg = 0.35;		
		flag_begin_lift = 1;
	}
	if(flag_begin_lift){GO1_state_change = 1;up_down_state = 0;}
			break;
	
		case 6://R1，R2合体状态
		if(flag_first==0)
			{combine_state = 1;flag_first = 1;}		
			combine();		
		if(combine_state == 0)
    {
			 remote_up_down = 0;
       flag_first = 0;
       up_down_state = 0;
    }									
			break;
		
		case 7://收腿重试
			
		GO1_state_change = 3;
		cmd_1.Target_Delta_Deg = -0.35;
		cmd_2.Target_Delta_Deg = 0.35;
		cmd_3.Target_Delta_Deg = -0.35;
		cmd_4.Target_Delta_Deg = 0.35;	
			break;
		
		case 8://放下腿重试
		GO1_state_change = 3;
		cmd_1.Target_Delta_Deg = 6.5;
		cmd_2.Target_Delta_Deg = -6.5;
		cmd_3.Target_Delta_Deg = 6.5;
		cmd_4.Target_Delta_Deg = -6.5;				
			break;
		
	
		default:
			break;
	}
}


//使用在周期为0.25ms的计时器中断里，令每个GO1电机控制频率为1000
void GO_ctrl_logic(void)
{
	static uint8_t motor_index = 0;
	switch(motor_index)
	{
		case 0:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);		
			motor_go1_cmd(&cmd_1,&GO1_td_1);
			break;

		case 1:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);		
			motor_go1_cmd(&cmd_2,&GO1_td_2);			
			break;

		case 2:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);		
			motor_go1_cmd(&cmd_3,&GO1_td_3);
			break;

		case 3:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);		
			motor_go1_cmd(&cmd_4,&GO1_td_4);			
			break;		
		
		default:
			break;
	}	
	
	motor_index++;
	if(motor_index>=4){motor_index = 0;}
}

//用于四条腿站起
void test_lift(float lift_rad)
{
	switch(lift_state)
	{
		case 0:
			break;
		
		case 1:
		GO1_state_change = 5;
		GO1_td_1.r = 550;
		GO1_td_2.r = 550;
		GO1_td_3.r = 550;
		GO1_td_4.r = 550;
		
		cmd_1.T =0;
		cmd_2.T =0;	
		cmd_3.T =0;
		cmd_4.T =0;		
		
		cmd_1.Target_Delta_Deg =0;
		cmd_2.Target_Delta_Deg =0;	
		cmd_3.Target_Delta_Deg =0;
		cmd_4.Target_Delta_Deg =0;
		if(fabs(cmd_4.target_td)<=0.05||fabs(cmd_3.target_td)<=0.05||fabs(cmd_2.target_td)<=0.05||fabs(cmd_1.target_td)<=0.05)
		{lift_state = 2;}
			break;
		
		case 2:
		GO1_state_change = 5;
		GO1_td_1.r = 620;
		GO1_td_2.r = 620;
		GO1_td_3.r = 620;
		GO1_td_4.r = 620;
		cmd_1.T =  0.64;
		cmd_2.T = -0.75;
		cmd_3.T =  0.75;
		cmd_4.T = -0.64;	
		cmd_1.Target_Delta_Deg =lift_rad;
		cmd_2.Target_Delta_Deg =-lift_rad;	
		cmd_3.Target_Delta_Deg =lift_rad;
		cmd_4.Target_Delta_Deg =-lift_rad;		
		if((fabs(cmd_4.target_td - cmd_4.Target_Delta_Deg)<=0.05||fabs(cmd_3.target_td- cmd_3.Target_Delta_Deg)<=0.05||fabs(cmd_2.target_td- cmd_2.Target_Delta_Deg)<=0.05||fabs(cmd_1.target_td- cmd_1.Target_Delta_Deg)<=0.05))
		{lift_state = 0;}
			break;
		
	}
}

//用于前两条腿垫高
void test_lift_two(float lift_rad)
{
	switch(lift_state_two)
	{
		case 0:
			break;
		
		case 1:
		GO1_state_change = 5;
		GO1_td_1.r = 1200;
		GO1_td_4.r = 1200;
		
		cmd_1.T =0;
		cmd_4.T =0;		
		
		cmd_1.Target_Delta_Deg =0;
		cmd_4.Target_Delta_Deg =0;
		
		if(fabs(cmd_4.target_td)<=0.05||fabs(cmd_1.target_td)<=0.05)
		{lift_state_two = 2;}
			break;
		
		case 2:
		GO1_state_change = 5;
		GO1_td_1.r = 3000;
		GO1_td_4.r = 3000;
		cmd_1.T =  0.64;
		cmd_4.T = -0.64;	
		cmd_1.Target_Delta_Deg =lift_rad;
		cmd_4.Target_Delta_Deg =-lift_rad;		
		if((fabs(cmd_4.target_td - cmd_4.Target_Delta_Deg)<=0.||fabs(cmd_1.target_td- cmd_1.Target_Delta_Deg)<=0.1))
		{lift_state_two = 0;}
			break;
		
	}
}


void GO1_fold(void)
{
		cmd_1.Target_Delta_Deg = -0.35;
		cmd_2.Target_Delta_Deg = 0.35;
		cmd_3.Target_Delta_Deg = -0.35;
		cmd_4.Target_Delta_Deg = 0.35;		
}




//测试函数
void test_height(void)
{
	GO1_state_change = 2;
	cmd_1.Target_Delta_Deg =test_rad;
	cmd_2.Target_Delta_Deg =-test_rad;
	cmd_3.Target_Delta_Deg =test_rad;	
	cmd_4.Target_Delta_Deg =-test_rad;	
	
}





