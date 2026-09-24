#include "motor_control.h"
/*********************************************************************************************
函数功能：四个跳跃的电机读取零点后进入位置控制模式，同时加入前馈
				 同时设置了反馈值用于判断是否进入位控模式
**********************************************************************************************/
int jump_motor_pos_mode(void)
{
	static u8 flag_exit;
	if(flag_exit == 0)
	{
		left_xc5000_motor.Motor_Data.flag_init = 0;//跳跃电机进行读取零点
		right_xc5000_motor.Motor_Data.flag_init = 0;
		left_4219_motor.Motor_Data.flag_init = 0;
		right_4219_motor.Motor_Data.flag_init = 0;
		
		flag_exit = 1;
	}
	if(left_xc5000_motor.Motor_Data.flag_init == 1
	 &&right_xc5000_motor.Motor_Data.flag_init == 1
	 &&left_4219_motor.Motor_Data.flag_init == 1
	 &&right_4219_motor.Motor_Data.flag_init == 1)//所有电机都读完零点之后进行双环控制
	{
		left_xc5000_motor.Pid_State = DOUBLE_LOOP;//变成双环
		left_xc5000_motor.feed_forward_state = WITH_FORWARD;//加入补偿电流

		right_xc5000_motor.Pid_State = DOUBLE_LOOP;
		right_xc5000_motor.feed_forward_state = WITH_FORWARD;

		left_4219_motor.Pid_State = DOUBLE_LOOP;
		left_4219_motor.feed_forward_state = WITH_FORWARD;

		right_4219_motor.Pid_State = DOUBLE_LOOP;
		right_4219_motor.feed_forward_state = WITH_FORWARD;
		
		flag_exit = 0;
		return 1;
	}
	else
	{
		return 0;
	}
}
/*********************************************************************************************
函数功能：给四个跳跃的电机位置环进行赋值
**********************************************************************************************/
void jump_motor_pos_ctrl(void)
{
	right_xc5000_motor.Target_Delta_Pos = jump_con_delta_pos;
	right_4219_motor.Target_Delta_Pos = -jump_con_delta_pos;
	left_xc5000_motor.Target_Delta_Pos = jump_con_delta_pos;
	left_4219_motor.Target_Delta_Pos = -jump_con_delta_pos;
}
/*********************************************************************************************
函数功能：四个跳跃的电机进入阻尼模式，用于缓冲掉落下来的冲击力
**********************************************************************************************/
void jump_motor_damp_mode(void)
{
	left_xc5000_motor.Pid_State = VELT_LOOP;//变成速度环
	left_xc5000_motor.target_speed = 0;//进入阻尼模式
	left_xc5000_motor.feed_forward_state = WITHOUT_FORWARD;//取消补偿电流

	right_xc5000_motor.Pid_State = VELT_LOOP;
	right_xc5000_motor.target_speed = 0;
	right_xc5000_motor.feed_forward_state = WITHOUT_FORWARD;

	left_4219_motor.Pid_State = VELT_LOOP;
	left_4219_motor.target_speed = 0;
	left_4219_motor.feed_forward_state = WITHOUT_FORWARD;

	right_4219_motor.Pid_State = VELT_LOOP;
	right_4219_motor.target_speed = 0;
	right_4219_motor.feed_forward_state = WITHOUT_FORWARD;
}
/*********************************************************************************************
函数功能：四个跳跃电机的控制中心
**********************************************************************************************/
void jump_motor_control(ST_Jump_Motor_Ctrl *motor_ctrl)
{
	if(motor_ctrl->Pid_State == OPEN_LOOP)
	{
		motor_ctrl->Motor_Data.motor_current = 0;
	}
	
	else if(motor_ctrl->Pid_State == VELT_LOOP)
	{
		PID_Calc(&motor_ctrl->Inner_Pid ,motor_ctrl->target_speed ,motor_ctrl->Inner_Pid.fpFB);
		
		if(motor_ctrl->feed_forward_state == WITHOUT_FORWARD)
			motor_ctrl->Motor_Data.motor_current = motor_ctrl->Inner_Pid.fpU;
		else
		{
			LESO_Order1(&motor_ctrl->Leso ,motor_ctrl->Motor_Data.anglev ,motor_ctrl->Inner_Pid.fpU);
			motor_ctrl->compensation = motor_ctrl->Leso.U - motor_ctrl->Leso.U0;
			
			motor_ctrl->Motor_Data.motor_current = motor_ctrl->Leso.U;
		}
	}
	
	else if(motor_ctrl->Pid_State == DOUBLE_LOOP)
	{
		PID_Calc(&motor_ctrl->Outer_Pid ,motor_ctrl->Target_Delta_Pos ,motor_ctrl->Outer_Pid.fpFB);
		
		PID_Calc(&motor_ctrl->Inner_Pid ,motor_ctrl->Outer_Pid.fpU ,motor_ctrl->Inner_Pid.fpFB);
		
		if(motor_ctrl->feed_forward_state == WITHOUT_FORWARD)
			motor_ctrl->Motor_Data.motor_current = motor_ctrl->Inner_Pid.fpU;
		else
		{
			LESO_Order1(&motor_ctrl->Leso ,motor_ctrl->Motor_Data.anglev ,motor_ctrl->Inner_Pid.fpU);
			motor_ctrl->compensation = motor_ctrl->Leso.U - motor_ctrl->Leso.U0;
			
			motor_ctrl->Motor_Data.motor_current = motor_ctrl->Leso.U;
		}
	}
	
}
/*********************************************************************************************
函数功能：机械臂电机A1的控制中心：
					0：DISABLE_MODE 
						失能模式，防烧
						同时将进入使能模式初始化的标志位清0
					1：ENABLE_MODE
						位控模式。加入了TD的平滑效果
				失能模式切换到使能模式会主动进行期望位置复位，防止电机乱动
**********************************************************************************************/
void arm_motor_control(void)
{
	static u8 flag_exit_ctrl_mode;
	if(Motor_A1.Ctrl_Data.mode == DISABLE_MODE)
	{
		flag_exit_ctrl_mode = 0;
	}
	else 
	{
		if(flag_exit_ctrl_mode == 0)
		{
			Motor_A1.Target_Delta_Deg = Motor_A1.Real_Delta_Deg;//J60目标位置记录为当前位置
			A1_TD.r = 300;
			A1_TD.x1 = Motor_A1.Real_Delta_Deg;
			A1_TD.x2 = 0;
			flag_exit_ctrl_mode = 1;
		}
		A1_TD.aim = ClipFloat(A1_target_pos,-190,0);
		CalTD(&A1_TD);
		Motor_A1.Target_Delta_Deg = A1_TD.x1;
//		Motor_A1.Ctrl_Data.T = ClipFloat(5.7*sin(Motor_A1.Real_Delta_Deg*RADIAN),-5.7,5.7);
	}
}

