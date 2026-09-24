#include "ROBOT.h"

/**-----------------------------
      ####变量####
-----------------------------**/


/*随动轮*/
float degreeA = 0;
float degreeB = 0;

/*dt35*/
//can_task.c回调函数里使用
uint16_t dt35_distance[5];

uint16_t dt35_x1, dt35_y1;
uint16_t dt35_x2, dt35_y2;

//locate.c里使用
fp32 q_now_fix_x, q_now_fix_y;
fp32 tem_y_dt35, tem_x_dt35;

fp32 q_fix_x1 = 0.0f, q_fix_x2 = 0.0f;
fp32 q_fix_y1 = 0.0f, q_fix_y2 = 0.0f;

bool flag_x_dt35, flag_y_dt35;
int flag_x = 0, flag_y = 0;

/*定位*/
fp32 fpPosXOffset = 0;//-351; // X方向纠偏量，勿动
fp32 fpPosYOffset = 0;//359; // Y方向纠偏量，勿动
fp32 fpQOffset = 0;    // 角度Q纠偏量，勿动 调整机器人初始姿态或补偿陀螺仪的系统误差。

fp32 fpStartX = 0; // 5557.5f;//319 底盘半宽+导轮
fp32 fpStartY = 0; // 545.0f;//361

ST_SYSTEM_MONITOR system_monitor;
ST_ROBOT stRobot;

ST_MOTOR leftup_motor;
ST_MOTOR leftdown_motor;
ST_MOTOR rightup_motor;
ST_MOTOR rightdown_motor;

ST_Chassis_Run chassis_run =
{
	.leftup =
{
	.fpKp=130.f,
	.fpKi=0.1f,
	.fpKd=2.f,
	
	.fpUMax=12300.f,
	.fpUpMax=10000.f,
	.fpUdMax=2000.f,
	.fpSumEMax=8000.f,
	
	.fpEMax=300.f,
	.fpEMin=0.5f
},
	.rightup =
{
	.fpKp=130.f,
	.fpKi=0.1f,
	.fpKd=2.f,
	
	.fpUMax=12300.f,
	.fpUpMax=10000.f,
	.fpUdMax=2000.f,
	.fpSumEMax=8000.f,
	
	.fpEMax=300.f,
	.fpEMin=0.5f
},
	.leftdown =
{
	.fpKp=130.f,
	.fpKi=0.1f,
	.fpKd=2.f,
	
	.fpUMax=12300.f,
	.fpUpMax=10000.f,
	.fpUdMax=2000.f,
	.fpSumEMax=8000.f,
	
	.fpEMax=300.f,
	.fpEMin=0.5f
},
	.rightdown =
{
	.fpKp=130.f,
	.fpKi=0.1f,
	.fpKd=2.f,
	
	.fpUMax=12300.f,
	.fpUpMax=10000.f,
	.fpUdMax=2000.f,
	.fpSumEMax=8000.f,
	
	.fpEMax=300.f,
	.fpEMin=0.5f
},
	.pid_state = VELT_LOOP,
	.feed_forward_state = WITHOUT_FORWARD
};
//ST_Chassis_Run chassis_run =
//{
//	.leftup =
//{
//	.fpKp=130.f,
//	.fpKi=0.8f,
//	.fpKd=2.f,
//	
//	.fpUMax=12300.f,
//	.fpUpMax=9000.f,
//	.fpUdMax=2000.f,
//	.fpSumEMax=3000.f,
//	
//	.fpEMax=300.f,
//	.fpEMin=0.5f
//},
//	.rightup =
//{
//	.fpKp=130.f,
//	.fpKi=0.8f,
//	.fpKd=2.f,
//	
//	.fpUMax=12300.f,
//	.fpUpMax=9000.f,
//	.fpUdMax=2000.f,
//	.fpSumEMax=3000.f,
//	
//	.fpEMax=300.f,
//	.fpEMin=0.5f
//},
//	.leftdown =
//{
//	.fpKp=130.f,
//	.fpKi=0.8f,
//	.fpKd=2.f,
//	
//	.fpUMax=12300.f,
//	.fpUpMax=9000.f,
//	.fpUdMax=2000.f,
//	.fpSumEMax=3000.f,
//	
//	.fpEMax=300.f,
//	.fpEMin=0.5f
//},
//	.rightdown =
//{
//	.fpKp=130.f,
//	.fpKi=0.8f,
//	.fpKd=2.f,
//	
//	.fpUMax=12300.f,
//	.fpUpMax=9000.f,
//	.fpUdMax=2000.f,
//	.fpSumEMax=3000.f,
//	
//	.fpEMax=300.f,
//	.fpEMin=0.5f
//},
//	.pid_state = VELT_LOOP,
//	.feed_forward_state = WITHOUT_FORWARD
//};

ST_Nav nav=
{
	.auto_path.pos_pid.x=
	{
		.fpKp=1.3f,
		.fpKi=0.003f,
		.fpKd=2.f,
		
		.fpUMax=2000.f,
		.fpUpMax=1000.f,
		.fpUdMax=500.f,
		.fpSumEMax=10000.f,
		
		.fpEMax=500.f,
		.fpEMin=10.0f
	},
	.auto_path.pos_pid.y=
	{
		.fpKp=1.3f,
		.fpKi=0.003f,
		.fpKd=2.f,
		
		.fpUMax=2000.f,
		.fpUpMax=1000.f,
		.fpUdMax=500.f,
		.fpSumEMax=10000.f,
		
		.fpEMax=500.f,
		.fpEMin=10.0f
	},
	.auto_path.pos_pid.w=
	{
		.fpKp=6.f,//12.f,
		.fpKi=0.001f,
		.fpKd=2.f,
		
		.fpUMax=90.f,//没有速度前馈下，最大输出角速度为90
		.fpUpMax=90.f,
		.fpUdMax=10.f,
		.fpSumEMax=2000.f,
		
		.fpEMax=90.f,
		.fpEMin=0.1f
	}
};

ST_FOLLOWER_WHEEL stFollowerWheel;

ST_DT35 dt35_save,dt35_now;

PATH_PERMUTATION Path_Permuta;

ST_JS_VALUE Js_Value=
{
	.usJsLeft_X=LEFT_JS_X_MID,
	.usJsLeft_Y=LEFT_JS_Y_MID,
	.usJsRight_X=RIGHT_JS_MID
};


Path_State path_state;
POINT point_end={0, 0, 0};

//NAV初始标志位
bool flag_lock=1;
bool flag_permutation_path=1;
u8 flag_global_manual = 1;

u8 FLAG_NRF;

float target_v[4];//用于四个轮子单独调PID

u8 dt35_location;

uint8_t UDP_rx_data[BUFF_SIZE];
uint8_t UDP_tx_data[BUFF_SIZE];

u8 switch_on_1;
u8 switch_on_2;

ST_GYRO Gyro_Data_Test;
int num_circle;//记录陀螺仪转的总圈数
fp32 fpSumPosQ;

TASK_STATE Reset_Task_State;
TASK_STATE Manual_Task_State;
TASK_STATE Vision_Task_State;
TASK_STATE ShootChal_Task_State;
TASK_STATE DribChal_Task_State;
TASK_CHOICE Task_Choice;

MANUAL_MODE Manual_Mode;

ST_VISION_DATA Vision_Data;

ST_PID Chassis_Aim_Yaw_Pid=
{
		.fpKp=4.f,//2.f,//5.f,//12.f,
		.fpKi=0.001f,
		.fpKd=2.f,
		
		.fpUMax=90.f,//没有速度前馈下，最大输出角速度为90
		.fpUpMax=90.f,
		.fpUdMax=10.f,
		.fpSumEMax=2000.f,
		
		.fpEMax=90.f,
		.fpEMin=0.1f
};

ST_PID Chassis_Global_Yaw_Pid=
{
	.fpKp=7.f,
	.fpKi=0.001f,
	.fpKd=2.f,
	
	.fpUMax=90.f,//没有速度前馈下，最大输出角速度为90
	.fpUpMax=90.f,
	.fpUdMax=10.f,
	.fpSumEMax=2000.f,
	
	.fpEMax=90.f,
	.fpEMin=0.1f
};
//ST_CascadePID Chassis_Global_Yaw_Pid=
//{
//	.inner =//车自身的速度环
//	{
//		.fpKp=0.4f,
//		.fpKi=0.008f,
//		.fpKd=0.0f,
//		
//		.fpUMax=90.f,
//		.fpUpMax=90.f,
//		.fpUdMax=5.f,
//		.fpSumEMax=8000.f,
//		
//		.fpEMax=90.f,
//		.fpEMin=0.1f
//	},
//	.outer =//车自身的位置环
//	{
//		.fpKp=1.f,
//		.fpKi=0.0005f,
//		.fpKd=0.2f,
//		
//		.fpUMax=90.f,//没有速度前馈下，最大输出角速度为90
//		.fpUpMax=90.f,
//		.fpUdMax=5.f,
//		.fpSumEMax=4000.f,
//		
//		.fpEMax=90.f,
//		.fpEMin=0.1f
//	}
//};

ST_LESO_1order leso_leftup=
{
	.Z2 = 0,
	.Z1 = 0,
	.Beta01 = 60.f,
	.Beta02 = 900.f,
	.b0 = 346.f,
	.h = 0.001f,
	.fpUMax = 13000.f
};
ST_LESO_1order leso_rightup=
{
	.Z2 = 0,
	.Z1 = 0,
	.Beta01 = 60.f,
	.Beta02 = 900.f,
	.b0 = 310.f,
	.h = 0.001f,
	.fpUMax = 13000.f
};
ST_LESO_1order leso_rightdown=
{
	.Z2 = 0,
	.Z1 = 0,
	.Beta01 = 60.f,
	.Beta02 = 900.f,
	.b0 = 390.f,
	.h = 0.001f,
	.fpUMax = 13000.f
};
ST_LESO_1order leso_leftdown=
{
	.Z2 = 0,
	.Z1 = 0,
	.Beta01 = 60.f,
	.Beta02 = 900.f,
	.b0 = 250.f,
	.h = 0.001f,
	.fpUMax = 13000.f
};
fp32 compensation_leftup;
fp32 compensation_rightup;
fp32 compensation_rightdown;
fp32 compensation_leftdown;

ST_Wheel_Encoder_Velt_Filter wheel_encoder_velt_filter=
{
	.leftup_velt = {0,0,0,50,0.001},
	.rightup_velt = {0,0,0,50,0.001},
	.rightdown_velt  = {0,0,0,50,0.001},
	.leftdown_velt = {0,0,0,50,0.001}
};

ST_RC_CTRL RC_Ctrl;

A1_STRUCTRUE Motor_A1=
{
	.Ctrl_Data=
	{
		.ID = 1,
		.mode = DISABLE_MODE,
//		.K_P = 0.5,
//		.K_W = 1.5
	}
};

ST_TD A1_TD=
{
	.r=300,	
	.h=0.001,	
	.T=0.001	
};
float A1_target_pos;
ST_Jump_Motor_Ctrl left_xc5000_motor=
{
	.Encoder_Filter = {0,0,0,250,0.001},
	.Pid_State = VELT_LOOP,
	.Inner_Pid = 
	{
		.fpKp=60.f,//130.f,
		.fpKi=0.2f,
		.fpKd=2.f,
		
		.fpUMax=15000.f,
		.fpUpMax=14000.f,
		.fpUdMax=2000.f,
		.fpSumEMax=8000.f,
		
		.fpEMax=300.f,
		.fpEMin=0.0f	
	},
	.Outer_Pid =
	{
		.fpKp=5.f,//12.f
		.fpKi=0.f,
		.fpKd=0.5f,
		
		.fpUMax=650.f,
		.fpUpMax=650.f,
		.fpUdMax=400.f,
		.fpSumEMax=1000.f,
		
		.fpEMax=400.f,
		.fpEMin=0.f
	},
	.Leso =
	{
		.Z2 = 0,
		.Z1 = 0,
		.Beta01 = 60.f,
		.Beta02 = 900.f,
		.b0 = 250.f,
		.h = 0.001f,
		.fpUMax = 19000.f//22000.f
	},
	.feed_forward_state = WITHOUT_FORWARD
};
ST_Jump_Motor_Ctrl right_xc5000_motor=
{
	.Encoder_Filter = {0,0,0,250,0.001},
	.Pid_State = VELT_LOOP,
	.Inner_Pid = 
	{
		.fpKp=60.f,//130.f,
		.fpKi=0.2f,
		.fpKd=2.f,
		
		.fpUMax=15000.f,
		.fpUpMax=14000.f,
		.fpUdMax=2000.f,
		.fpSumEMax=8000.f,
		
		.fpEMax=300.f,
		.fpEMin=0.0f	
	},
	.Outer_Pid =
	{
		.fpKp=5.f,//12.f
		.fpKi=0.f,
		.fpKd=0.5f,
		
		.fpUMax=650.f,
		.fpUpMax=650.f,
		.fpUdMax=400.f,
		.fpSumEMax=1000.f,
		
		.fpEMax=300.f,
		.fpEMin=0.f
	},
	.Leso =
	{
		.Z2 = 0,
		.Z1 = 0,
		.Beta01 = 60.f,
		.Beta02 = 900.f,
		.b0 = 250.f,
		.h = 0.001f,
		.fpUMax = 19000.f//22000.f
	},
	.feed_forward_state = WITHOUT_FORWARD
};
ST_Jump_Motor_Ctrl left_4219_motor=
{
	.Encoder_Filter = {0,0,0,250,0.001},
	.Pid_State = VELT_LOOP,
	.Inner_Pid = 
	{
		.fpKp=60.f,//130.f,
		.fpKi=0.2f,
		.fpKd=2.f,
		
		.fpUMax=15000.f,
		.fpUpMax=14000.f,
		.fpUdMax=2000.f,
		.fpSumEMax=8000.f,
		
		.fpEMax=300.f,
		.fpEMin=0.0f	
	},
	.Outer_Pid =
	{
		.fpKp=5.f,//12.f
		.fpKi=0.f,
		.fpKd=0.5f,
		
		.fpUMax=650.f,
		.fpUpMax=650.f,
		.fpUdMax=400.f,
		.fpSumEMax=1000.f,
		
		.fpEMax=300.f,
		.fpEMin=0.f
	},
	.Leso =
	{
		.Z2 = 0,
		.Z1 = 0,
		.Beta01 = 60.f,
		.Beta02 = 900.f,
		.b0 = 250.f,
		.h = 0.001f,
		.fpUMax = 19000.f//24000.f
	},
	.feed_forward_state = WITHOUT_FORWARD
};
ST_Jump_Motor_Ctrl right_4219_motor=
{
	.Encoder_Filter = {0,0,0,250,0.001},
	.Pid_State = VELT_LOOP,
	.Inner_Pid = 
	{
		.fpKp=60.f,//130.f,
		.fpKi=0.2f,
		.fpKd=2.f,
		
		.fpUMax=15000.f,
		.fpUpMax=14000.f,
		.fpUdMax=2000.f,
		.fpSumEMax=8000.f,
		
		.fpEMax=300.f,
		.fpEMin=0.0f	
	},
	.Outer_Pid =
	{
		.fpKp=5.f,//12.f
		.fpKi=0.f,
		.fpKd=0.5f,
		
		.fpUMax=650.f,
		.fpUpMax=650.f,
		.fpUdMax=400.f,
		.fpSumEMax=1000.f,
		
		.fpEMax=300.f,
		.fpEMin=0.f
	},
	.Leso =
	{
		.Z2 = 0,
		.Z1 = 0,
		.Beta01 = 60.f,
		.Beta02 = 900.f,
		.b0 = 250.f,
		.h = 0.001f,
		.fpUMax = 19000.f//24000.f
	},
	.feed_forward_state = WITHOUT_FORWARD
};

float jump_con_delta_pos;//四个跳跃电机的共同角度

DUNK_STATE Dunk_State;
SUCTION_STATE Suction_State;

u8 dunk_flag;
u8 pos_mode_flag;
u8 damp_mode_flag;

u8 uart6_test[200];


VOFA vofa=
{
	{0},
	{0x00, 0x00, 0x80, 0x7f}
};



