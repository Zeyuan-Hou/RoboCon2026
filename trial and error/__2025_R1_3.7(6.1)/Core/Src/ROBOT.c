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
fp32 fpPosXOffset = 0;//X方向纠偏量
fp32 fpPosYOffset = 0;//Y方向纠偏量
fp32 fpQOffset = 0;    // 角度Q纠偏量，勿动 调整机器人初始姿态或补偿陀螺仪的系统误差。

fp32 fpStartX = 0; // 5557.5f;//319 底盘半宽+导轮
fp32 fpStartY = 0; // 545.0f;//361

ST_SYSTEM_MONITOR system_monitor;
SYSTEM_STATE System_State;

ST_ROBOT stRobot;
//ST_Chassis_Run chassis_run =
//{
//	.leftup =
//{
//	.fpKp=50.f,
//	.fpKi=0.2f,
//	.fpKd=2.f,
//	
//	.fpUMax=12300.f,
//	.fpUpMax=8300.f,
//	.fpUdMax=2000.f,
//	.fpSumEMax=40000.f,
//	
//	.fpEMax=300.f,
//	.fpEMin=0.5f
//},
//	.rightup =
//{
//	.fpKp=50.f,
//	.fpKi=0.2f,
//	.fpKd=2.f,
//	
//	.fpUMax=12300.f,
//	.fpUpMax=8300.f,
//	.fpUdMax=2000.f,
//	.fpSumEMax=40000.f,
//	
//	.fpEMax=300.f,
//	.fpEMin=0.5f
//},
//	.leftdown =
//{
//	.fpKp=50.f,
//	.fpKi=0.2f,
//	.fpKd=2.f,
//	
//	.fpUMax=12300.f,
//	.fpUpMax=8300.f,
//	.fpUdMax=2000.f,
//	.fpSumEMax=40000.f,
//	
//	.fpEMax=300.f,
//	.fpEMin=0.5f
//},
//	.rightdown =
//{
//	.fpKp=50.f,
//	.fpKi=0.2f,
//	.fpKd=2.f,
//	
//	.fpUMax=12300.f,
//	.fpUpMax=8300.f,
//	.fpUdMax=2000.f,
//	.fpSumEMax=40000.f,
//	
//	.fpEMax=300.f,
//	.fpEMin=0.5f
//},
//	.pid_state = VELT_LOOP,
//	.feed_forward_state = WITHOUT_FORWARD
//};
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

uart1_tx_protocol_t uart1_eft=
{
	0x55,
	0x22,
	UART1_TX_DATA_LEN,
	{0},
	0x01,
	0xAA
};

uart1_rx_protocol_t uart1_efr=
{
	0x55,
	0x22,
	UART1_RX_DATA_LEN,
	{0},
	0x01,
	0xAA
};


ST_MOTOR leftup_motor = { .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0}};
ST_MOTOR leftdown_motor = { .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0}};
ST_MOTOR rightup_motor = { .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0}};
ST_MOTOR rightdown_motor = { .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0}};

u8 FLAG_NRF;

float target_v[4];//用于四个轮子单独调PID

u8 dt35_location;

uint8_t UDP_rx_data[BUFF_SIZE];
uint8_t UDP_tx_data[BUFF_SIZE];


DRIBBLE_STATE Dribble_State;

ST_MOTOR Drib_Motor_Left = { .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0}}; //3508
ST_MOTOR Drib_Motor_Right = { .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0}}; //3508

ST_PID Drib_Left_Pid =
{
	.fpKp=35.f,
	.fpKi=0.001f,
	.fpKd=2.f,
	
	.fpUMax=13000.f,
	.fpUpMax=10000.f,
	.fpUdMax=2000.f,
	.fpSumEMax=100000.f,
	
	.fpEMax=1000.f,
	.fpEMin=20.f
};
ST_PID Drib_Right_Pid =
{
	.fpKp=35.f,
	.fpKi=0.001f,
	.fpKd=2.f,
	
	.fpUMax=12000.f,
	.fpUpMax=10000.f,
	.fpUdMax=2000.f,
	.fpSumEMax=100000.f,
	
	.fpEMax=1000.f,
	.fpEMin=20.f
};

float Drib_Left_Targetv;
float Drib_Right_Targetv;


ST_MOTOR Lift_Motor_Left = { .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0}}; //3508
ST_MOTOR Lift_Motor_Right = { .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0}}; //3508
ST_CascadePID Lift_Left_Pid=
{
	.inner =//车自身的速度环
	{
		.fpKp=150.f,
		.fpKi=0.1f,
		.fpKd=2.0f,
		
		.fpUMax=12000.f,
		.fpUpMax=12000.f,
		.fpUdMax=12000.f,
		.fpSumEMax=30000.f,
		
		.fpEMax=300.f,
		.fpEMin=1.0f
	},
	.outer =//车自身的位置环
	{
		.fpKp=4.f,
		.fpKi=0.00066f,
		.fpKd=0.0f,
		
		.fpUMax=200.f,//没有速度前馈下，最大输出角速度为90
		.fpUpMax=200.f,
		.fpUdMax=20.f,
		.fpSumEMax=4000.f,
		
		.fpEMax=150.f,
		.fpEMin=0.1f
	}
};
ST_CascadePID Lift_Right_Pid=
{
	.inner =//车自身的速度环
	{
		.fpKp=150.f,
		.fpKi=0.1f,
		.fpKd=2.0f,
		
		.fpUMax=12000.f,
		.fpUpMax=12000.f,
		.fpUdMax=12000.f,
		.fpSumEMax=30000.f,
		
		.fpEMax=300.f,
		.fpEMin=1.0f
	},
	.outer =//车自身的位置环
	{
		.fpKp=4.f,
		.fpKi=0.00066f,
		.fpKd=0.0f,
		
		.fpUMax=200.f,//没有速度前馈下，最大输出角速度为90
		.fpUpMax=200.f,
		.fpUdMax=20.f,
		.fpSumEMax=4000.f,
		
		.fpEMax=150.f,
		.fpEMin=0.1f
	}
};

float Lift_target_pos;

ST_TD Lift_Left_td=
{
	.r=3000,	
	.h=0.001,	
	.T=0.001	
};
ST_TD Lift_Right_td=
{
	.r=3000,	
	.h=0.001,	
	.T=0.001	
};



ST_SMC Drib_Left_Smc=
{
	.fpUMax=10000,
	.b=0.7,
	.eps=200,
	.gain=80,
	.dead=100,
	.TD={0,0,0,400000,0.001,0.001,0}
};
ST_SMC Drib_Right_Smc=
{
	.fpUMax=10000,
	.b=0.7,
	.eps=200,
	.gain=80,
	.dead=100,
	.TD={0,0,0,400000,0.001,0.001,0}
};
//ST_SMC Drib_Left_Smc=
//{
//	.fpUMax=10000,
//	.b=4.49,
//	.eps=7000,
//	.gain=54,
//	.dead=15,
//	.TD={0,0,0,200000,0.001,0.001,0}
//};
//ST_SMC Drib_Right_Smc=
//{
//	.fpUMax=10000,
//	.b=4.49,
//	.eps=7000,
//	.gain=54,
//	.dead=15,
//	.TD={0,0,0,200000,0.001,0.001,0}
//};

DRIBBLE_CHALLENGE Dribble_Challenge;
DRIBBLE_CHALLENGE Drib_Chal_Record;
u8 Load_State;

DRIB_LOAD Drib_Load;

u8 Drib_flag;
u8 Load_flag;

REC_BALL_STATE Rec_Ball_State;
u8 Rec_Ball_flag;



u8 switch_on_1;
u8 switch_on_2;

ST_LPF gyro_fliter={0,0,0,50,0.001};

ST_GYRO Gyro_Data_Test;
int num_circle;//记录陀螺仪转的总圈数
fp32 fpSumPosQ;

SHOOT_CHALLENGE Shoot_Challenge;

TASK_STATE Reset_Task_State;
TASK_STATE Manual_Task_State;
TASK_STATE Vision_Task_State;
TASK_STATE ShootChal_Task_State;
TASK_STATE DribChal_Task_State;
TASK_CHOICE Task_Choice;

MANUAL_MODE Manual_Mode;

MotorCMD J60_Motor_Ctrl={
	.position_=0.0f,
	.velocity_=0.0f,
	.torque_=0.0f,
	.kp_=200.0f,
	.kd_=7.0f};
MotorDATA J60_Motor_Data;
double J60_angle;
J60_ENABLE_STATE J60_Enable_State;
J60_CTRL_MODE J60_Ctrl_Mode;
ST_TD J60_td=
{
	.r=600,	
	.h=0.001,	
	.T=0.001	
};
float J60_target_pos=10;
float J60_motor_temp;
float J60_board_temp;
int J60_start_protect;
int J60_protect_cnt;

/* SCS舵机串口通信*/
int8_t uart4_tx_buffer[50]={0};
uint8_t uart4_rx_buffer[8]={0};

/*舵机*/
Servo Servo_Set = 
{ .ID = {1,2},
	.stable_state=0,

	.Time={0,0}, //用时间控制舵机转角度
	.Ref_Speed  = {700,700}, 
											
	.Servo_Stab = {.Stab = {0,0},//初始状态不稳定
	.StabDomain = {30,10},
	.Stab_MaxTime =100}
};

SERVO_STATE Servo_State;

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

ST_TD posX_veltX=
{
	.r=5000,
	.h=0.001,
	.T=0.001
};

ST_TD posY_veltY=
{
	.r=5000,
	.h=0.001,
	.T=0.001
};

ST_TD posW_veltW=
{
	.r=5000,
	.h=0.001,
	.T=0.001
};
ST_GLOBAL_VELT_FILTER global_velt_filter=
{
	.global_vx = {0,0,0,50,0.001},
	.global_vy = {0,0,0,50,0.001},
	.global_w  = {0,0,0,50,0.001}
};
ST_WHEEL_ENCODER_VELT_FILTER wheel_encoder_velt_filter=
{
	.leftup_velt = {0,0,0,50,0.001},
	.rightup_velt = {0,0,0,50,0.001},
	.rightdown_velt  = {0,0,0,50,0.001},
	.leftdown_velt = {0,0,0,50,0.001}
};

ST_WHEEL2BODY_VELT Wheelvelt_To_Bodyvelt=
{
	.Vx = {0,0,0,50,0.001},
	.Vy = {0,0,0,50,0.001},
	.W  = {0,0,0,50,0.001}
};
//ST_BODY_ACCEL Accel_Data=
//{
//	.accel_x_filter = {0,0,0,80,0.001},
//	.accel_y_filter = {0,0,0,80,0.001}
//};   

KalmanFilter KF_Vx=
{
	.v = 0.f,
	.P = 1.0f,
	.Q = 0.01f,
	.R_ins = 0.1f,
	.R_whl_base = 0.05f,
	.slip_thres = 150.0f,
	.slip_scale = 10.f
};
KalmanFilter KF_Vy=
{
	.v = 0.f,
	.P = 1.0f,
	.Q = 0.01f,
	.R_ins = 0.1f,
	.R_whl_base = 0.05f,
	.slip_thres = 150.0f,
	.slip_scale = 10.f
};
KalmanFilter KF_W=
{
	.v = 0.f,
	.P = 1.0f,
	.Q = 0.01f,
	.R_ins = 0.1f,
	.R_whl_base = 0.05f,
	.slip_thres = 15.0f,
	.slip_scale = 10.f
};

ST_RC_CTRL RC_Ctrl;

ST_TD radar_x_td=
{
	.r=5000,
	.h=0.02,
	.T=0.02
};
ST_TD radar_y_td=
{
	.r=5000,
	.h=0.02,
	.T=0.02
};

ST_LPF radar_x_lpf={0,0,0,200,0.02};
ST_LPF radar_y_lpf={0,0,0,200,0.02};

ST_DT35_NEW DT35_NEW;

ST_UPBOARD_DATA Upboard_Data;//上板发回来的东西，传到遥控器上显示

ST_AUTO_AIM_DATA Auto_Aim_Data;//自瞄的公用数据，随动轮定位的自瞄和雷达的自瞄都存在这里

VOFA vofa=
{
	{0},
	{0x00, 0x00, 0x80, 0x7f}
};



