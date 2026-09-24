#include "Types.h"

/**-----------------------------
      ####变量####
-----------------------------**/

/*气动板*/
uint8_t AirCtrl[3];
uint8_t DT35_Data[8];//从气动板接收，通过串口一发送给下层板
uint16_t DT35_Digital_Data[4];
uint8_t AirOperaterCtrlBuf[12] = {0};//控制电磁阀，[0]-[5]对应电磁阀开关1-6，分别对应取块机械臂气泵，车边存块气泵，车内存块气泵，取块机械臂电磁阀,存杆电磁阀,取杆电磁阀
QD_CTRL QD_Ctrl;//气动控制结构体

/*帧率检测结构体*/
ST_SYSTEM_MONITOR System_Monitor;

/*串口通信接收数据*/
ST_JS_VALUE Communicate_Js_Value;//遥控器摇杆数据结构体变量
uint8_t Communicate_KEY;//通信任务的按键变量
uint8_t RxMsg_USART1[32];
uint16_t Receive_length; //串口接收字节数
/*CAN通信接收数据*/
uint8_t RxMsg_CAN1[8];
uint8_t RxMsg_CAN2[8];

/*电机输入*/
MOTORINPUT MotorInput;
/*大疆电机发送电流*/
int16_t Current1_4[4];
int16_t Current5_8[4];

/********************************************************************************************
存取块机械臂的各种变量：存取块状态机枚举、电机变量
*********************************************************************************************/

ARM_BACKSOLVING Arm_BackSolving=
{
    .L1=350.f,
    .L2=300.f,
    .x0=-43.5f,
    .y0=227.8f,
    .theta3_0=0.f
};//机械臂运动解算结构体
BLOCKARM_TASK_CHANGE BlockArm_Task_Change=
{
    .First_Init=1,
	.BlockPut_Num=1
};//存取块机械臂任务状态切换标志
BLOCKARM_TASK_STATE BlockArm_Task_State;//存取块机械臂任务状态机变量
BLOCKARM_TASK_STATE BlockArm_Task_StatePre;//存取块机械臂任务状态机变量


MotorCMD BlockArm_Joint2_J60CMD=
{
//	0
    .position_=0.0f,
	.velocity_=0.0f,
	.torque_=0.0f,
	.kp_=250.f,
	.kd_=12.f
}; //存取块机械臂第二个关节的J60电机,ID2
MotorDATA BlockArm_Joint2_J60Data= 
{
    .error_=0,.state = 0
}; //存取块机械臂第二个关节的J60电机,ID2



MotorCMD BlockArm_Joint1_J60CMD=
{
//	0
    .position_=0.0f,
	.velocity_=0.0f,
	.torque_=0.0f,
	.kp_=200.f,
	.kd_=10.f
}; //存取块机械臂第一个关节（靠近基座）的J60电机,ID1
MotorDATA BlockArm_Joint1_J60Data=
{
    .error_=0,.state = 0
}; //存取块机械臂第一个关节（靠近基座）的J60电机,ID1



MotorCMD BlockArm_Gimbal_J60CMD=
{
//	0
    .position_=0.0f,
	.velocity_=0.0f,
	.torque_=0.0f,
	.kp_=100.f,
	.kd_=10.f
}; //控制云台的J60电机,ID3
MotorDATA BlockArm_Gimbal_J60Data; //控制云台的J60电机,ID3


float YSC_Tor[3]; //J60MIT控制参数，调整后利用SetMotorCMD函数修改MotorCMD中的对应参数
float YSC_Pos[3]; //J60MIT控制参数，调整后利用SetMotorCMD函数修改MotorCMD中的对应参数
float YSC_Vel[3]; //J60MIT控制参数，调整后利用SetMotorCMD函数修改MotorCMD中的对应参数
J60_ENABLE_STATE J60_Enable_State[3];//J60电机的状态







ST_MOTOR BlockArm_Joint3_3508= 
{
    .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0,.state=1},
};//存取块机械臂第三个关节的3508电机,ID4

ST_CascadePID BlockArm_Joint3_3508_PID=
{
//	0
    .outer={.fpEMin=0.f,.fpSumEMax=20000,.fpUMax=16800,.fpUpMax=12000.f,.fpUdMax=4000.f,.fpEMax=10000,.fpKp=2.3f,.fpKi=0,.fpKd=0},
    .inner={.fpEMin=0.f,.fpSumEMax=3000,.fpUMax=12000,.fpUpMax=8000.f,.fpUdMax=4000.f,.fpEMax=100000,.fpKp=120,.fpKi=0.35,.fpKd=0}
};
/********************************************************************************************
存取杆机械臂的各种变量：存取块状态机枚举、电机变量
*********************************************************************************************/
float PoleArm_Joint1_3508_Init_Position=0.f; //存取杆机械臂第一个关节的3508电机初始化位置
float PoleArm_Joint1_3508_Fetch_Position=368.f; //存取杆机械臂第一个关节的3508电机取杆位置
float PoleArm_Joint1_3508_Place_Position=20.f; //存取杆机械臂第一个关节的3508电机放杆位置
float PoleArm_Joint1_3508_Connect_Position=20.f; //存取杆机械臂第一个关节的3508电机连接位置
float PoleArm_Joint1_3508_Store_Position=20.f; //存取杆机械臂第一个关节的3508电机存储位置
float PoleArm_Joint1_3508_Error_Position=60.f; //存取杆机械臂第一个关节的3508电机初始化位置

float PoleArm_Joint2_2006_Init_Position =0.f; //存取杆机械臂第二个关节的2006电机初始化位置
float PoleArm_Joint2_2006_Fetch_Position =-30.f; //存取杆机械臂第二个关节的2006电机取杆位置
float PoleArm_Joint2_2006_Place_Position =-410.f; //存取杆机械臂第二个关节的2006电机放杆位置
float PoleArm_Joint2_2006_Connect_Position =-410.f; //存取杆机械臂第二个关节的2006电机连接位置
float PoleArm_Joint2_2006_Store_Position =-410.f; //存取杆机械臂第二个关节的2006电机存储位置
float PoleArm_Joint2_2006_Error_Position =0.f; //存取杆机械臂第二个关节的2006电机初始化位置

float PoleArm_FrictionWheel_3508_Init_Position =0.f; //驱动摩擦轮的3508电机初始化速度
float PoleArm_FrictionWheel_3508_Fetch_Position =0.f; //驱动摩擦轮的3508电机取杆速度
float PoleArm_FrictionWheel_3508_Place_Position =0.f; //驱动摩擦轮的3508电机放杆速度
float PoleArm_FrictionWheel_3508_Connect_Position =540.f; //驱动摩擦轮的3508电机连接速度
float PoleArm_FrictionWheel_3508_Store_Position =-540.f; //驱动摩擦轮的3508电机存储速度
float PoleArm_FrictionWheel_3508_Manual_Position =-540.f; //驱动摩擦轮的3508电机手动上限速度


POLEARM_TASK_STATE PoleArm_Task_State = POLEARM_TASK_INIT;//存取杆机械臂任务状态机初始状态
POLEARM_TASK_STATE PoleArm_Task_State_Pre = POLEARM_TASK_INIT;//存取杆机械臂任务状态机上一个状态

POLEARM_TASK_TIMER PoleArm_Task_Timer={0};//存取杆机械臂任务计时结构体

POLEARM_TASK_CHANGE PoleArm_Task_Change;


 ST_TD PoleArm_Joint1_3508_TD={.h=0.1,.r=1500,.T=0.001};//存取杆机械臂第一个关节的3508电机的跟随目标数据结构体
 ST_TD PoleArm_Joint2_2006_TD={.h=0.1,.r=1500,.T=0.001};//存取杆机械臂第二个关节的2006电机的跟随目标数据结构体
 ST_TD PoleArm_FrictionWheel_3508_TD={.h=0.1,.r=12000,.T=0.001};//驱动摩擦轮的3508电机的跟随目标数据结构体


ST_MOTOR PoleArm_Joint1_3508= {.motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0,.state=1}};//存取杆机械臂第一个关节（靠近基座）的3508电机,ID2，关节减速比1：4
ST_MOTOR PoleArm_Joint2_2006= {.motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0,.state=1}};//存取杆机械臂第二个关节的2006电机,ID1，关节减速比1：2
ST_MOTOR PoleArm_FrictionWheel_3508= {.motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0,.state=1}};//驱动摩擦轮的3508电机,ID3
ST_MOTOR PoleArm_Adjustment_2006= {.motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0,.state=1}};//用于对接微调的电机,ID5

ST_CascadePID PoleArm_Joint1_3508_PID=
{
//    0,//初始化不使用PID时将所有结构体成员赋值为零
    .outer={.fpEMin=0.f,.fpSumEMax=1000,.fpUMax=16800,.fpUpMax=12000.f,.fpUdMax=4000.f,.fpEMax=10000,.fpKp=2,.fpKi=0,.fpKd=0},
    .inner={.fpEMin=0.f,.fpSumEMax=3000,.fpUMax=16800,.fpUpMax=12000.f,.fpUdMax=4000.f,.fpEMax=100000,.fpKp=140,.fpKi=0.25,.fpKd=0}
};
ST_CascadePID PoleArm_Joint2_2006_PID=
{   
//    0,//初始化不使用PID时将所有结构体成员赋值为零
    .outer={.fpEMin=0.f,.fpSumEMax=1000,.fpUMax=16800,.fpUpMax=12000.f,.fpUdMax=4000.f,.fpEMax=10000,.fpKp=2.38,.fpKi=0,.fpKd=0},
    .inner={.fpEMin=0.f,.fpSumEMax=100,.fpUMax=16800,.fpUpMax=12000.f,.fpUdMax=4000.f,.fpEMax=100000,.fpKp=150,.fpKi=18,.fpKd=0}
};
ST_CascadePID PoleArm_FrictionWheel_3508_PID=
{
//	  0,
    .outer={.fpEMin=0.f,.fpSumEMax=20000,.fpUMax=16800,.fpUpMax=12000.f,.fpUdMax=4000.f,.fpEMax=10000,.fpKp=0,.fpKi=0,.fpKd=0},
    .inner={.fpEMin=0.f,.fpSumEMax=6000,.fpUMax=8000,.fpUpMax=5000.f,.fpUdMax=4000.f,.fpEMax=100000,.fpKp=150,.fpKi=0.15f,.fpKd=0}
};
ST_CascadePID PoleArm_Adjustment_2006_PID=
{
//	 0,//初始化不使用PID时将所有结构体成员赋值为零
    .outer={.fpEMin=0.f,.fpSumEMax=20000,.fpUMax=16800,.fpUpMax=12000.f,.fpUdMax=4000.f,.fpEMax=10000,.fpKp=0,.fpKi=0,.fpKd=0},
    .inner={.fpEMin=0.f,.fpSumEMax=100,.fpUMax=16800,.fpUpMax=12000.f,.fpUdMax=4000.f,.fpEMax=100000,.fpKp=60,.fpKi=0,.fpKd=0}
};

ST_PID PoleArm_Adjustment_PID=
{
	.fpKd=0.f,.fpKi=0.3f,.fpKp=10.f,.fpEMax=100.f,.fpUdMax=40.f,.fpUpMax=30.f,.fpUMax=50.f,
    .fpSumEMax=20.f,
};

/*前馈变量*/
GRAVITYPARAM Gravity_PoleArm=
{
    .angle0[0]=162.3625f,
    .angle0[1]=90.f,
    .k_Joint1=1330,
    .k_Joint2=100,
    .k_Joint3=0
};
FRICTIONPARAM Friction_PoleArm_Joint1=
{
    .B=0,
    .Fc=0,
    .velocity_threshold=100,
};
FRICTIONPARAM Friction_PoleArm_Joint2=
{
    .B=0,
    .Fc=0,
    .velocity_threshold=100,
};
FRICTIONPARAM Friction_PoleArm_FrictionWheel=
{
    .B=0,
    .Fc=0,
    .velocity_threshold=100,
};

FRICTIONPARAM Friction_PoleArm_Adjustment=
{
    .B=50.f,
    .Fc=10.f,
    .velocity_threshold=1.f,
};

GRAVITYPARAM Gravity_BlockArm=
{
    .k_Joint1=3,
    .k_Joint2=1.05,
    .k_Joint3=250
};
FRICTIONPARAM Friction_BlockArm_Joint1=
{
    .B=0,
    .Fc=0,
    .velocity_threshold=100,
};
FRICTIONPARAM Friction_BlockArm_Joint2=
{
    .B=0,
    .Fc=0,
    .velocity_threshold=100,
};
FRICTIONPARAM Friction_BlockArm_Joint3=
{
    .B=0,
    .Fc=0,
    .velocity_threshold=100,
};
FRICTIONPARAM Friction_BlockArm_Gimbal=
{
    .B=0,
    .Fc=0,
    .velocity_threshold=100,
};
YawController_t Yaw_ff=
{
	.ff_gain_1=0.08f,
	.ff_gain_2=0.08f
};

float theta1_cand[2];
float d;
float test[6];

/*运动规划*/

Trajectory Traj_Gimbal=
{
    .Type = TRAJ_TYPE_JOINT,
    .Data.Angle = {0}
};

Trajectory Traj_Arm=
{
	.Type = TRAJ_TYPE_LINE,
	.Data.Line = {0}
};

Trajectory Traj_BlockArm_Joint3=
{
    .Type = TRAJ_TYPE_JOINT,
    .Data.Angle = {0}
};
float Traj_tLast;

/*对接微调*/
float QR_OFFSET_X=8;

P_VISION_DATA p_vision_data;//对接微调视觉数据结构体变量

uint8_t Traj_Flag;
float pos;
float vel;
float acc;

uint8_t MotorCtrl_Flag;
