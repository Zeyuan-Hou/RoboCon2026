#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "stm32h7xx.h"

#define main_match 1
//#define adaptive_training 1

//USER'S PARAM
#define MY_J60_MAX_POS 40
#define MY_J60_MIN_POS -40

/** -----------------------------
================================
      ######宏定义 #####
================================
 ---------------------------- **/
 
//*******************constant*****************************//
typedef FDCAN_HandleTypeDef hfdcan_t;

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;
typedef unsigned char  		UCHAR8;                    /* defined for unsigned 8-bits integer variable 	    */
typedef signed   char  		SCHAR8;                    /* defined for signed 8-bits integer variable	 */
typedef unsigned short 		USHORT16;                  /* defined for unsigned 16-bits integer variable 	*/
typedef signed   short 		SSHORT16;                  /* defined for signed 16-bits integer variable 	 */
typedef unsigned int   		UINT32;                    /* defined for unsigned 32-bits integer variable 	*/
typedef int   				    SINT32;                    /* defined for signed 32-bits integer variable 	*/
typedef float          		FP32;                      /* single precision floating point variable (32bits) */
typedef double         		DB64;                      /* double precision floating point variable (64bits)  */
typedef FP32              fp32;

#define pi 3.14159265358f
#define PI 3.14159265368f
#define PI2 6.2831853072f

// 角度弧度转换常用值
#define RADIAN		0.0174532922f			//PI/180 
#define RADIAN_10 0.00174532922f // PI/1800
#define RADIAN_15 0.261799387f
#define RADIAN_45 0.785398163f
#define RADIAN_75 1.308996939f
#define RADIAN_100 0.000174532922f
#define RADIAN_105 1.832595715f
#define RADIAN_135 2.356194490f
#define RADIAN_165 2.879793266f

//*******************Private Define************************//
/**
 * 电机参数
 * -----------------------------------------------------
 */
#define UMAX_GM6020        27000     /**< GM6020最大输出*/
#define UMAX_M3508         14000     /**< M3508最大输出*/
#define UMAX_M2006         9000      /**< M2006最大输出*/
#define UMAX_MG6012 			 2048				/**< MG6012最大输出 **/
#define GM6020_uiGearRatio 1         /**< GM6020 减速比 */
#define M3508_uiGearRatio  19        /**< M3508减速比 */
#define M2006_uiGearRatio  36        /**< M2006减速比 */
#define M3508_siNumber     8192      /**< M3508编码器线数 */
#define M2006_siNumber 8192 				 /**< M2006编码器线数 */
#define j60_siNumber 16384					 /**< J60编码器线数 */


/** -----------------------------
================================
   ##### Structure #####
================================
 ---------------------------- **/
 /*通用结构体*/
/*PID*/
/**
 * @brief PID控制结构体
 */
typedef struct
{
    float fpDes;      // 目标值
    float fpFB;        // 反馈值

    float fpE;          // 当前误差
    float fpPreE;    // 上一次误差
    float fpSumE;    // 误差累加和

    float fpKp;       // 比例系数
    float fpKi;       // 积分系数
    float fpKd;       // 微分系数

    float fpU;        // PID输出值
    float fpUp;      // 比例分量
    float fpUi;      // 积分分量
    float fpUd;      // 微分分量
    float fpPreUd;    // 上一次微分分量

    float fpUMax;      // 输出最大值
    float fpUpMax;    // 比例分量最大值
		float fpSumEMax; // 误差累积量最大值
    float fpUdMax;    // 微分分量最大值

    float fpEMax;     // 误差最大值
    float fpEMin;     // 误差最小值
} ST_PID;

/**
 * @brief 串级PID控制结构体
 */
typedef struct
{
    ST_PID inner;                                                 // 内环PID
    ST_PID outer;                                                  // 外环PID
    float output;                               // 输出值
} ST_CascadePID;


 //First-order low-pass filter
/**
 * @brief 低通滤波器结构体
 * 用于存储低通滤波器相关的参数和状态
 */
typedef struct
{
    float preout;    /**< 前一次滤波输出值 */
    float out;          /**< 当前滤波输出值 */
    float in;            /**< 滤波器输入值 */
    float off_freq;/**< 截止频率 */
    float samp_tim;/**< 采样时间 */
} ST_LPF;          
 


/*TD*/
typedef struct
{
    float x1;  // target position calculated
    float x2;  // target velocity calculated
    float x;   // a variable used in TD calculation
    float r;   // TD param
    float h;   // TD param
    float T;   // TD param
    float aim; // target position
} ST_TD;

// 陷波滤波器结构体
typedef struct {
    float x1, x2;      // 输入历史值 (x[n-1], x[n-2])
    float y1, y2;      // 输出历史值 (y[n-1], y[n-2])
    float b0, b1, b2;  // 分子系数
    float a1, a2;      // 分母系数 (a0已归一化为1)
} NotchFilter;

/* 2. ZVD 输入整形器结构体 */
typedef struct {
    float A1, A2, A3; // 三个脉冲的幅值
    float Td;         // 脉冲时间间隔
    float buffer[3];  // 环形缓冲区，用于存储历史输入指令
    uint8_t index;    // 当前缓冲区索引
} ZVD_Shaper;


/* MOTOR */
//DJI Motors
/**
 * @brief 编码器结构体，用于存储编码器的相关数据
 * 
 * 该结构体包含了编码器的原始值、前一次原始值、差值、累加值、齿轮比、编号、速度以及状态等信息
 */
typedef struct {
    int siRawValue;                ///< 编码器的当前原始值
    int siPreRawValue;          ///< 编码器的前一次原始值
    int siDiff;                         ///< 编码器当前值与前一次值的差值
    int siSumValue;               ///< 编码器值的累加值
    float siGearRatio;           ///< 编码器的齿轮比
    int siNumber;                    ///< 编码器的编号
    float fpSpeed;                 ///< 通过编码器计算得到的速度
		u8 state;							              ///< 编码器的工作状态
} ST_ENCODER;

/**
 * @brief DJI电机结构体定义
 * 
 * 该结构体用于存储DJI电机的各项参数和状态信息，包括编码器、PID控制器、TD控制器等
 */
typedef struct
{
  ST_ENCODER motor_encoder;		// 电机编码器结构体，用于获取电机位置和速度信息
  ST_CascadePID motor_pid;	// 级联PID控制器结构体，用于电机速度和位置的闭环控制
  ST_TD motor_td;			// TD（跟踪微分器）结构体，用于信号处理和微分估计
	
  uint16_t id;              // 电机ID，用于标识和区分不同的电机
  float EncoderNum;					// 编码器计数值，记录电机转过的总脉冲数
  float encoder_speed;				// 编码器速度，电机的实时转速
  float angle;								// 电机角度，电机的当前角度位置
  float anglev;				// 角速度，电机的实时角速度
  float motor_current;				// 电机电流，电机的当前输出电流

  float outerTarget;				  		// 外环目标值，通常用于位置控制的目标位置
  float outerFeedback;			  		// 外环反馈值，实际位置反馈
  float innerTarget;				  		// 内环目标值，通常用于速度控制的目标速度
  float innerFeedback;			  		// 内环反馈值，通常用于速度控制的速度反馈
	
  uint8_t getStartPos;		// 获取起始位置的标志位，用于判断是否已获取电机初始位置
  float Start_Pos;			// 电机起始位置，记录电机的初始位置
  float Pos;				// 当前位置，电机的当前位置
  float MaxPos; 			// 最大位置限制，电机的运动范围上限
  float MinPos;    			// 最小位置限制，电机的运动范围下限
  s16 temp;														// 临时变量，用于存储中间计算结果或临时数据
}ST_DJI_MOTOR; //  DJI电机结构体，用于存储电机相关的所有参数和状态

/**J60_Structrue**/
//Motor
typedef struct /** * @brief DEEP_MOTOR 结构体定义，用于存储电机控制参数和状态信息 */
{
	uint8_t motor_id_; //  电机ID标识符，用于区分不同电机
	uint8_t cmd_; //  电机控制模式
	float position_; //  电机目标位置
	float velocity_; //  电机目标速度
	float torque_; //  电机目标扭矩
	float kp_; //  Kp
	float kd_; //  Kd
	float MaxPos;  //  电机最大位置限制
	float MinPos;  //  电机最小位置限制
	float temp_;									 //  电机温度监测值
} DEEP_MOTOR;

//Data received in main

typedef struct
{
	uint8_t motor_id_;    // 电机ID，8位无符号整数
    uint8_t cmd_;         // 控制命令，8位无符号整数
	float position_;      // 位置值，单位是弧度
	float velocity_;      // 速度值，单位是弧度/秒
	float torque_;        // 扭矩值，单位是N·m
	bool flag_;           // 标志位，布尔类型
	float temp_;          // 温度值，单精度浮点数
	uint16_t error_;      // 错误码，16位无符号整数
}MotorDATA;             // J60接收数据结构

//Data transmitted to the j60 motor
typedef struct
{
	uint8_t motor_id_;    // 电机ID标识符，用于区分不同的电机
	uint8_t cmd_;         // 电机控制命令，如启动、停止、复位等
	float position_;      // 目标位置值，单位是弧度
	float velocity_;      // 目标速度值，单位是弧度/秒
	float torque_;        // 目标扭矩值，单位是N·m
	float kp_;            // 位置环比例增益参数
	float kd_;            // 位置环微分增益参数
}MotorCMD;              // J60控制命令结构
	
/**
 * 温度标志枚举类型定义
 * 用于标识不同组件的温度状态标志
 */
	enum Temp_Flag{
    kDriverTempFlag=0,  // 驱动器温度标志，值为0
    kMotorTempFlag=1    // 电机温度标志，值为1
	};
	
	
/****************************************************************/

typedef struct
{
    uint16_t usJsKey;     // 独立+矩阵按键
    uint16_t usJsLeft_X;  // 左摇杆x方向
    uint16_t usJsLeft_Y;  // 左摇杆y方向
    uint16_t usJsRight_X; // 右摇杆x方向
    uint16_t usJsRight_Y; // 右摇杆y方向
} ST_JS_VALUE;

typedef struct{
	fp32 vel_x;
	fp32 vel_y;
	fp32 w;
}ST_VEL;

typedef struct{
	fp32 pos_x;
	fp32 pos_y;
	fp32 Q;
}ST_POS;
/*************************AIR_OPERATOR***************************/
typedef struct
{
//	uint8_t LimitSwitch[8];		// 行程开关状态
	uint8_t airOperatorTxBuf[10];  //  储存气动板控制数据
	//0~9->1~10;		0:0V  1:+24V    2:-24V
    uint8_t airOperatorTx[3];  
	uint16_t dt35_origin[4];
	fp32 dt35[4];
	float dt35_front;
  float dt35_back;
  float dt35_left;
  float dt35_right;
}AIR_OPERATOR;

void Dt35_DataReceive(AIR_OPERATOR *qd , uint8_t *data);
	
	/****************** system monitor ********************/
typedef struct{
	uint16_t Shoulder_J60;
	uint16_t Elbow_J60;
	uint16_t Gimbal_J60;
	uint16_t Wrist_3508;
	uint16_t Move_2006;
	uint16_t Friction_3508;
	u16 Claw_3508;
	u16 Platform_L2006;
	u16 Platform_R2006;
	uint16_t AirOperater;
	u16 visionCommunicate;
	u16 remoteControl;
	uint16_t withinBoardCommunicate;
	
	uint16_t StateMachineTask;
	u16 UpActionTask;
	uint16_t AlgorithmTask;
	uint16_t CAN_Trans1Task;
	uint16_t CAN_Trans2Task;
	uint16_t UsartCommunicateTask;
}SINGLE_MONITOR;
typedef struct{
	SINGLE_MONITOR cntMonitor;
	SINGLE_MONITOR fpsMonitor;
	SINGLE_MONITOR errorMonitor;
	bool error[19];
}SYSTEM_MONITOR;



#define 	 STAND_BY 0
#define 	 INIT     1
#define    GRAB_HALFWEAPON 2
#define    HOLD_HALFWEAPON 3
#define    HOLD_WEAPON 4
#define 	 HOLD_KFS 2
#define 	 IN 1
#define  	 OUT 2

#define 	 INIT_ACTION 0
#define    GET_WEAPON 1
#define    STORE_WEAPON 2
#define    COMBINE_WEAPON 3
#define    USE_WEAPON 4
#define    MOVE_WEAPON_OUT 5
#define    KEEP_QUIET 10
#define 	 GET_KFS 1
#define    STORE_KFS 2
#define    GET_STORED_KFS 3
#define    USE_KFS 4
#define    HANDOVER_KFS 5
#define		 GET_AND_STORE 7
#define 	 MOVEOUT 1
#define    MOVEIN 2
#define 	 DEINIT 6

//KFS master orientation
#define III_DEFAULT  0
#define III_MIRRORED 1

#define STORE_AND_GET_STORED_LEFT  1
#define STORE_AND_GET_STORED_RIGHT 0

#define USE_KFS_RIGHT_OF_R1 1
#define USE_KFS_LEFT_OF_R1  0

#define HANDOVER_KFS_AT_LEFT_OF_R2  1
#define HANDOVER_KFS_AT_RIGHT_OF_R2 0
//nav target
#define PUT_KFS_IN_FIRST_COLUMN 0x3003	//RIGHT
#define PUT_KFS_IN_SECOND_COLUMN 0x3002	//MIDDLE
#define PUT_KFS_IN_THIRD_COLUMN 0x3001	//LEFT

#define MIRRORED_PUT_KFS_IN_FIRST_COLUMN 0x3006		//LEFT
#define MIRRORED_PUT_KFS_IN_SECOND_COLUMN 0x3005	//MIDDLE
#define MIRRORED_PUT_KFS_IN_THIRD_COLUMN 0x3004		//RIGHT


#define COMBINE_WITH_SECOND_COLUMN_R2 0x3011	//R1 AT LEFT
#define COMBINE_WITH_FIRST_COLUMN_R2 0x3012 	//R1 AT MIDDLE

#define MIRRORED_COMBINE_WITH_THIRD_COLUMN_R2 0x3015		//R1 AT MIDDLE
#define MIRRORED_COMBINE_WITH_SECOND_COLUMN_R2 0x3016		//R1 AT RIGHT

#define PUSH_PLATFORM_FOR_SECOND_COLUMN_R2 0x3017	//R1 AT LEFT
#define PUSH_PLATFORM_FOR_FIRST_COLUMN_R2 0x3018	//R1 AT MIDDLE

#define MIRRORED_PUSH_PLATFORM_FOR_THIRD_COLUMN_R2 0x3021		//R1 AT MIDDLE
#define MIRRORED_PUSH_PLATFORM_FOR_SECOND_COLUMN_R2 0x3022	//R1 AT RIGHT



#define DEFAULT_PIN 0
#define PUSH_PIN 2
#define PULL_PIN 1

typedef struct{
    u8 now_KFS_ID;
    u8 pre_KFS_ID;
    u8 R1_KFS_cnt;
    u8 KFS_height;
    u8 KFS_store;
		u8 KFS_IDs[3];
		u8 KFS_GET_MODE[2];
		u8 move_in_flag;
}II_ROBOT;

extern u8 delay_timer_flag;
extern u32 delay_timer;
extern u8 CAN1_RxBuf[8],CAN1_TxBuf[8],CAN2_RxBuf[8],CAN2_TxBuf[8],CAN3_RxBuf[8],CAN3_TxBuf[8];
extern u8 uart4_rxbuf[20],uart4_txbuf[23],//板件通讯
			usart1_rxbuf[122],vision_rec[122],usart1_txbuf[3],//视觉
			usart2_rxbuf[13],usart2_txbuf[24],//遥控器
			IRModuleTxBuffer[9],IRModuleRxBuffer[9];
extern ST_JS_VALUE remoteRec;
extern ST_VEL remote_vel;
extern ST_POS robotPos,deltaPos;
extern II_ROBOT ii_robot;
extern u8 get_KFS_source;
extern u8 input_mode;

extern u8 part_flag,part_step,lil_step,refresh,part1_refresh,part2_refresh;


extern u32 timer;
extern u8 get_KFS_height,store_KFS_orientation,use_KFS_orientaton;//0:ground 1:low 2:high 3:top
extern u8 get_KFS_ID;//1~12
extern u8 use_KFS_ID;//0:left 1:straight 2:right

extern u8 weapon_compensation_mode,KFS_compensation_mode;
extern u8 weapon_player_action;
extern u8 KFS_master_action;
extern u8 platform_action;
extern u8 weapon_player_state;
extern u8 KFS_master_state;
extern u8 platform_state;

extern u8 chassis_action,nav_cplt,weapon_num,chassis_move,get_allowed,cmf;
extern fp32 nav_leaved_time;
extern u16 nav_target,pre_target;
extern fp32 chassis_vel_rec1,chassis_vel_rec2;
extern fp32 nav_progress;
extern u8 get_start;

extern u8 QD_show_1,//0:无意义 1~6：取头1~6 7：结束对接
    QD_show_2,//0:无意义  8：等待5s 9：等待10s
		QD_show_final_1,QD_show_final_2;//让视觉展示
		
extern u8 weapon_cplt,KFS_cplt,platform_cplt;
extern u8 complete_weapon,complete_KFS,complete_platform;
extern u8 move2006_mode,move2006_align;
extern u8 friction3508_mode;

extern fp32 shoulder_angle, elbow_angle, wrist_angle, gimbal_angle,weapon_wrist_angle,weapon_move_angle,weapon_push_angle;
extern fp32 shoulder_torque, elbow_torque, wrist_torque, gimbal_torque,weapon_wrist_torque;
extern u32 sm_cnt1, sm_cnt2, sm_cnt3;

//Motor Define
//DJI Motors
extern ST_DJI_MOTOR move2006,claw3508,friction3508,friction3508_sub,wrist3508,
			//CAN2
			platform_L2006,platform_R2006;

extern u8 claw_mode,claw_pin;
//J60 Motors
extern DEEP_MOTOR gimbalJ60,
						//CAN2
						shoulderJ60,elbowJ60;
extern MotorCMD gimbalJ60CMD,shoulderJ60CMD,elbowJ60CMD;
extern MotorDATA gimbalJ60Receive,shoulderJ60Receive,elbowJ60Receive;
extern fp32 gimbal_torque,shoulder_torque,elbow_torque;
extern  ST_TD gimbalTD;
extern NotchFilter gimbalFilter;
//AirOperator
extern AIR_OPERATOR airOperator;
extern u8 left_limit,right_limit;
//SystemMonitor						
extern SYSTEM_MONITOR systemMonitor;
extern u8 upFpsError,downFpsError;
#endif
