#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "stm32f4xx.h"

//USER'S PARAM
#define MY_J60_MAX_POS 40
#define MY_J60_MIN_POS -40

/** -----------------------------
================================
      ######宏定义 #####
================================
 ---------------------------- **/
 
//*******************constant*****************************//
typedef CAN_HandleTypeDef hcan_t;

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
#define LK_uiGearRatio     10        /**<LKi10 5010编码器线数  */
#define LK_siNumber        65536
//LK
#define DEVICE_STD_ID						(0x140)
#define DEVICE_STD_BOARDCAST_ID	(0x280)


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
/*
 * 定义一个名为MotorCMD的结构体，用于存储电机控制命令和参数
 */
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

/*LK Motor*/

/****************************************************************/

/**
 * @brief 定义一个名为Motor_LK_8016的结构体，用于描述LK-8016电机的相关参数和控制变量
 */
typedef struct
{
  ST_ENCODER LK_motor_encoder;        // 编码器结构体，用于存储LK电机的编码器信息
	ST_CascadePID lk_pid;           // 级联PID控制器结构体，用于LK电机的闭环控制
  uint32_t LK_ID;                 // LK电机的ID标识符
	int16_t temp;                                             // 电机温度值，单位可能是摄氏度
	int16_t current;                              // 电机电流值，单位可能是安培
	int16_t speed;                                  // 电机速度值，单位可能是RPM
	uint16_t encoder_num;                    // 编码器计数值
  float angle;				          			          // 电机角度值，单位可角度
	float anglev;						        // 电机角速度值，单位是角度/秒

  float Send_Current;             // 发送给电机的电流值
  float outerTarget;				    			    // 外环控制目标值
  float outerFeedback;			    		    // 外环反馈值
  float innerFeedback;			    		    // 内环反馈值
}Motor_LK_8016;
	
/****************************************************************/

/*DM*/

/****************************************************************/
// 
typedef struct
{
    int id; //  电机ID标识，用于区分不同的电机
    int state; //  电机状态，表示电机的当前工作状态
    int p_int; //  位置积分值，用于位置控制中的积分项
    int v_int; //  速度积分值，用于速度控制中的积分项
    int t_int; //  转矩积分值，用于转矩控制中的积分项
    int kp_int; //  位置控制的比例系数整数部分
    int kd_int; //  位置控制的微分系数整数部分
    float pos; //  电机位置值，弧度
    float vel; //  电机速度值，弧度/s
    float tor; //  电机转矩值，表示电机的当前转矩
    float Kp;
    float Kd;
    float Tmos; //  电机温度值，表示电机的当前温度
    float Tcoil; //  电机线圈温度值，表示电机线圈的当前温度
} motor_fbpara_t; //  电机反馈参数结构体类型定义，用于存储电机的各项反馈参数

/**
 * @brief 电机控制结构体定义
 * 
 * 该结构体用于存储电机控制相关的参数和设定值
 */
typedef struct
{
    uint8_t mode;    /**< 电机控制模式 */
    float pos_set;   /**< 位置设定值 */
    float vel_set;   /**< 速度设定值 */
    float tor_set;   /**< 转矩设定值 */
	  float cur_set;   /**< 电流设定值 */
    float kp_set;    /**< 位置环比例系数 */
    float kd_set;    /**< 速度环微分系数 */
} motor_ctrl_t;     /**< 电机控制结构体类型别名 */

/* 电机数据结构体定义 */
typedef struct
{
    uint16_t id;        /* 电机ID标识 */
	uint16_t mst_id;    /* 主控ID标识 */
    motor_fbpara_t para;    /* 电机反馈参数结构体 */
    motor_ctrl_t ctrl;      /* 电机控制参数结构体 */
} Motor_DM;            /* 电机数据管理结构体 */

/*DT35*/
typedef struct
{
	uint16_t dt35[4]; //  储存DT35反馈
	uint8_t airOperatorTxBuf[1];  //  储存气动板控制数据
}AIR_OPERATOR;


/**************Two Aris Wrist***********/

typedef struct{
	fp32 rollCtrl;      //表示横滚控制量，单位为度
	fp32 rollRec;       //表示横滚反馈量，单位为度
	fp32 pitchCtrl;     //表示俯仰控制量，单位为度
	fp32 pitchRec;      //表示俯仰反馈量，单位为度
	ST_DJI_MOTOR* motor1; //指向第一个电机对象的指针
	ST_DJI_MOTOR* motor2; //指向第二个电机对象的指针
}lilWrist;            //锥齿轮联合控制结构体

/****************** system monitor ********************/
typedef struct{
	uint32_t leftShoulder;
	uint32_t rightShoulder;
	uint32_t left_2006_1;
	uint32_t left_2006_2;
	uint32_t right_2006_1;
	uint32_t right_2006_2;
	uint32_t stretch_2006;
	uint32_t stretch_DM;
	uint32_t stretch_LK;
	uint32_t withinBoardCommunicate;
	uint32_t AO;
	
	uint32_t WBCT;
	uint32_t SCMT;
	uint32_t CT;
	uint32_t MSMT;
}SINGLE_MONITOR;
typedef struct{
	SINGLE_MONITOR cntMonitor;
	SINGLE_MONITOR fpsMonitor;
	SINGLE_MONITOR errorMonitor;
}NEW_SYSTEM_MONITOR;

typedef struct{
	uint32_t leftShoulder_cnt;
	uint32_t rightShoulder_cnt;
	uint32_t left_2006_1_cnt;
	uint32_t left_2006_2_cnt;
	uint32_t right_2006_1_cnt;
	uint32_t right_2006_2_cnt;
	uint32_t stretch_2006_cnt;
	uint32_t stretch_DM_cnt;
	uint32_t stretch_LK_cnt;
	uint32_t withinBoardCommunicate_cnt;
	uint32_t AO_cnt;
	
	uint32_t WBCT_cnt;
	uint32_t SCMT_cnt;
	uint32_t CT_cnt;
	uint32_t MSMT_cnt;
	
	uint32_t leftShoulder_fps;
	uint32_t rightShoulder_fps;
	uint32_t left_2006_1_fps;
	uint32_t left_2006_2_fps;
	uint32_t right_2006_1_fps;
	uint32_t right_2006_2_fps;
	uint32_t stretch_2006_fps;
	uint32_t stretch_DM_fps;
	uint32_t stretch_LK_fps;
	uint32_t withinBoardCommunicate_fps;
	uint32_t AO_fps;
	
	uint32_t WBCT_fps;
	uint32_t SCMT_fps;
	uint32_t CT_fps;
	uint32_t MSMT_fps;
}SYSTEM_MONITOR;

/** -----------------------------
================================
   ##### Function #####
================================
 ---------------------------- **/

/**Common Func**/
void LpFilter(ST_LPF *lpf);
int32_t my_intabs(int32_t num);
fp32 my_fp32abs(fp32 num);
fp32 ConvertAngle(fp32 fpAngA);
fp32 ClipFloat(fp32 fpValue, fp32 fpMin, fp32 fpMax);
fp32 rampSignalFP(fp32 start,fp32 end,uint32_t time,uint32_t whole_time);
fp32 curveSignalFP(fp32 start,fp32 end,uint32_t time,uint32_t whole_time);
void pushAndPull(uint8_t flag1,uint8_t flag2);
uint8_t bin_array_to_u8(uint8_t *bits);
/** -----------------------------
================================
   ##### Global_Variables #####
================================
 ---------------------------- **/
extern uint8_t action_DA ,  action_SA,height_flag,new_height_flag;
extern uint8_t feedback_DA,feedback_SA;
//CAN Communicate
extern uint8_t CAN1_RxBuf[8],CAN1_TxBuf[8],CAN2_RxBuf[8],CAN2_TxBuf[8];
extern SYSTEM_MONITOR system_monitor;
//Within Board Communicate
extern uint8_t RxBufFromZGT[17],TxBufToZGT[17];
extern uint8_t Tx_Completed_Flag;
extern uint8_t leaveKFSFlag;
//gravity Compensation
extern uint8_t gravityCompensation_DA_state;
extern uint8_t gravityCompensation_SA_state;
//AirOperator Control
extern uint8_t AirOperaterCtrlBuf[6];
extern uint8_t AirOperaterCtrl[1];
//DJI variables
extern ST_DJI_MOTOR left_2006_1,left_2006_2,right_2006_1,right_2006_2,stretch_2006;
extern lilWrist wrist_L,wrist_R;
extern fp32 roll_L,pitch_L,roll_R,pitch_R;

//J60 variables
extern DEEP_MOTOR leftShoulder,rightShoulder;
extern MotorCMD leftShoulderCMD,rightShoulderCMD;
extern MotorDATA leftShoulderReceive,rightShoulderReceive;
extern fp32 gTorqueLeft,gTorqueRight;
//DM variable
extern Motor_DM stretch_DM;
extern fp32 gTorqueDM;
//LK variables
extern Motor_LK_8016 stretch_LK;
extern ST_TD LK1_TD;
extern int16_t /*can1_cur_LK,*/g_cur_LK;
#endif
