#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "main.h"

// USER'S PARAM
#define MY_J60_MAX_POS 40
#define MY_J60_MIN_POS -40

/** -----------------------------
================================
      ######宏定义 #####
================================
 ---------------------------- **/

//*******************constant*****************************//
typedef FDCAN_HandleTypeDef hcan_t;

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;
typedef int32_t s32;
typedef int16_t s16;
typedef int8_t s8;
typedef unsigned char UCHAR8;    /* defined for unsigned 8-bits integer variable 	    */
typedef signed char SCHAR8;      /* defined for signed 8-bits integer variable	 */
typedef unsigned short USHORT16; /* defined for unsigned 16-bits integer variable 	*/
typedef signed short SSHORT16;   /* defined for signed 16-bits integer variable 	 */
typedef unsigned int UINT32;     /* defined for unsigned 32-bits integer variable 	*/
typedef int SINT32;              /* defined for signed 32-bits integer variable 	*/
typedef float FP32;              /* single precision floating point variable (32bits) */
typedef double DB64;             /* double precision floating point variable (64bits)  */
typedef FP32 fp32;

#define pi 3.14159265358f
#define PI 3.14159265368f
#define PI2 6.2831853072f

// 角度弧度转换常用值
#define RADIAN 0.0174532922f     // PI/180
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
#define UMAX_GM6020 27000    /**< GM6020最大输出*/
#define UMAX_M3508 14000     /**< M3508最大输出*/
#define UMAX_M2006 9000      /**< M2006最大输出*/
#define UMAX_MG6012 2048     /**< MG6012最大输出 **/
#define GM6020_uiGearRatio 1 /**< GM6020 减速比 */
#define M3508_uiGearRatio 19 /**< M3508减速比 */
#define M2006_uiGearRatio 36 /**< M2006减速比 */
#define M3508_siNumber 8192  /**< M3508编码器线数 */
#define M2006_siNumber 8192  /**< M2006编码器线数 */
#define j60_siNumber 16384   /**< J60编码器线数 */

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
    float fpDes; // 目标值
    float fpFB;  // 反馈值

    float fpE;    // 当前误差
    float fpPreE; // 上一次误差
    float fpSumE; // 误差累加和

    float fpKp; // 比例系数
    float fpKi; // 积分系数
    float fpKd; // 微分系数

    float fpU;     // PID输出值
    float fpUp;    // 比例分量
    float fpUi;    // 积分分量
    float fpUd;    // 微分分量
    float fpPreUd; // 上一次微分分量

    float fpUMax;    // 输出最大值
    float fpUpMax;   // 比例分量最大值
    float fpSumEMax; // 误差累积量最大值
    float fpUdMax;   // 微分分量最大值

    float fpEMax; // 误差最大值
    float fpEMin; // 误差最小值
} ST_PID;

/**
 * @brief 串级PID控制结构体
 */
typedef struct
{
    ST_PID inner; // 内环PID
    ST_PID outer; // 外环PID
    float output; // 输出值
} ST_CascadePID;

// First-order low-pass filter
/**
 * @brief 低通滤波器结构体
 * 用于存储低通滤波器相关的参数和状态
 */
typedef struct
{
    float preout;   /**< 前一次滤波输出值 */
    float out;      /**< 当前滤波输出值 */
    float in;       /**< 滤波器输入值 */
    float off_freq; /**< 截止频率 */
    float samp_tim; /**< 采样时间 */
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

typedef struct
{
    fp32 fpDes;  // 控制变量目标值
    fp32 fpFB;   // 控制变量反馈值
    fp32 fpE;    // 本次偏差
    fp32 fpU;    // 本次运算结果
    fp32 fpUMax; // 输出上限

    // SMC参数
    fp32 b;    // 转动惯量倒数
    fp32 eps;  // 饱和函数增益项，作用是扰动补偿
    fp32 gain; // 比例增益项
    fp32 dead; // 饱和函数死区，在死区内为放大环节，死区外为固定值
    ST_TD TD;  // 使用TD来获取微分信号
} ST_SMC;

/* MOTOR */
// DJI Motors
/**
 * @brief 编码器结构体，用于存储编码器的相关数据
 *
 * 该结构体包含了编码器的原始值、前一次原始值、差值、累加值、齿轮比、编号、速度以及状态等信息
 */
typedef struct
{
    int siRawValue;    ///< 编码器的当前原始值
    int siPreRawValue; ///< 编码器的前一次原始值
    int siDiff;        ///< 编码器当前值与前一次值的差值
    int siSumValue;    ///< 编码器值的累加值
    float siGearRatio; ///< 编码器的齿轮比
    int siNumber;      ///< 编码器的编号
    float fpSpeed;     ///< 通过编码器计算得到的速度
    u8 state;          ///< 编码器的工作状态
} ST_ENCODER;

/**
 * @brief DJI电机结构体定义
 *
 * 该结构体用于存储DJI电机的各项参数和状态信息，包括编码器、PID控制器、TD控制器等
 */
typedef struct
{
    ST_ENCODER motor_encoder; // 电机编码器结构体，用于获取电机位置和速度信息
    ST_CascadePID motor_pid;  // 级联PID控制器结构体，用于电机速度和位置的闭环控制
    ST_TD motor_td;           // TD（跟踪微分器）结构体，用于信号处理和微分估计

    uint16_t id;         // 电机ID，用于标识和区分不同的电机
    float EncoderNum;    // 编码器计数值，记录电机转过的总脉冲数
    float encoder_speed; // 编码器速度，电机的实时转速
    float angle;         // 电机角度，电机的当前角度位置
    float anglev;        // 角速度，电机的实时角速度
    float motor_current; // 电机电流，电机的当前输出电流

    float outerTarget;   // 外环目标值，通常用于位置控制的目标位置
    float outerFeedback; // 外环反馈值，实际位置反馈
    float innerTarget;   // 内环目标值，通常用于速度控制的目标速度
    float innerFeedback; // 内环反馈值，通常用于速度控制的速度反馈

    uint8_t getStartPos; // 获取起始位置的标志位，用于判断是否已获取电机初始位置
    float Start_Pos;     // 电机起始位置，记录电机的初始位置
    float Pos;           // 当前位置，电机的当前位置
    float MaxPos;        // 最大位置限制，电机的运动范围上限
    float MinPos;        // 最小位置限制，电机的运动范围下限
    s16 temp;            // 临时变量，用于存储中间计算结果或临时数据
} ST_DJI_MOTOR;          //  DJI电机结构体，用于存储电机相关的所有参数和状态

/**J60_Structrue**/
// Motor
typedef struct /** * @brief DEEP_MOTOR 结构体定义，用于存储电机控制参数和状态信息 */
{
    uint8_t motor_id_; //  电机ID标识符，用于区分不同电机
    uint8_t cmd_;      //  电机控制模式
    float position_;   //  电机目标位置
    float velocity_;   //  电机目标速度
    float torque_;     //  电机目标扭矩
    float kp_;         //  Kp
    float kd_;         //  Kd
    float MaxPos;      //  电机最大位置限制
    float MinPos;      //  电机最小位置限制
    float temp_;       //  电机温度监测值
} DEEP_MOTOR;

// Data received in main

typedef struct
{
    uint8_t motor_id_; // 电机ID，8位无符号整数
    uint8_t cmd_;      // 控制命令，8位无符号整数
    float position_;   // 位置值，单位是弧度
    float velocity_;   // 速度值，单位是弧度/秒
    float torque_;     // 扭矩值，单位是N·m
    bool flag_;        // 标志位，布尔类型
    float temp_;       // 温度值，单精度浮点数
    uint16_t error_;   // 错误码，16位无符号整数
} MotorDATA;           // J60接收数据结构

// Data transmitted to the j60 motor
typedef struct
{
    uint8_t motor_id_; // 电机ID标识符，用于区分不同的电机
    uint8_t cmd_;      // 电机控制命令，如启动、停止、复位等
    float position_;   // 目标位置值，单位是弧度
    float velocity_;   // 目标速度值，单位是弧度/秒
    float torque_;     // 目标扭矩值，单位是N·m
    float kp_;         // 位置环比例增益参数
    float kd_;         // 位置环微分增益参数
} MotorCMD;            // J60控制命令结构

/**
 * 温度标志枚举类型定义
 * 用于标识不同组件的温度状态标志
 */
enum Temp_Flag
{
    kDriverTempFlag = 0, // 驱动器温度标志，值为0
    kMotorTempFlag = 1   // 电机温度标志，值为1
};

/****************************************************************/

/*A1*/
typedef enum
{
    DISABLE_MODE = 0,
    ENABLE_MODE = 10
} A1_MODE;
// ！！！使用时注意电机模式
typedef struct
{
    uint8_t ID;
    uint8_t mode;
    float T;
    float W;
    float Pos;
    float K_P;
    float K_W;
} A1_CTRL_DATA; // A1电机控制结构体
typedef struct
{
    uint8_t ID;
    uint8_t mode;
    float T;
    float Temp;
    float Error;
    float W;
    float Pos;
    float Acc;
    float K_P;
    float K_W;
    float gyro[3];
    float acc[3];
} A1_RECEIVE_DATA; // A1电机接收数据结构体

typedef struct
{
    A1_CTRL_DATA Ctrl_Data;   // A1真实控制结构体
    A1_RECEIVE_DATA Rec_Data; // 电机接收的反馈值
    u8 Flag_Init;             // 电机标0初始位置的标志位
    float Init_Rad;           // 最初位置(弧度制)，基于此计算目标角度和反馈的真实角度
    float TargetPos;          // 真实目标角度(弧度值)
    float RealPos;            // 真实反馈角度(弧度值)
} A1_STRUCTRUE;

/****************************************************************/

/*DM*/

/****************************************************************/
//
typedef struct
{
    int id;     //  电机ID标识，用于区分不同的电机
    int state;  //  电机状态，表示电机的当前工作状态
    int p_int;  //  位置积分值，用于位置控制中的积分项
    int v_int;  //  速度积分值，用于速度控制中的积分项
    int t_int;  //  转矩积分值，用于转矩控制中的积分项
    int kp_int; //  位置控制的比例系数整数部分
    int kd_int; //  位置控制的微分系数整数部分
    float pos;  //  电机位置值，弧度
    float vel;  //  电机速度值，弧度/s
    float tor;  //  电机转矩值，表示电机的当前转矩
    float Kp;
    float Kd;
    float Tmos;   //  电机温度值，表示电机的当前温度
    float Tcoil;  //  电机线圈温度值，表示电机线圈的当前温度
} motor_fbpara_t; //  电机反馈参数结构体类型定义，用于存储电机的各项反馈参数

/**
 * @brief 电机控制结构体定义
 *
 * 该结构体用于存储电机控制相关的参数和设定值
 */
typedef struct
{
    uint8_t mode;  /**< 电机控制模式 */
    float pos_set; /**< 位置设定值 */
    float vel_set; /**< 速度设定值 */
    float tor_set; /**< 转矩设定值 */
    float cur_set; /**< 电流设定值 */
    float kp_set;  /**< 位置环比例系数 */
    float kd_set;  /**< 速度环微分系数 */
} motor_ctrl_t;    /**< 电机控制结构体类型别名 */

/* 电机数据结构体定义 */
typedef struct
{
    uint16_t id;         /* 电机ID标识 */
    uint16_t mst_id;     /* 主控ID标识 */
    motor_fbpara_t para; /* 电机反馈参数结构体 */
    motor_ctrl_t ctrl;   /* 电机控制参数结构体 */
} Motor_DM;              /* 电机数据管理结构体 */

/*DT35*/
typedef struct
{
    uint8_t LimitSwitch[8];      // 行程开关状态
    uint8_t airOperatorTxBuf[6]; //  储存气动板控制数据
    uint8_t airOperatorTx;
} AIR_OPERATOR;

/****************** system monitor ********************/
typedef struct
{
    uint16_t J60;
    uint16_t A1;
    uint16_t DM;
    uint16_t DJI_3508;

    uint16_t stretch_2006;
    uint16_t AirOperater;
    uint16_t withinBoardCommunicate;

    uint16_t StateMachineTask;
    uint16_t PIDCalcTask;
    uint16_t GravityCompensationTask;
    uint16_t MotorControlTask;
    uint16_t WithinBoardCommunicateTask;
} SINGLE_MONITOR;
typedef struct
{
    uint32_t tim;
    SINGLE_MONITOR cntMonitor;
    SINGLE_MONITOR fpsMonitor;
    SINGLE_MONITOR errorMonitor;
} SYSTEM_MONITOR;

typedef enum
{
    INIT_ACTION,                 // 0
    DEINIT,                      // 1
    GET_WEAPON_HEAD_FOR_COMBINE, // 2
    ENDING_COMBINE,              // 3
    GET_KFS,                     // 4
    LEAVE_KFS_BACK,              // 5
    LEAVE_KFS_FRONT,             // 6
    STORAGE_KFS,                 // 7
    GET_KFS_BEHIND,              // 8
    USE_KFS,                     // 9
    KEEP_QUIET                   // 10
} Action;

typedef enum
{
    STAND_BY,
    INIT,
    WAIT_FOR_COMBINE,
    READY_TO_GET_KFS,
    HOLDING_KFS
} State;

extern Action actionFlag;      // 执行什么动作
extern State stateFlag;        // 保持何种状态
extern uint8_t actionCpltFlag, // 动作是否完成
    chassisMoveFlag,           // 是否允许地盘移动
    completeActionFlag;        // 是否完成动作最后一步
extern int32_t sm_cnt;
extern uint8_t KFS_height;          // 0:lower 1:higher 2:top
extern uint8_t KFS_orientation;     // 0:straight 1:left 2:right
extern uint8_t KFS_level;           // 0:middle 1:high
extern uint8_t use_KFS_orientation; // 0:LEFT 1:RIGHT 2:STRAIGHT
extern uint8_t noWeapon;
extern uint8_t init_mode;

// system monitor
extern SYSTEM_MONITOR system_monitor;
// CAN Communicate
extern uint8_t CAN1_RxBuf[8], CAN1_TxBuf[8], CAN2_RxBuf[8], CAN2_TxBuf[8];

// Within Board Communicate
extern uint8_t RxBufFromZGT[6], TxBufToZGT[6];
extern uint8_t WBC_TxCplt_Flag; // 板间通讯发送完成标志位

// gravity Compensation
extern uint8_t gravityCompensation_state; // 0:without KFS;1:with KFS

// AirOperator
extern AIR_OPERATOR airOperator;

// DJI variables
extern ST_DJI_MOTOR stretch_2006, DJI_3508;

// A1 variables
extern A1_STRUCTRUE Motor_A1;
extern uint8_t usartBufForA1[78];
extern u8 A1_TxCplt_Flag; // A1发送完成标志位
extern fp32 gTorqueA1;

// DM variables
extern Motor_DM stretch_DM;
extern fp32 gTorqueDM;

// J60 variables
extern DEEP_MOTOR Gimbal_J60;
extern MotorCMD Gimbal_J60_CMD;
extern MotorDATA Gimbal_J60_Receive;

#endif
