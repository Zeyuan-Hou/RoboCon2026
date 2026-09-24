#ifndef __GLOBAL_VARIABLES_H__
#define __GLOBAL_VARIABLES_H__

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "main.h"

typedef FDCAN_HandleTypeDef hcan_t;

#define PI 3.14159265368f
#define PI2 6.2831853072f

#define RADIAN 0.0174532922f     
#define RADIAN_10 0.00174532922f
#define RADIAN_15 0.261799387f
#define RADIAN_45 0.785398163f
#define RADIAN_75 1.308996939f
#define RADIAN_100 0.000174532922f
#define RADIAN_105 1.832595715f
#define RADIAN_135 2.356194490f
#define RADIAN_165 2.879793266f

#define UMAX_GM6020 27000    /**< GM6020最大输出*/
#define UMAX_M3508 16000     /**< M3508最大输出*/
#define UMAX_M2006 10000      /**< M2006最大输出*/
#define UMAX_MG6012 2048     /**< MG6012最大输出 **/
#define GM6020_uiGearRatio 1 /**< GM6020 减速比 */
#define M3508_uiGearRatio 19 /**< M3508减速比 */
#define M2006_uiGearRatio 36 /**< M2006减速比 */
#define M3508_siNumber 8192  /**< M3508编码器线数 */
#define M2006_siNumber 8192  /**< M2006编码器线数 */

#define MY_J60_MAX_POS 40
#define MY_J60_MIN_POS -40

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

typedef struct
{
    ST_PID inner; // 内环PID
    ST_PID outer; // 外环PID
    float output; // 输出值
} ST_CascadePID;

typedef struct
{
    float preout;   /**< 前一次滤波输出值 */
    float out;      /**< 当前滤波输出值 */
    float in;       /**< 滤波器输入值 */
    float off_freq; /**< 截止频率 */
    float samp_tim; /**< 采样时间 */
} ST_LPF;

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
    int siRawValue;    ///< 编码器的当前原始值
    int siPreRawValue; ///< 编码器的前一次原始值
    int siDiff;        ///< 编码器当前值与前一次值的差值
    int siSumValue;    ///< 编码器值的累加值
    float siGearRatio; ///< 编码器的齿轮比
    int siNumber;      ///< 编码器的编号
    float fpSpeed;     ///< 通过编码器计算得到的速度
    uint8_t state;          ///< 编码器的工作状态
} ST_ENCODER;

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
    float tor_cur;
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
    int16_t temp;            // 临时变量，用于存储中间计算结果或临时数据
    uint8_t err;
} ST_DJI_MOTOR;          //  DJI电机结构体，用于存储电机相关的所有参数和状态

/**J60_Structrue**/
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

enum Temp_Flag
{
    kDriverTempFlag = 0, // 驱动器温度标志，值为0
    kMotorTempFlag = 1   // 电机温度标志，值为1
};

/*A1*/
typedef enum
{
    DISABLE_MODE = 0,
    ENABLE_MODE = 10
} A1_MODE;

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
    uint8_t Flag_Init;             // 电机标0初始位置的标志位
    float Init_Rad;           // 最初位置(弧度制)，基于此计算目标角度和反馈的真实角度
    float TargetPos;          // 真实目标角度(弧度值)
    float RealPos;            // 真实反馈角度(弧度值)
} A1_STRUCTRUE;

// DM
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

typedef struct
{
    uint16_t id;         /* 电机ID标识 */
    uint16_t mst_id;     /* 主控ID标识 */
    motor_fbpara_t para; /* 电机反馈参数结构体 */
    motor_ctrl_t ctrl;   /* 电机控制参数结构体 */
} Motor_DM;              /* 电机数据管理结构体 */

typedef struct
{
    uint8_t LimitSwitch[8];      // 行程开关状态
    uint8_t airOperatorTxBuf[6]; //  储存气动板控制数据
    uint8_t airOperatorTx;
} AIR_OPERATOR;

//gyro ys320
typedef enum
{
    crc_err = -3,
    data_len_err = -2,
    para_err = -1,
    analysis_ok = 0,
    analysis_done = 1
} analysis_res_t;

#pragma pack(1)

typedef struct
{
    unsigned char header1; /*0x59*/
    unsigned char header2; /*0x53*/
    unsigned short tid;    /*1 -- 60000*/
    unsigned char len;     /*length of payload, 0 -- 255*/
} output_data_header_t;

typedef struct
{
    unsigned char data_id;
    unsigned char data_len;
} payload_data_t;

typedef struct
{
    unsigned int itow;
    unsigned short year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char miniute;
    unsigned char second;
} utc_data_t;

typedef struct
{
    float accel_x; /*unit: m/s2*/
    float accel_y;
    float accel_z;

    float angle_x; /*unit: ° (deg)/s*/
    float angle_y;
    float angle_z;

    float mag_x; /*unit: 归一化值*/
    float mag_y;
    float mag_z;

    float raw_mag_x; /*unit: mGauss*/
    float raw_mag_y;
    float raw_mag_z;

    float pitch; /*unit: ° (deg)*/
    float roll;
    float yaw;

    float quaternion_data0;
    float quaternion_data1;
    float quaternion_data2;
    float quaternion_data3;

    double latitude;  /*unit: deg*/
    double longtidue; /*unit: deg*/
    float altidue;    /*unit: m*/

    float vel_n; /*unit: m/s */
    float vel_e;
    float vel_d;

    utc_data_t utc_data; /*utc data*/

    unsigned int sample_timestamp;     /*unit: us*/
    unsigned int data_ready_timestamp; /*unit: us*/

    float imu_temp;
} protocol_info_t;

#pragma pack()
// gyro

/****************** system monitor ********************/
typedef struct
{
    uint16_t J60;
    uint16_t A1;
    uint16_t DM;
    uint16_t DJI_3508;
    uint16_t stretch_2006;
    uint16_t AirOperater;
    uint16_t gyro;
    uint16_t withinBoardCommunicate;
    uint16_t IR;

    uint16_t Task1;
    uint16_t Task2;
    uint16_t Task3;
    uint16_t Task4;
    uint16_t Task5;
    uint16_t Task6;
} SINGLE_MONITOR;

typedef struct
{
    uint32_t tim;
    SINGLE_MONITOR cntMonitor;
    SINGLE_MONITOR fpsMonitor;
    SINGLE_MONITOR errorMonitor;
} SYSTEM_MONITOR;

typedef struct
{
    float dm_pos, j60_pos, a1_pos, m2006_pos, m3508_pos;
    float dm_tor, j60_tor, a1_tor, m2006_tor, m3508_tor;
} OBSERVATION;

// state machine
typedef enum
{
    GET_WEAPON = 1,
    COMBINE_PREPARE = 2,
    COMBINE_END = 3,

    COLLECT_PREPARE = 4,
    COLLECT_KFS = 5,
    THROW_KFS = 6,
    STORE_KFS = 7,

    PLACE_MID_KFS_NEAR = 8,
    GET_KFS_BEHIND = 9,
    PLACE_TOP_KFS_NEAR = 10,
    GET_KFS_FROM_R1 = 11,

    RE_GET_WEAPON = 12,

    TRANSFER_FOR_400 = 13,
    TRANSFER_FOR_TEMPORARY = 14,
    TRANSFER_FOR_TOP = 15,
    PLACE_TOP_KFS_FAR = 16,
    GET_KFS_FROM_R1_BACK = 17,
    PLACE_MID_KFS_FAR = 18,

    INIT_FOR_3 = 19,
    INIT = 20,

    PULL_KFS_TEMPORARY = 21,
    PULL_KFS_HOLD = 22,

    TRANSFER_FROM_TOP = 23,

    SILENT = 24,
    IDLE = 25,

    TRIAL_LEFT = 51,
    TRIAL_MID = 52,
    TRIAL_RIGHT = 53,

    ALL_PUSH = 55,
    INIT_PICK = 56,
} UP_ACTION_STATE;

typedef struct
{
    uint8_t kfs_height;
    uint8_t kfs_orientation;
    uint8_t next_action;
}  KFS_COLLECTION;

typedef struct
{
    uint8_t weapon_cplt_flag;
    uint8_t collect_kfs_cplt_flag; // 二区 
    uint8_t place_kfs_cplt_flag;
    uint8_t get_kfs_cplt_flag; // 三区
    uint8_t prepare_kfs_cplt_flag;
    uint8_t transfer_cplt_flag;
}  UP_CPLT_STATE;

typedef struct
{
    UP_ACTION_STATE act_state;
    KFS_COLLECTION kfs;
    UP_CPLT_STATE cplt_state;
    uint32_t tim;
    uint8_t set_flag;
    uint8_t last_act_state;
    float init_pos[5];
    float real_time_target_pos[5];
    uint8_t r1_orient;
} MANIPULATION;

typedef struct
{
    float pos_a1, pos_dm, pos_m2006, pos_m3508;
    float pos_a1_0, pos_dm_0;
    float k0_dm, k1_dm;
    float k0_a1, b0_a1, k1_a1, b1_a1, b0_a1_dm, b1_a1_dm, k1_a1_dm;
    float k0_m2006, k1_m2006;
    float k_m3508;
} G_FEEDFORWARD_PARAM;

typedef struct
{
    float A1, DM, M2006, M3508;
    uint8_t flag;
    G_FEEDFORWARD_PARAM param;
} G_FEEDFORWARD;

extern SYSTEM_MONITOR system_monitor;
extern OBSERVATION obs;

extern uint8_t rx_flag;
extern uint8_t CAN1_RxBuf[8], CAN1_TxBuf[8], CAN2_RxBuf[8], CAN2_TxBuf[8];

extern uint8_t RxBufFromZGT[8], TxBufToZGT[21], uart4_rev[8], uart2_rev[9], RxBufFromIR[3];
extern uint16_t IR_tim;
extern uint8_t Tx_IR[9];
extern uint8_t collect_kfs_queue[32], point1, point2;
extern int32_t communicate_tim;
extern uint8_t communicate_flag;
extern uint8_t r1_orient_flag;
extern uint8_t kfs_param_flag;

extern AIR_OPERATOR airOperator;

extern ST_DJI_MOTOR stretch_2006, DJI_3508;
extern uint8_t m3508_ctrl_flag;

extern A1_STRUCTRUE Motor_A1;
extern uint8_t usartBufForA1[78];
extern uint8_t a1_init_flag;
extern float a1_init_pos;
extern uint8_t a1_err;

extern Motor_DM stretch_DM;
extern uint8_t dm_init_flag;
extern float dm_init_pos;

extern DEEP_MOTOR Gimbal_J60;
extern MotorCMD Gimbal_J60_CMD;
extern MotorDATA Gimbal_J60_Receive;

extern MANIPULATION ace;
extern uint8_t flag_tic_tac_toe;
extern uint8_t flag_stand, index_stand;
extern G_FEEDFORWARD G_ff;
extern uint8_t flag_trial_orient;

extern uint8_t g_uart_rx_buf[512];
extern uint8_t g_decode_data[512];
extern uint16_t g_uart_rx_cnt, g_decode_data_pos;
extern protocol_info_t g_output_info;

extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1, huart4, huart3, huart7, huart2;
extern FDCAN_HandleTypeDef hfdcan1;
// extern DMA_HandleTypeDef hdma_usart2_rx, hdma_usart2_tx;
// extern uint8_t gyro_rx_ready;
// extern uint8_t gyro_rx_shadow_buf[512];
// extern int16_t gyro_rx_len;
#endif
