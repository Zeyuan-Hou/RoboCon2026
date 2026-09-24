#ifndef __GLOBAL_DECLARE_H__
#define __GLOBAL_DECLARE_H__

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define PI 3.14159265368f
#define PI2 6.2831853072f
#define RADIAN 0.0174532922f
#define RADIAN_10 0.00174532922f

#define UMAX_GM6020 27000    /**< GM6020电机最大电流值 */
#define UMAX_M3508 14000     /**< M3508电机最大电流值 */
#define UMAX_M2006 9000      /**< M2006电机最大电流值 */
#define GM6020_uiGearRatio 1 /**< GM6020齿轮比 */
#define M3508_uiGearRatio 19 /**< M3508齿轮比 */
#define M2006_uiGearRatio 36 /**< M2006齿轮比 */
#define M3508_siNumber 8192  /**< M3508编码器线数 */

typedef struct
{
    float fpDes; // 控制变量目标值
    float fpFB;  // 控制变量反馈值

    float fpKp; // 比例系数Kp
    float fpKi; // 积分系数Ki
    float fpKd; // 微分系数Kd

    float fpE;    // 本次偏差
    float fpEMin; // 偏差死区
    float fpEMax; // 偏差限幅

    float fpPreE;    // 上次偏差
    float fpSumE;    // 总偏差
    float fpSumEMax; // 积分偏差最大值

    float fpU;     // 总输出
    float fpUMax;  // 总输出最大值
    float fpUpMax; // 比例项输出限幅
    float fpUdMax; // 微分项输出上限
    float fpUiMax; // 积分项输出限幅 ////

    float fpUKp; // 比例项输出
    float fpUKi; // 积分项输出
    float fpUKd; // 微分项输出
    float fpElimit; ////
    float fpEforID; ////
} ST_PID;

typedef struct
{
    float fpDes; // 控制变量目标值
    float fpFB;  // 控制变量反馈值
    float fpE;   // 本次偏差

    float fpKp; // 比例系数Kp
    float fpKi; // 积分系数Ki

    float fpU; // 总输出
    float fpUKp; // 比例项输出
    float fpUKi; // 积分项输出

    float fpUMax;  // 总输出最大值
    float fpUpMax; // 比例项输出限幅
    float fpUiMax; // 积分项输出限幅
    float fpEMax;  // 偏差限幅

    float fpKffv; // 前馈系数Kff
    float fpKffa; // 前馈系数Kff
    float last_target; // 上次目标值
    float fpUff; // 前馈输出
    float fpUffMax; // 前馈输出最大值
} ST_PI_Feedforward;

typedef struct
{
    float fpDes; // 控制变量目标值
    float fpFB;  // 控制变量反馈值

    float fpKp; // 比例系数Kp
    float fpKi; // 积分系数Ki
    float fpKd; // 微分系数Kd

    float fpE;    // 本次偏差
    float fpPreE; // 上次偏差
    float fpSumE; // 总偏差
    float fpEMin; // 小于该值启动积分项
    float fpEMax; // 偏差限幅

    float fpU;     // 总输出
    float fpUMax;  // 总输出最大值
    float fpUpMax; // 比例项输出限幅
    float fpUdMax; // 微分项输出上限
    float fpUiMax; // 积分项输出限幅

    float fpUKp; // 比例项输出
    float fpUKi; // 积分项输出
    float fpUKd; // 微分项输出

    float const_far;         // 模糊PID远距离阈值
    float const_near;        // 模糊PID近距离阈值
    float membership_degree; // 模糊隶属度

    float kp_init;   // Kp初始值
    float upmax_init; // 总输出最大值初始值

    float kp_boost;   // Kp提升系数
    float upmax_boost; // 总输出最大值提升系数
} ST_PID_Fuzzy; // 模糊逻辑pid，专门用于不计算实时目标速度的纯位置控制导航优化

typedef struct
{
    float x1;  // 位置
    float x2;  // 速度
    float x3;  // 加速度
    float x;   // 位移
    float r;   // TD跟踪因子（决定跟踪速度，r越大跟得越快，如果追求快速响应，微分预测的滤波效果会变差）
    float h;   // TD滤波因子（算法式中的h0，h0越大微分预测的滤波效果越好）
    float T;   // TD积分步长（h为步长,h越小滤波效果越好，这个值应该与采样周期一致）
    float aim; // 目标位置
    float Inner_Original;
    float Inner_Derived;
} ST_TD;

typedef struct
{
    ST_PID inner;
    ST_PID outer;
    ST_TD td;
    float final_fpU;
} ST_CASCADE_PID;

enum State_ControlLoop
{
    SPEED_LOOP,
    MULTIPLE_LOOP,
    MULTIPLE_LOOP_TD,
};

enum MOTOR_TYPE
{
    M2006,
    M4219
};

// 电机码盘结构体
typedef struct
{
    int siRawValue;    /**< 当前编码器原始值 */
    int siPreRawValue; /**< 上次编码器原始值 */
    int siDiff;        /**< 编码器差值 */
    int siSumValue;    /**< 编码器累计值 */
    float siGearRatio; /**< 齿轮比 */
    int siNumber;      /**< 编码器线数 */
    float fpSpeed;     /**< 编码器测得的速度，单位：转/分钟 */
    uint8_t state;     // 判断初值是否为0，用于清零大疆电机初始编码器带来的角度
} ST_ENCODER;

// 电机总结构体
typedef struct
{
    ST_ENCODER motor_encoder; /**< 电机编码器信息 */
    float EncoderNum;         /**< 编码器计数值 */
    float encoder_speed;      /**< 编码器测得的速度 */
    float angle;              /**< 电机角度位置 */
    float anglev;             /**< 电机角速度 */
    float motor_current;      /**< 电机电流 */

    ST_PID pid_inner;
    ST_PID pid_outer;
    ST_TD td;
    float outerTarget;   /**< 外环目标位置 */
    float outerFeedback; /**< 外环反馈位置 */
    float innerFeedback; /**< 内环反馈速度 */

    uint32_t motor_id;
    enum MOTOR_TYPE motor_type;
    uint8_t uiGearRatio;
    float Input;
    float Input_v;
    enum State_ControlLoop ControlLoop_State;
    uint8_t Motor_RxMsg[8];
} ST_MOTOR;

// 一阶低通滤波
typedef struct
{
    float preout;   // 上一个输出值，用于保持滤波器状态，以便在连续调用之间维持滤波效果
    float out;      // 当前输出值，即经过低通滤波处理后的信号
    float in;       // 输入值，这是将要被滤波的原始信号
    float off_freq; // 截止频率，近似为Hz
    float samp_tim; // 采样步长，单位为s
} ST_LPF;           // 定义了一阶低通滤波器的结构体，一阶意味着它的传递函数有一阶多项式

// J60 start
#define J60_POSITION_MIN -40.0f
#define J60_POSITION_MAX 40.0f
#define J60_VELOCITY_MIN -40.0f
#define J60_VELOCITY_MAX 40.0f
#define J60_KP_MIN 0.0f
#define J60_KP_MAX 1023.0f
#define J60_KD_MIN 0.0f
#define J60_KD_MAX 51.0f
#define J60_TORQUE_MIN -40.0f
#define J60_TORQUE_MAX 40.0f
#define J60_MOTOR_TEMP_MIN -20.0f
#define J60_MOTOR_TEMP_MAX 200.0f
#define J60_DRIVER_TEMP_MIN -20.0f
#define J60_DRIVER_TEMP_MAX 200.0f
#define J60_SEND_POSITION_LENGTH 16
#define J60_SEND_VELOCITY_LENGTH 14
#define J60_SEND_KP_LENGTH 10
#define J60_SEND_KD_LENGTH 8
#define J60_SEND_TORQUE_LENGTH 16
#define J60_SEND_DLC_ENABLE 0
#define J60_SEND_DLC_DISABLE 0
#define J60_SEND_DLC_CONTROL 8
#define J60_RECEIVE_DLC_ENABLE 1
#define J60_RECEIVE_DLC_DISABLE 1
#define J60_RECEIVE_DLC_CONTROL 8

#define J60_ENABLE_CMD 2
#define J60_DISABLE_CMD 1
#define J60_CONTROL_CMD 4
#define J60_RECEIVE_POSITION_LENGTH 20
#define J60_RECEIVE_VELOCITY_LENGTH 20
#define J60_RECEIVE_TORQUE_LENGTH 16
#define J60_RECEIVE_TEMP_FLAG_LENGTH 1
#define J60_RECEIVE_TEMP_LENGTH 7

// J60是否正常工作的状态，用于过热或堵转保护
typedef enum
{
    OK,
    PROBLEM,
    PROTECTED,
} J60_Status;

typedef struct
{
    J60_Status status;
    int j60_problem_time;
    uint8_t motor_id_;
    uint8_t cmd_;
    float position_;
    float velocity_;
    float torque_;
    bool flag_;
    float temp_;
    uint16_t error_;
} J60_MotorDATA;

typedef struct
{
    uint8_t motor_id_;
    uint8_t cmd_;
    ST_TD j60_td;
    float position_;
    float velocity_;
    float torque_;
    float kp_;
    float kd_;
} J60_MotorCMD;

#pragma pack(push, 1)

__PACKED_STRUCT ReceivedMotionData_T
{
    uint32_t position : 20;
    uint32_t velocity : 20;
    uint16_t torque : 16;
    uint8_t temp_flag : 1;
    uint8_t temperature : 7;
};

typedef __PACKED_STRUCT ReceivedMotionData_T J60_ReceivedMotionData;

#pragma pack(pop)
// J60 end

// GO1 start
#pragma pack(1)
/**
 * @brief 电机模式控制信息
 *
 */
typedef struct
{
    uint8_t id : 4;      // 电机ID: 0,1...,13,14 15表示向所有电机广播数据(此时无返回)
    uint8_t status : 3;  // 工作模式: 0.锁定 1.FOC闭环 2.编码器校准 3.保留
    uint8_t reserve : 1; // 保留位
} RIS_Mode_t;            // 控制模式 1Byte

/**
 * @brief 电机状态控制信息
 *
 */
typedef struct
{
    int16_t tor_des; // 期望关节输出扭矩 unit: N.m      (q8)
    int16_t spd_des; // 期望关节输出速度 unit: rad/s    (q8)
    int32_t pos_des; // 期望关节输出位置 unit: rad      (q15)
    int16_t k_pos;   // 期望关节刚度系数 unit: -1.0-1.0 (q15)
    int16_t k_spd;   // 期望关节阻尼系数 unit: -1.0-1.0 (q15)

} RIS_Comd_t; // 控制参数 12Byte

/**
 * @brief 电机状态反馈信息
 *
 */
typedef struct
{
    int16_t torque;      // 实际关节输出扭矩 unit: N.m     (q8)
    int16_t speed;       // 实际关节输出速度 unit: rad/s   (q8)
    int32_t pos;         // 实际关节输出位置 unit: rad     (q15)
    int8_t temp;         // 电机温度: -128~127°C
    uint8_t MError : 3;  // 电机错误标识: 0.正常 1.过热 2.过流 3.过压 4.编码器故障 5-7.保留
    uint16_t force : 12; // 足端气压传感器数据 12bit (0-4095)
    uint8_t none : 1;    // 保留位
} RIS_Fbk_t;             // 状态数据 11Byte

/**
 * @brief 控制数据包格式
 *
 */
typedef struct
{
    uint8_t head[2]; // 包头         2Byte
    RIS_Mode_t mode; // 电机控制模式  1Byte
    RIS_Comd_t comd; // 电机期望数据 12Byte
    uint16_t CRC16;  // CRC          2Byte

} RIS_ControlData_t; // 主机控制命令     17Byte

/**
 * @brief 电机反馈数据包格式
 *
 */
typedef struct
{
    uint8_t head[2]; // 包头         2Byte
    RIS_Mode_t mode; // 电机控制模式  1Byte
    RIS_Fbk_t fbk;   // 电机反馈数据 11Byte
    uint16_t CRC16;  // CRC          2Byte

} RIS_MotorData_t; // 电机返回数据     16Byte

#pragma pack()

/// @brief 电机指令结构体
typedef struct
{
    unsigned short id;   // 电机ID，15代表广播数据包
    unsigned short mode; // 0:空闲 1:FOC控制 2:电机标定
    float T;             // 期望关节的输出力矩(电机本身的力矩)(Nm)
    float W;             // 期望关节速度(电机本身的速度)(rad/s)
    float Pos;           // 期望关节位置(rad)
    float K_P;           // 关节刚度系数(0-25.599)
    float K_W;           // 关节速度系数(0-25.599)
    ST_TD td;
    RIS_ControlData_t motor_send_data;
} MotorCmd_t;

/// @brief 电机反馈结构体
typedef struct
{
    unsigned char motor_id; // 电机ID
    unsigned char mode;     // 0:空闲 1:FOC控制 2:电机标定
    int Temp;               // 温度
    int MError;             // 错误码
    float T;                // 当前实际电机输出力矩(电机本身的力矩)(Nm)
    float W;                // 当前实际电机速度(电机本身的速度)(rad/s)
    float Pos;              // 当前电机位置(rad)
    int correct;            // 接收数据是否完整(1完整，0不完整)
    int footForce;          // 足端力传感器原始数值
    float Pos_converted;
    uint16_t calc_crc;
    uint32_t timeout;                // 通讯超时 数量
    uint32_t bad_msg;                // CRC校验错误 数量
    RIS_MotorData_t motor_recv_data; // 电机接收数据结构体
} MotorData_t;
// GO1 end

/*定位结构体（机器人整车ST_ROBOT）*/
// 坐标结构体
typedef struct
{
    float fpPosX; // 横坐标X（单位：mm）
    float fpPosY; // 竖坐标Y（单位：mm）
    float fpPosZ; // 纵坐标Z (单位：mm)
    float fpPosQ; // 航向角Q（单位：0.1度）
} ST_POS;

// 速度结构体
typedef struct
{
    float fpVx; // Ｘ方向速度（单位mm/s）
    float fpVy; // Y方向速度（单位：mm/s）
    float fpW;  // 角速度（单位0.1度/s）
} ST_VEL;

typedef struct
{
    ST_POS stPos;         // 机器人中心坐标姿态
    ST_VEL stVelt_global; // 机器人中心在全场坐标系下的速度
    ST_VEL stVelt_local;  // 机器人中心在局部坐标系下的速度
} ST_ROBOT;

// DT35近似垂直照射墙体 是 置信DT35数据 的必要条件
typedef enum
{
    DEG_0,
    DEG_90,
    DEG_180,
    DEG_270
} DT35_STANDARD_ANGLE;

typedef struct
{
    DT35_STANDARD_ANGLE dt35_angle;
    bool x_sucpect, y_sucpect; // 失信度，失信为1，可信为0
    float dt35_voltage_1, dt35_voltage_2, dt35_voltage_3, dt35_voltage_4;
    float dt35_pre_voltage_1, dt35_pre_voltage_2, dt35_pre_voltage_3, dt35_pre_voltage_4;
    float dt35_1, dt35_2, dt35_3, dt35_4;
    float dt35_robot_x, dt35_robot_y;
} ST_DT35;

// NRF24L01 引脚定义
#define NRF_CS_Pin GPIO_PIN_12
#define NRF_CS_GPIO_Port GPIOB
#define NRF_CE_Pin GPIO_PIN_8
#define NRF_CE_GPIO_Port GPIOD
#define NRF_IRQ_Pin GPIO_PIN_9
#define NRF_IRQ_GPIO_Port GPIOD

// 遥控器摇杆
#define LEFT_JS_X_MID 0x0828
#define LEFT_JS_X_MAX 0x0003
#define LEFT_JS_X_MIN 0x0FF6
#define LEFT_JS_Y_MID 0x0750
#define LEFT_JS_Y_MAX 0x0004
#define LEFT_JS_Y_MIN 0x0FF7
#define RIGHT_JS_MID 0x089E
#define RIGHT_JS_MIN 0x0005
#define RIGHT_JS_MAX 0x0FF8

/*遥控器结构体*/
typedef struct
{
    uint16_t usJsKey;     // 独立+矩阵按键
    uint16_t usJsLeft_X;  // 左摇杆x方向
    uint16_t usJsLeft_Y;  // 左摇杆y方向
    uint16_t usJsRight_X; // 右摇杆x方向
    uint16_t usJsRight_Y; // 右摇杆y方向
} ST_JS_VALUE;

// 坐标 start
typedef enum
{
    CARTESIAN, // 笛卡尔坐标系
    POLAR      // 极坐标系
} COORDINATE;

// 向量结构体 一般用作函数里局部变量
typedef struct
{
    // 笛卡尔直角坐标系
    float fpX; // X方向差
    float fpY; // Y方向差
    float fpW; // 旋转速度  叉乘运算？待定

    // 极坐标系
    float fpLength;  // 向量长度（单位mm）
    float fpThetha;  // 向量与X轴角度（单位:弧度）
    COORDINATE type; // 坐标系类型
} ST_VECTOR;
// 坐标 end

// 底盘导航 start
typedef struct
{
    ST_PID x;
    ST_PID y;
    ST_PID w;
    // ST_TD td_x;
    // ST_TD td_y;
    // ST_TD td_w;
} ST_Nav_Pid;

typedef struct
{
    float k1_x, k2_x;
    float k1_y, k2_y;
    float k1_w, k2_w;
}  PATH_FEEDFORWARD;

// typedef enum
// {
//     UP_DOWN_STATE_OFF = 0,
//     UP_APPROACH_1 = 1,
//     UP_APPROACH_2 = 2,
//     UP_APPROACH_3 = 3,
//     UP_APPROACH_4 = 4,
//     UP_UPSTAIRS = 5,
//     DOWN_APPROACH_AND_DOWNSTAIRS = 6,
// } UP_DOWN_STATE;

typedef struct
{
    // 定义自动路径数据，包含位置 PID 数据
    // 包含位置控制的 PID 数据
    ST_Nav_Pid pos_pid;   // 实时车身定位与目标位置的pid
    // ST_VEL auto_path_vel; // 自动路径规划出的目标速度
    ST_VEL basic_velt;    // 自动路径规划出的目标速度
    // uint32_t run_time;    // 运行时间
    // uint8_t run_time_flag;
    // UP_DOWN_STATE up_down_state; // 半自动上下台阶时底盘导航状态
    // uint32_t run_Sumtime;        // 总运行时间
    // uint32_t rotation_time;
    // uint8_t number;             // 自动路径选择第几个
    // uint8_t number_permutation; // 路径组合选择第几个
    uint8_t number_point;       // 微调路径选择第几个
    // uint8_t get_block;
} ST_Auto_Path;
// 路线类型

// typedef enum
// {
//     LINE,   // 直线
//     CIRCLE, // 圆弧
//     BEZIER  // 贝塞尔曲线
// } PATH_TYPE;

// // 自动路径相关结构体
// typedef struct
// {
//     ST_VECTOR Point_Start[20]; // 每段路径起始点
//     ST_VECTOR Point_Inc[20];   // 每段路径位移
//     ST_VECTOR V_Start[20];     // 每段路径起始速度
//     ST_VECTOR V_End[20];       // 每段路径终止速度
//     float Rotation_Start[20];  // 每段路径起始角度
//     float Rotation_Inc[20];    // 每段路径变化角度
//     float W_Start[20];         // 每段路径起始角速度
//     float W_End[20];           // 每段路径终止角速度
//     PATH_TYPE Path_Type[20];   // 每段路径的路线类型
//     float A[20];               // 每段路径加速度
//     float A_W[20];             // 每段路径角加速度
//     float R[20];               // 每段路径半径(直线路程半径为0)
//     float T[20];               // 每段路径对应时间
//     float T_W[20];             // 每段路径中角度变换对应时间
//     uint8_t num_module;        // 路径模块数量
//     uint8_t num_module_w;
// } PATH_PERMUTATION;

typedef enum
{
    QUINTIC_LINE,
    BEZIER,
} PATH_TYPE;

typedef struct
{
    ST_VECTOR point[5]; // 一段组合微调中有若干个目标点
    ST_VECTOR velt[5];  // 到达每个目标点对应的期望速度
    ST_VECTOR acc[5];
    int32_t time[5];    // 走每一段路径的期望时间
    int32_t extra_ff[5];// 用于适应上下斜坡的额外一阶前馈系数
    uint8_t point_num;  // 一段组合微调中小段路径的数量
    PATH_TYPE type[5]; // 该段路径的类型

    uint8_t point_inx; // 第几段小路径，从0计数
    uint8_t flag_point_to_point; // 用于navigate.c第一次进入选择路径
    uint8_t flag_path_set;  // 第一次进入小段路径时计算三次多项式系数
    uint32_t point_tim;     // 用于轨迹计算的计时器
} PATH_POINT;

typedef struct
{
    float k3, k2, k1, k0;
} Cube_Line;

typedef struct
{
    float k5;
    float k4;
    float k3;
    float k2;
    float k1;
    float k0;
} Quintic_Line;

typedef struct
{
    ST_VECTOR pos1, pos2, pos3, pos4;
    Cube_Line yaw;
} Bezier_3rd;

typedef enum
{
    CHASSIS_INIT = 0,            // 初始化，舵轮转向电机转一圈找零点
    CHASSIS_INIT_DONE = 1,       // 初始化完成，舵轮转向电机位于零点
    CHASSIS_OFF = 2,             // 底盘无输出，跳过pid直接给电机赋值0，pid累计误差清零
    RC_LOCAL = 3,                // 以车体坐标系遥控
    RC_GLOBAL = 4,               // 以全场坐标系遥控
    SEMIAUTO_UP_DOWN_STAIRS = 5, // 手操校准车身到台阶的距离以及yaw角后，一键上下台阶的导航部分
    NAV_LOCK = 6,                // 坐标锁死
    NAV_PERMUTATION_PATH = 7,    // 组合路径导航
    NAV_POINT_TO_POINT = 8,      // 仅需要目标位置的实时反馈位置的点到点导航
} Nav_State;

// 整个导航系统的主结构体
typedef struct
{
    Nav_State nav_state;                // 导航系统状态
    ST_VECTOR expect_robot_global_velt; // 目标全局速度
    ST_Auto_Path auto_path;             // 自动路径总结构体
} ST_Nav;                               // 定位 导航共用
// 底盘导航 end

// 前馈标志位
typedef enum
{
    WITHOUT_FORWARD, // 没有前馈
    WITH_FORWARD
} Feed_Forward_State;

// 三角舵轮底盘 start
//            |
//      *LU*  |  *RU*
//            |
//  ---------*O*----------
//            | 
//           *D*
//            |

#define UP_WHEEL_TO_ROBOT 305.5f   // distance(O, LU/RU)  mm
#define DOWN_WHEEL_TO_ROBOT 79.f   // distance(O, D)  mm
#define UP_ANGLE 0.93061f          // <ORU, Ox+>  rad
// #define DOWN_ANGLE 0.6610f        // <OLD, Ox+>  rad
#define R_WHEEL 63.5f
#define RUN_GEAR_RATIO 5.294f
#define TURN_GEAR_RATIO 2.23809524f

typedef struct
{
    ST_PI_Feedforward leftup; // 已包含摩擦补偿
    ST_PI_Feedforward rightup;
    ST_PI_Feedforward down;
    Feed_Forward_State feed_forward_state; // 用于斜坡补偿
} ST_Chassis_Run2;

// 四轮编码器返回值进行滤波，滤除其高频抖动
typedef struct
{
    ST_LPF leftup_velt;
    ST_LPF rightup_velt;
    ST_LPF down_velt;
} ST_WHEEL_ENCODER_VELT_FILTER;

// 暂时存储底盘电机目标位置/速度的结构体
typedef struct
{
    float rightup, leftup, down;
} chassis_run_des;
// 梯形舵轮底盘 end

// 小脚及上下台阶 start
#define FOOT_LEFTUP_TOR_MAX_1 -1.8f
#define FOOT_RIGHTUP_TOR_MAX_1 +2.0f
#define FOOT_DOWN_TOR_MAX_1 -9.f

#define FOOT_LEFTUP_TOR_MAX_2 -1.8f
#define FOOT_RIGHTUP_TOR_MAX_2 +2.0f
#define FOOT_DOWN_TOR_MAX_2 -9.f

#define MEIHUA_1 400.f
#define MEIHUA_2 200.f
#define MEIHUA_3 400.f
#define MEIHUA_4 200.f
#define MEIHUA_5 400.f
#define MEIHUA_6 600.f
#define MEIHUA_7 400.f
#define MEIHUA_8 600.f
#define MEIHUA_9 400.f
#define MEIHUA_10 200.f
#define MEIHUA_11 400.f
#define MEIHUA_12 200.f

typedef enum
{
    FOOT_INIT = 0,
    FOOT_CLEAR = 1,
    FOOT_UPSTAIRS_LV1 = 2,
    FOOT_UPSTAIRS_LV2 = 3,
    FOOT_DOWNSTAIRS_LV1 = 4,
    FOOT_DOWNSTAIRS_LV2 = 5,
    FOOT_TEST_STAND_PREPARE = 6,
    FOOT_TEST_STAND = 7,
    FOOT_TEST_SIT = 8,
} FOOT_STATE;

typedef enum
{
    UP11 = 0,
    UP12 = 1,
    UP13 = 2,
    UP14 = 3,
} FOOT_UP_LV1;

typedef enum
{
    UP21 = 0,
    UP22 = 1,
    UP23 = 2,
    UP24 = 3,
} FOOT_UP_LV2;

typedef enum
{
    DOWN11 = 0,
    DOWN12 = 1,
    DOWN13 = 2,
    DOWN14 = 3,
} FOOT_DOWN_LV1;

typedef enum
{
    DOWN21 = 0,
    DOWN22 = 1,
    DOWN23 = 2,
    DOWN24 = 3,
} FOOT_DOWN_LV2;

typedef struct
{
    FOOT_STATE foot_state; // 总状态
    FOOT_UP_LV1 foot_up_lv1;       // 内部子状态
    FOOT_UP_LV2 foot_up_lv2;
    FOOT_DOWN_LV1 foot_down_lv1;
    FOOT_DOWN_LV2 foot_down_lv2;
    float leftup_aim_angle; // 对应的各电机当前状态的目标角度
    float rightup_aim_angle;
    float down_aim_angle;
    uint8_t up_tor_feedforward_flag;   // 是否需要前脚重力前馈
    uint8_t down_tor_feedforward_flag; // 是否需要后脚重力前馈
    int up_foot_circle_count;          // 小脚上下台阶计数，用于统计go1电机转圈数
    uint8_t runtime_flag;
    uint32_t runtime;
} FOOT;

typedef struct
{
    float leftup;
    float rightup;
    float down;
} FOOT_G_feedforward;
// 小脚及上下台阶 end

// 视觉
typedef struct
{
    float x1;
    float y1;
    float z1;
    float yaw1;

    float x2;
    float y2;
    float z2;
    float yaw2;
} VISION_DATA_AUTO;

typedef struct
{
    float x[5];
    float y[5];
    float yaw[5];
    float lpf_k;
    int32_t tim;
    uint8_t flag;
} Location_Filter;

// 主状态机 start
typedef enum
{
    Total_Init = 0,
    Total_Clear = 1,
    TASK_1_4_7_10 = 2,
    TASK_2_5_8_11 = 3,
    TASK_10_7_4_1 = 4,
    TASK_11_8_5_2 = 5,
} ACE_State;

typedef enum
{
    from_One_to_2_1 = 0,
    from_2_1_to_2_2 = 1,
    from_2_2_to_2_3 = 2,
    from_2_3_to_2_4 = 3,
    from_2_4_to_Three = 4,
} Across_Forest;

typedef struct
{
    ACE_State state;
    Across_Forest across;
    uint8_t path_inx;
    uint8_t flag;
    uint8_t flag_;
    uint8_t flag__;
} ACE_Manipulator;
// 主状态机 end

typedef struct
{
    uint16_t can1_d;
    uint16_t can1_d_turn;
    uint16_t can1_foot;
    uint16_t can1_travelSwitch;
    uint16_t can1_j60;
    uint16_t can2_lu;
    uint16_t can2_lu_turn;
    uint16_t can2_ru;
    uint16_t can2_ru_turn;
    uint16_t can2_j60;
    uint16_t go1_left;
    uint16_t go1_right;
    uint16_t rc;
    uint16_t vofa;
    uint16_t task1;
    uint16_t task2;
    uint16_t task3;
    uint16_t task4;
    uint16_t task5;
    uint16_t task6;
    uint16_t VISION_REC;
    uint16_t VISION_TX;
    uint16_t UPPER_TO_LOWER; // 11 //接收上板数据
    uint16_t LOWER_TO_UPPER; // 12 //向上板发数据
} MONITOR;

typedef struct
{
    uint32_t time_base; // 每1ms自加一次 作为时间基准
    MONITOR rate_cnt;
    MONITOR rate_fps;
    MONITOR system_error;
} ST_SYSTEM_MONITOR;

extern ST_MOTOR leftup_motor, rightup_motor, down_motor;
extern ST_MOTOR leftup_turn_motor, rightup_turn_motor, down_turn_motor, foot_motor;
extern float leftup_init_angle, rightup_init_angle, down_init_angle;
extern uint8_t leftup_init_flag, rightup_init_flag, down_init_flag;

extern J60_MotorDATA j60_motor_data_down1, j60_motor_data_down2;
extern J60_MotorCMD j60_motor_cmd_down1, j60_motor_cmd_down2;
extern uint8_t j60_Tx[8];
extern ST_CASCADE_PID j60_pid;

extern MotorCmd_t go1_send_left, go1_send_right;
extern MotorData_t go1_recv_left, go1_recv_right;
extern RIS_MotorData_t uart_rx_data;
extern float go1_left_0, go1_right_0;
extern uint8_t go1_left_init_flag, go1_right_init_flag;

extern ST_ROBOT stRobot;

extern float LineAccelStep;
extern ST_Nav nav;

extern ST_WHEEL_ENCODER_VELT_FILTER wheel_encoder_velt_filter;
extern ST_VECTOR expect_robot_local_Velt;
extern chassis_run_des steer_velt, steer_pos, steer_pos_pre, friction_feedforward, cur_steer_velt, fric_k;
extern ST_Chassis_Run2 chassis_run2;    

extern FOOT foot;
extern FOOT_G_feedforward foot_g_feedforward;
extern uint8_t foot_up_G_feedforward_flag, foot_down_G_feedforward_flag;

// extern uint16_t dt35_distance[5];
// extern ST_DT35 dt35_now;

extern uint8_t travel_switch;
extern uint8_t travel_switch_mode[8];

extern uint8_t FLAG_NRF;
extern ST_JS_VALUE Js_Value;
extern uint16_t RC_Key_pre_value;

extern long long int tim_ms;
extern long long int time_cnt, time_cnt_pre;

extern ST_SYSTEM_MONITOR monitor;

extern uint8_t vision_rec[34];
extern VISION_DATA_AUTO vision_data_recieve;
extern Location_Filter location_filter;

extern PATH_POINT Path_Point;
extern Cube_Line cube_x, cube_y, cube_w;
extern Quintic_Line quintic_x, quintic_y, quintic_w;
extern Bezier_3rd bezier;
extern PATH_FEEDFORWARD path_ff;
extern uint8_t flag_lock;

extern ACE_Manipulator ace;

#endif
