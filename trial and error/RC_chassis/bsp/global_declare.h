#ifndef __GLOBAL_DECLARE_H__
#define __GLOBAL_DECLARE_H__

#include "stdint.h"
#include <stdbool.h>

// 角度制
#define leftup_init_angle 0.0f
#define rightup_init_angle 0.0f
#define leftdown_init_angle 135.439453f
#define rightdown_init_angle -136.845703f

#define PI 3.14159265368f
#define PI2 6.2831853072f
#define RADIAN 0.0174532922f     // PI/180
#define RADIAN_10 0.00174532922f
#define RADIAN_15 0.261799387f
#define RADIAN_45 0.785398163f
#define RADIAN_75 1.308996939f
#define RADIAN_90 1.570796327f
#define RADIAN_105 1.832595715f
#define RADIAN_135 2.356194490f
#define RADIAN_165 2.879793266f

#define UMAX_GM6020 27000    /**< GM6020电机最大电流值 */
#define UMAX_M3508 14000     /**< M3508电机最大电流值 */
#define UMAX_M2006 9000      /**< M2006电机最大电流值 */
#define GM6020_uiGearRatio 1 /**< GM6020齿轮比 */
#define M3508_uiGearRatio 19 /**< M3508齿轮比 */
#define M2006_uiGearRatio 36 /**< M2006齿轮比 */
#define M3508_siNumber 8192  /**< M3508编码器线数 */

// matlab标定 THETA_A_UP THETA_A_DOWN THETA_B_UP THETA_B_DOWN
// A和B随动轮线速度方向与机器人局部坐标系Y轴的夹角。
#define ALPHA_A_Inc -2.369977//-2.37415609  // 0.77228663252223073154567600795417;
#define ALPHA_A_Dec -2.373173//-2.37711172  // 0.76659874568048147480681109300349;
#define ALPHA_B_Inc 2.359390//2.34997958 //-0.78445819003811279035431880402029;
#define ALPHA_B_Dec 2.358759//2.36122702 //-0.78857451449441562374431669013575;

// matlab标定 KA_UP KA_DOWN KB_UP KB_DOWN
//  随动轮编码器数值到实际物理位移的转换系数
//  位移(mm)=编码器值差×转换系数(mm/脉冲)
#define FW_Len_A_Inc -0.888323//-0.88098607 //-0.2195809107592067432879190391759;
#define FW_Len_A_Dec -0.890475//-0.88027724 //-0.22079534122602795243039963679621;
#define FW_Len_B_Inc -0.232325//-0.22569889 //-0.22499185661708787087320615682984;
#define FW_Len_B_Dec -0.230476//-0.22484166 //-0.22401719945817957779787832350848;

#define FW_Rob_Len 127.f//281.f                       // 随动轮中心与机器人中心的距离
#define FW_rob_Alpha 0.f // 随动轮坐标与机器人坐标的夹角（单位：弧度）

// // matlab的 KA DOWN等放在这里
//   A 和 B 随动轮线速度方向与机器人局部坐标系 Y 轴的夹角。
// matlab标定 THETA_A_UP THETA_A_DOWN THETA_B_UP THETA_B_DOWN
// A和B随动轮线速度方向与机器人局部坐标系Y轴的夹角。                          // 舵轮
// #define ALPHA_A_Inc 0.75150929846763070418802499261801 //-0.7436512910425646660783627339697
// #define ALPHA_A_Dec -2.3863990203177487536834178172285;
// #define ALPHA_B_Inc -0.71850342512286202723004180370481;
// #define ALPHA_B_Dec 2.4228255048882436639701154490467;

// // matlab标定的KA_UP KA_DOWN KB_UP KB_DOWN等放在这里
// //  随动轮编码器数值到实际物理位移的转换系数
// //  位移(mm)=编码器值差×转换系数(mm/脉冲)
// #define FW_Len_A_Inc -0.17932774471591958476146544398944;
// #define FW_Len_A_Dec -0.1800607922755442791284963277576;
// #define FW_Len_B_Inc -0.18482373797946513582779459738958;
// #define FW_Len_B_Dec -0.18554163020289279883989763675345;

// #define FW_Rob_Len 0       // 290.f//260.3f                         // 随动轮中心与机器人中心的距离
// #define FW_rob_Alpha 1.57f // 随动轮坐标与机器人坐标的夹角（单位：弧度）
                           // 舵轮
// #define ALPHA_A_Inc -2.5644254488942976877297041937709  //-2.3699770253274912157337439566618;
// #define ALPHA_A_Dec -2.4150778560479899681467941263691  //-2.3731726707176292734402522910386;
// #define ALPHA_B_Inc -0.99561060094540065890100777323823 // 2.3593895838501288686472889821744;
// #define ALPHA_B_Dec 0.78348023930735433140171153354459  // 2.3587588361459310704049130436033;

// // matlab标定的KA_UP KA_DOWN KB_UP KB_DOWN等放在这里
// //  随动轮编码器数值到实际物理位移的转换系数
// //  位移(mm)=编码器值差×转换系数(mm/脉冲)                        
// #define FW_Len_A_Inc -0.33813428798424477461637138731021 //-0.8883232683656019368356737686554;
// #define FW_Len_A_Dec -0.30253363215435497002303577573912 //-0.89047547379041136483834861792275;
// #define FW_Len_B_Inc -0.21378153862731844037092798771482 //-0.23232504723389549305956336411327;
// #define FW_Len_B_Dec -0.2781285887156612623982709919801  //-0.23047634972513630913226734264754;

// #define FW_Rob_Len 0       // 290.f//260.3f                         // 随动轮中心与机器人中心的距离
// #define FW_rob_Alpha 1.57f // 随动轮坐标与机器人坐标的夹角（单位：弧度）

// matlab标定的电压模拟量线性变换到实际距离（mm）的系数
#define K_DT35_X1 0.917067111049f // 1
#define B_DT35_X1 99.617068452831f

#define K_DT35_X2 0.f
#define B_DT35_X2 0.f

#define K_DT35_Y1 0.f
#define B_DT35_Y1 0.f

#define K_DT35_Y2 0.9077145807813f // 0
#define B_DT35_Y2 96.2195111723603f

/*场地参数*/
#define FIELD_WIDTH 8000
#define FIELD_HEIGHT 15000

#define LENGTH 235.0f                         // 半车长
#define WIDTH 235.0f                          // 半车宽
#define R_ROBOT Geometric_mean(LENGTH, WIDTH) // 轮子到车中心距离
#define RUN_GEAR_RATIO 6.f
#define R_WHEEL 65.f // 63.5f

// NRF24L01 引脚定义
#define NRF_CS_Pin GPIO_PIN_12
#define NRF_CS_GPIO_Port GPIOB
#define NRF_CE_Pin GPIO_PIN_8
#define NRF_CE_GPIO_Port GPIOD
#define NRF_IRQ_Pin GPIO_PIN_9
#define NRF_IRQ_GPIO_Port GPIOD

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
    float fpUdMax;   // 微分项输出上限

    float fpUKp; // 比例项输出
    float fpUKi; // 积分项输出
    float fpUKd; // 微分项输出

}ST_PID;

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
}ST_TD;

enum State_ControlLoop
{
    SPEED_LOOP,
    POSITION_LOOP,
    MULTIPLE_LOOP
};

enum MOTOR_TYPE
{
    M6020,
    M6C18
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
    uint8_t state;          // 判断初值是否为0，用于清零大疆电机初始编码器带来的角度
}ST_ENCODER;

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
    enum State_ControlLoop ControlLoop_State;
    uint8_t Motor_RxMsg[8];
}ST_MOTOR;

// 一阶低通滤波
typedef struct
{
    float preout;   // 上一个输出值，用于保持滤波器状态，以便在连续调用之间维持滤波效果
    float out;      // 当前输出值，即经过低通滤波处理后的信号
    float in;       // 输入值，这是将要被滤波的原始信号
    float off_freq; // 截止频率或称为权重，它决定了哪些频率成分可以通过滤波器
    float samp_tim; // 采样步长（时间），两次采样之间的时间，它对确定滤波器的时间常数至关重要
}ST_LPF;           // 定义了一阶低通滤波器的结构体，一阶意味着它的传递函数有一阶多项式

// 四轮编码器返回值进行滤波，滤除其高频抖动
typedef struct
{
    ST_LPF leftup_velt;
    ST_LPF rightup_velt;
    ST_LPF rightdown_velt;
    ST_LPF leftdown_velt;
}ST_WHEEL_ENCODER_VELT_FILTER;

/*dt35*/
// 四个DT35
/*              |dt35_y1
                |y+
                |
   dt35_x2 —————————————x+  dt35_x1
                |
                |
                |dt35_y2*/
typedef struct
{
    float dt35_voltage_x1, dt35_voltage_x2, dt35_voltage_y1, dt35_voltage_y2; // can接收 电压模拟量
    float dt35_x1, dt35_x2, dt35_y1, dt35_y2;                                 // 在locate_algorithm.c里DT35到墙的距离
    float robot_x, robot_y;                                                   // 机器人中心到墙的距离，在locate_algorithm.c里加上
    float robot_q;                                                            // 机器人中心姿态角
}ST_DT35;

/*定位结构体（机器人整车ST_ROBOT）*/
// 坐标结构体
typedef struct
{
    float fpPosX; // 横坐标X（单位：mm）
    float fpPosY; // 竖坐标Y（单位：mm）
    float fpPosQ; // 航向角Q（单位：0.1度）
}ST_POS;

// 速度结构体
typedef struct
{
    float fpVx; // Ｘ方向速度（单位mm/s）
    float fpVy; // Y方向速度（单位：mm/s）
    float fpW;  // 角速度（单位0.1度/s）
}ST_VEL;

typedef struct
{
    ST_POS stPos;         // 机器人中心坐标姿态
    ST_VEL stVelt_global; // 机器人中心在全场坐标系下的速度
    ST_VEL stVelt_local;  // 机器人中心在局部坐标系下的速度
}ST_ROBOT;

/*随动轮*/
// 随动轮码盘过线计数及随动轮相关结构体
typedef struct
{
    int siCoderACur; // 当前码盘A读数 编码器的值
    int siCoderAPre; // 上一次码盘A读数，判断随动轮旋转方向
    int siCoderBCur; // 当前码盘B读数
    int siCoderBPre; // 上一次码盘B读数

    ST_POS stPos; // 随动轮中心坐标姿态
}ST_FOLLOWER_WHEEL;

/*陀螺仪*/
typedef struct
{
    float fpQ_Cur; // 陀螺当前数据读数
    float fpQ_Pre; // 陀螺上一次数据读数，判断旋转方向
}ST_GYRO;

// 用于判断DT35启用条件
typedef enum
{
    DEG_0,
    DEG_90,
    DEG_180,
    DEG_270
}DT35_ANGLE;

typedef struct
{
    DT35_ANGLE dt35_angle;
    bool x_sucpect, y_sucpect; // 失信度，失信为1，可信为0
    float dt35_voltage_x1, dt35_voltage_y1, dt35_voltage_y2;
    float dt35_x1, dt35_x2, dt35_y1, dt35_y2;
    float dt35_x1_save[5], dt35_x2_save[5], dt35_y1_save[5], dt35_y2_save[5];
    float dt35_x1_save_sum, dt35_x2_save_sum, dt35_y1_save_sum, dt35_y2_save_sum;
    float dt35_robot_x, dt35_robot_y;
}ST_DT35_NEW;

/********************************************************************************************
利用四轮速逆解算出车体速度，可以将轮子编码器逆解算出的车体速度和惯导系统里的车体速度进行比较，判断打滑
直接加了滤波器，逆解算出的车体速度更加平滑一些
*********************************************************************************************/
typedef struct
{
    ST_LPF Vx;
    ST_LPF Vy;
    ST_LPF W;
}ST_WHEEL2BODY_VELT;

/********************************************************************************************
惯导系统得出来的车体速度
直接加了滤波器，解算出的车体速度更加平滑一些
*********************************************************************************************/
typedef struct
{
    ST_LPF global_vx;
    ST_LPF global_vy;
    ST_LPF global_w;
}ST_GLOBAL_VELT_FILTER;

typedef struct
{
    float v;          // 状态量 (速度估计)
    float P;          // 状态协方差
    float Q;          // 过程噪声
    float R_ins;      // 惯导观测噪声
    float R_whl_base; // 轮速基础观测噪声
    float slip_thres; // 打滑判断阈值 (mm/s)
    float slip_scale; // 打滑时噪声放大倍数
}KalmanFilter;

typedef enum
{
    CARTESIAN, // 笛卡尔坐标系
    POLAR      // 极坐标系
}COORDINATE;

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
}ST_VECTOR;

/*导航结构体（包含）*/
typedef struct
{
    float V_max;  // 最大速度
    float A_up;   // 加速度
    float A_down; // 减速度
}ST_Motion_Path;

// 自动导航路径的信息
typedef struct
{
    ST_PID x;
    ST_PID y;
    ST_PID w;
}ST_Nav_Pid;
typedef struct
{
    // 定义自动路径数据，包含位置 PID 数据
    // 包含位置控制的 PID 数据
    ST_Nav_Pid pos_pid;
    ST_VEL basic_velt;    // 自动路径规划出的目标速度
    uint32_t run_time;    // 运行时间
    uint32_t run_Sumtime; // 总运行时间
    uint32_t rotation_time;
    uint8_t number;             // 自动路径选择第几个
    uint8_t number_permutation; // 路径组合选择第几个
}ST_Auto_Path;

// 导航标志位 ，自己全部重新写
typedef enum
{
    NAV_INIT,          // 初始化
    NAV_OFF,           // 四轮无输出，卸力
    NAV_LOCK,          // 坐标锁死
    NAV_LOCAL_MANUAL,  // 局部坐标系手操
    NAV_GLOBAL_MANUAL, // 锁YAW手操
    NAV_AUTO_PATH,     // 自动路径导航
    NAV_PERMUTATION_PATH // 组合路径导航
} Nav_State;

// 整个导航系统的主结构体
typedef struct
{
    Nav_State nav_state;                // 导航系统状态
    ST_VECTOR expect_robot_global_velt; // 导航
    ST_Auto_Path auto_path;             // 自动路径总结构体
}ST_Nav;                               // 定位 导航共用

// pid状态标志位
typedef enum
{
    VELT_LOOP,  // 单环 无位置环纠偏
    OPEN_LOOP,  // 开环控制 不进行pid计算
    DOUBLE_LOOP // 双环
}Pid_State;

// 前馈标志位
typedef enum
{
    WITHOUT_FORWARD, // 没有前馈
    WITH_FORWARD     // 有前馈
}Feed_Forward_State;

/*底盘解算结构体*/
typedef struct
{
    ST_PID leftup;
    ST_PID rightup;
    ST_PID leftdown;
    ST_PID rightdown;
    Pid_State pid_state;
    Feed_Forward_State feed_forward_state;
}ST_Chassis_Run;

typedef struct
{
    ST_PID leftup_in;
    ST_PID leftup_out;
    ST_TD leftup_td;
    ST_PID rightup_in;
    ST_PID rightup_out;
    ST_TD rightup_td;
    ST_PID leftdown_in;
    ST_PID leftdown_out;
    ST_TD leftdown_td;
    ST_PID rightdown_in;
    ST_PID rightdown_out;
    ST_TD rightdown_td;
    Feed_Forward_State feed_forward_state;
} ST_Chassis_Turn;

/*遥控器结构体*/
typedef struct
{
    uint16_t usJsKey;            // 合并按键
    uint16_t indepen_usJsKey[8]; // 8个独立按键
    uint16_t usJsLeft_X;         // 左摇杆x方向
    uint16_t usJsLeft_Y;         // 左摇杆y方向
    uint16_t usJsRight_X;        // 右摇杆x方向
    uint16_t usJsRight_Y;        // 右摇杆y方向
}ST_JS_VALUE;

typedef enum
{
    PATH_INIT,
    PATH_END,
    PATH_ONGOING,
    PATH_FREE
} Path_State;

typedef struct
{
    float x;
    float y;
    float q;
} POINT;

typedef enum
{
    LINE,   // 直线
    CIRCLE, // 圆弧
    BEZIER  // 贝塞尔曲线
} PATH_TYPE;

typedef struct
{
    ST_VECTOR Point_Start[10], Point_Inc[10], V_Start[10], V_End[10];  // 依次存储每段路径的起点坐标，增量坐标，起始速度，结束速度
    float Rotation_Start[10], Rotation_Inc[10], W_Start[10], W_End[10]; // 没有用到
    PATH_TYPE Path_Type[10];                                           // 存储路径类型，直线和圆弧
    float A[10], A_W[10];                                               // 存储加速度和角加速度
    float R[10];                                                        // 存储圆弧的半径
    float T[10], T_W[10];                                               // 存储每段路径的时间
    uint8_t num_module, num_module_w;                                       // 存储路径的数量
    // A_W、T_W和num_module_w都会用在NavRotation(）函数中
} PATH_PERMUTATION;

extern ST_WHEEL2BODY_VELT Wheelvelt_To_Bodyvelt;

extern ST_MOTOR leftup_motor, leftdown_motor, rightup_motor, rightdown_motor;
extern ST_MOTOR leftup_motor_angle, leftdown_motor_angle, rightup_motor_angle, rightdown_motor_angle;
extern ST_WHEEL_ENCODER_VELT_FILTER wheel_encoder_velt_filter;

extern ST_DT35 dt35_save, dt35_now;
extern ST_DT35_NEW DT35_NEW;
extern uint16_t dt35_distance[5];
extern uint16_t dt35_x1, dt35_y1, dt35_x2, dt35_y2;

extern ST_FOLLOWER_WHEEL stFollowerWheel;
extern float degreeA, degreeB;

extern ST_GYRO Gyro_Data_Test;
extern int num_circle;
extern float fpSumPosQ;

extern float fpPosXOffset, fpPosYOffset, fpQOffset;
extern float fpStartX, fpStartY;

extern ST_ROBOT stRobot;
extern float v, v_pre, v_filtered, dx, dy, dx_pre, dy_pre;
extern float V, A;

extern ST_TD posX_veltX, posY_veltY, posW_veltW;
extern ST_GLOBAL_VELT_FILTER global_velt_filter;
extern KalmanFilter KF_Vx, KF_Vy, KF_W;

extern ST_Chassis_Run chassis_run;
extern ST_Chassis_Turn chassis_steer_angle;

extern ST_Nav nav;
extern uint8_t flag_lock, flag_global_manual;
extern uint8_t end_flag;
extern ST_PID Chassis_Global_Yaw_Pid;
extern uint8_t flag_permutation_path;

extern ST_JS_VALUE Js_Value;
extern uint8_t FLAG_NRF;

extern Path_State path_state;
extern POINT point_end;
extern PATH_PERMUTATION Path_Permuta;

extern float delta_x, delta_y, delta_q, v_max, a_max, w, alpha;
// extern float end_x, end_y, end_q, current_x, current_y, current_q;

#endif
