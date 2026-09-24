#ifndef __GLOBAL_DECLARE_H__
#define __GLOBAL_DECLARE_H__

#include "stm32h7xx_hal.h"
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
    float fpUiMax; // 积分项输出限幅

    float fpUKp; // 比例项输出
    float fpUKi; // 积分项输出
    float fpUKd; // 微分项输出
    float fpElimit;
    float fpEforID;
} ST_PID;

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

enum State_ControlLoop
{
    SPEED_LOOP,
    MULTIPLE_LOOP,
    MULTIPLE_LOOP_TD,
};

enum MOTOR_TYPE
{
    M2006,
    MAD_4219,
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
    float off_freq; // 截止频率或称为权重，它决定了哪些频率成分可以通过滤波器
    float samp_tim; // 采样步长（时间），两次采样之间的时间，它对确定滤波器的时间常数至关重要
} ST_LPF;

typedef struct
{
    uint16_t task1;
    uint16_t task2;
    uint16_t task3;

    uint16_t udp_recv;
    uint16_t udp_send;
} MONITOR;

typedef struct
{
    uint32_t time_base; // 每1ms自加一次 作为时间基准
    MONITOR rate_cnt;
    MONITOR rate_fps;
} ST_SYSTEM_MONITOR;

extern ST_SYSTEM_MONITOR monitor;

#endif /* __GLOBAL_DECLARE_H__ */
