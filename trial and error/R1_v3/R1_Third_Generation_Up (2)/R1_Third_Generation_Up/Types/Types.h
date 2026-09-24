#ifndef  __TYPES_H__
#define  __TYPES_H__

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h" 

/** -----------------------------
================================
      ######宏定义 #####
================================
 ---------------------------- **/
 
//*******************constant*****************************//

#define fp32 float
#define fp64 double	
#define u8 uint8_t
#define u32 uint32_t

#define pi 3.14159265358f
#define PI 3.14159265368f
#define PI2 6.2831853072f

// 角的度量方式包括弧度（radian）和角度（通常以度（degree）为单位）
#define RADIAN		0.0174532922f			//PI/180 
#define RADIAN_10 0.00174532922f // PI/1800，多次需要运算，故单独提取出来
#define RADIAN_100 0.000174532922f
#define RADIAN_15 0.261799387f
#define RADIAN_45 0.785398163f
#define RADIAN_75 1.308996939f

#define RADIAN_105 1.832595715f
#define RADIAN_135 2.356194490f
#define RADIAN_165 2.879793266f

//*******************Private Define************************//
/**
 * 电机控制相关常量定义
 * -----------------------------------------------------
 * 说明：定义了不同电机型号的最大电流值和齿轮比。
 * -----------------------------------------------------
 */
#define UMAX_GM6020        27000     /**< GM6020电机最大电流值 */
#define UMAX_M3508         14000     /**< M3508电机最大电流值 */
#define UMAX_M2006         9000      /**< M2006电机最大电流值 */
#define GM6020_uiGearRatio 1         /**< GM6020齿轮比 */
#define M3508_uiGearRatio  19        /**< M3508齿轮比 */
#define M3508_uiGearRatio_Longer  51 /**< 长减速箱M3508齿轮比 */
#define M2006_uiGearRatio  36        /**< M2006齿轮比 */
#define M3508_siNumber     8192      /**< M3508编码器线数 */

/**
 * 随动轮相关常量定义
 * -----------------------------------------------------
 * 说明：
 * -----------------------------------------------------
 */
// 随动轮通道参数
#define FollowerWheel_CoderA_Ch 4 // 随动轮A通道
#define FollowerWheel_CoderB_Ch 3 // 随动轮B通道

// matlab的 KA DOWN等放在这里
//   A 和 B 随动轮线速度方向与机器人局部坐标系 Y 轴的夹角。
// matlab标定 THETA_A_UP THETA_A_DOWN THETA_B_UP THETA_B_DOWN
//A和B随动轮线速度方向与机器人局部坐标系Y轴的夹角。
#define ALPHA_A_Inc 2.3635479490699906612860559107503//0.77228663252223073154567600795417;//0.7614903727054939119867071894987;//0.79162334795495759021122239573742;//0.73391963008362282039342971984297;//0.73388454804082614568727649384527;//0.72552841073043961017674519098364;
#define ALPHA_A_Dec 2.3664682105364480690923301153816//0.76659874568048147480681109300349;//0.75770947160294632727328689725255;//0.773739689719615708618505323102;//0.73949741556436110467842581783771;//0.7394490991526031509195604485285//0.74485825036227737427907413803041;
#define ALPHA_B_Inc -2.3614517290998890963749090587953//-0.78445819003811279035431880402029;//-0.79722192938325953104339305355097;//-0.77105730680585404801519189277315l;//-0.77986102005500868017406901344657;//0.26733857481581191350983317533974;//-0.78739386585441051291667236000649;
#define ALPHA_B_Dec -2.3599077611978271917791971645784//-0.78857451449441562374431669013575;//-0.80335053658832533685085763863754;//-0.78644580062906721540372245726758;//-0.78543554314687158424135304812808;//-0.25204047094419673724630115430045;//-0.81512847259566212354542358298204;

// matlab标定的KA_UP KA_DOWN KB_UP KB_DOWN等放在这里
//  随动轮编码器数值到实际物理位移的转换系数
//  位移(mm)=编码器值差×转换系数(mm/脉冲)
#define FW_Len_A_Inc -0.22174669312099748452737912884913//-0.2195809107592067432879190391759;//-0.21300854732823762405224954363803;//-0.2168791983588005123362307813295;//-0.86879280200113762067104516972904;//-0.86865874650200158857415999591467;//-0.86854451350083916594257971155457;
#define FW_Len_A_Dec -0.22206118207024572175356524894596//-0.22079534122602795243039963679621;//-0.21456299506907061669380709645338;//-0.22009512350621895926394699927187;//-0.87778487398167504007773231933243;//-0.87763223741475038242043638092582;//-0.88355678174946961078717322379816;
#define FW_Len_B_Inc -0.22186377737772988716358213423518//-0.22499185661708787087320615682984;//-0.23162122659526671042407031109178;//-0.22768946924042263169063460281905;//-0.23164559344676569074827909844316;//-0.077185790888697494716019775751192;//-0.22833365708133998572826328654628;
#define FW_Len_B_Dec -0.22167495421542818268001440173975//-0.22401719945817957779787832350848;//-0.23099784323905139804544717208046;//-0.22391021980541581104517945277621;//-0.23624415612251004059629622133798;//-0.23430730146719233597529807866522;//-0.23570714393367522832001270671753;

#define FW_Rob_Len  290.f//260.3f                         // 随动轮中心与机器人中心的距离
#define FW_rob_Alpha 0.f // 随动轮坐标与机器人坐标的夹角（单位：弧度）

/*dt35*/
/*              |dt35_y1
                |y+
                |
   dt35_x2 —————————————x+  dt35_x1
                |
                |
                |dt35_y2
*/
//matlab标定的电压模拟量线性变换到实际距离（mm）的系数
#define K_DT35_X1 1.003f
#define B_DT35_X1 470.36f

#define K_DT35_X2 1.0926f
#define B_DT35_X2 398.86f

#define K_DT35_Y1 0.9911f
#define B_DT35_Y1 399.05f

#define K_DT35_Y2 0.f
#define B_DT35_Y2 0.f

//距离
#define DT35_X1_to_center 320.00f
#define DT35_X2_to_center 320.00f
#define DT35_Y1_to_center 320.00f
#define DT35_Y2_to_center 320.00f
/*gyro*/
// 陀螺仪系数
#define K_ANTICLOCK 1.003577568182874f
#define K_CLOCK 1.004884856943465f

#define sin45 sin(45*PI/180.0f)
#define LENGTH 235.0f  //半车长
#define WIDTH 235.0f	//半车宽
#define R_ROBOT Geometric_mean(LENGTH,WIDTH) //轮子到车中心距离
#define RUN_GEAR_RATIO 6.f
#define R_WHEEL 65.f//63.5f

#define Current_Time HAL_GetTick()
/** -----------------------------
================================
   ##### Structure #####
================================
 ---------------------------- **/
 /*通用结构体*/
 //一阶低通滤波
typedef struct
{
    float preout;     //上一个输出值，用于保持滤波器状态，以便在连续调用之间维持滤波效果
    float out;        //当前输出值，即经过低通滤波处理后的信号
    float in;         //输入值，这是将要被滤波的原始信号
    float off_freq;   // 截止频率或称为权重，它决定了哪些频率成分可以通过滤波器
    float samp_tim;   //采样步长（时间），两次采样之间的时间，它对确定滤波器的时间常数至关重要
} ST_LPF;             //定义了一阶低通滤波器的结构体，一阶意味着它的传递函数有一阶多项式
 

typedef struct
{
    float fpDes; // 控制变量目标值
    float fpFB;  // 控制变量反馈值

    float fpE;    // 本次偏差
    float fpPreE; // 上次偏差
    float fpSumE; // 总偏差

    float fpKp; // 比例系数Kp
    float fpKi; // 积分系数Ki
    float fpKd; // 微分系数Kd

    float fpU; // 本次PID运算结果
    float fpUp; // 比例输出
    float fpUi; // 积分输出
    float fpUd; // 微分输出
    float fpPreUd; // 上次微分输出

    float fpUMax;  // 总输出限幅
    float fpUpMax; // 比例项输出限幅
		float fpSumEMax; //总误差限幅
    float fpUdMax; // 微分项输出上限

    float fpEMax; // 偏差限幅
    float fpEMin; // 偏差死区
} ST_PID;

typedef struct
{
    ST_PID inner;                                             
    ST_PID outer;                                              
    float output;// 串级输出，等于inner.output                            
} ST_CascadePID;


/*TD（微分跟踪器）结构体*/
typedef struct
{
    float x1;  // TD 生成的目标位置（平滑位置）
    float x2;  // TD 生成的目标速度（平滑速度）
    float x;   // 当前目标与实际目标的偏差，用于计算平滑控制
    float r;   // TD 阻尼因子，影响响应速度，越大响应越快
    float h;   // TD 滤波因子，影响系统平滑度，越大越平滑
    float T;   // TD 时间步长，影响更新频率，通常与系统采样时间相同
    float aim; // 系统期望达到的目标位置
} ST_TD;

/*滑模控制器结构体*/
typedef struct
{
		fp32 fpDes;        //控制变量目标值
		fp32 fpFB;        //控制变量反馈值
		fp32 fpE;        //本次偏差
		fp32 fpU;        //本次运算结果
		fp32 fpUMax;        //输出上限
		
		//SMC参数
		fp32 b;//转动惯量倒数
		fp32 eps;//饱和函数增益项，作用是扰动补偿
		fp32 gain;//比例增益项
		fp32 dead;//饱和函数死区，在死区内为放大环节，死区外为固定值
		ST_TD TD; //使用TD来获取微分信号
}ST_SMC; 

/*一阶LESO算法结构体*/
typedef struct
{
  float Beta01;
  float Beta02;
  float b0;

  float Z1;
  float Z2;

  float h;//积分步长
  float E;//观测误差 Z1 - Y 

  float U0;
  float U;

  float fpUMax;
}ST_LESO_1order;
/*二阶LESO算法结构体*/
typedef struct
{
  float Beta01;
  float Beta02;
  float Beta03;
  float b0;

  float Z1;
  float Z2;
  float Z3;

  float h;//积分步长
  float E1hat;//观测误差 Z1 - Y 
  float E2hat;//观测误差 Z2 - Y

  float U0;
  float U;

  float fpUMax;
}ST_LESO_2order;
/*龙伯格观测器结构体*/
typedef struct
{
	float y_Matrix[2];
	float x_Matrix_head[2];
	float x_Matrix_head_prior[2];
	float L_Matrix[2][2];
}ST_Luenberger_observer;

/**
 * @brief 摩擦力模型参数结构体
 * 
 * 适用于单个关节的摩擦力模型：τ_friction = Fc * sign(v) + B * v
 * 若需要静摩擦（Stiction），可在低速时额外处理
 */
typedef struct {
    float Fc;           /**< 库仑摩擦力矩常数 [Nm] */
    float B;            /**< 粘滞摩擦系数 [Nm/(rad/s)] */
    float velocity_threshold; /**< 速度阈值，低于此值认为接近零速，可返回0或特殊处理 */
} FRICTIONPARAM;

/**
 * @brief 重力前馈结构体，这里的前馈系数实际含义是m0gr+m1gl，r是该关节控制的机械臂质心到该关节的直线距离，m0是该关
          节控制的机械臂的质量，l是下一个关节到这个关节的直线距离，m1是该关节之后所有机械臂质量的总和。使用时给定angle0，各级前馈系数，将电机反馈的角度存储到q中，调用GravityFeedforward函数，得到OutPut_Tor;
 */
typedef struct
{
    float q[3];  /**< 关节角度 [rad] */
    float angle0[3]; //各级机械臂初始角度与水平方向的夹角，逆时针为正
    float k_Joint1; //前馈系数,最靠近基座的关节
    float k_Joint2;
    float k_Joint3;
    float Output_Tor[3];//输出前馈力矩
} GRAVITYPARAM;

/*系统自检结构体*/
typedef struct
{
    
		uint16_t Fps_Can_Receive[2];        /**< CAN 接收帧率，分别对应CAN1和CAN2 */
		uint16_t Cnt_Can_Receive[2];        /**< CAN 接收计数，分别对应CAN1和CAN2 */


        uint16_t Fps_DJI_Send;/**<对应电机CAN发送帧率 */
        uint16_t Cnt_DJI_Send;/**<对应电机CAN发送计数 */

        uint16_t FPS_DM_Send;/**<对应电机CAN发送帧率 */
        uint16_t Cnt_DM_Send;/**<对应电机CAN发送帧率 */

        uint16_t FPS_J60_Send;/**<对应电机CAN发送帧率 */
        uint16_t Cnt_J60_Send;/**<对应电机CAN发送帧率 */

        uint16_t Fps_PoleArm_Joint1_3508_Receive;/**<对应电机CAN接收帧率 */
        uint16_t Cnt_PoleArm_Joint1_3508_Receive;/**<对应电机CAN接收计数 */	

		uint16_t Fps_PoleArm_Joint2_2006_Receive;/**<对应电机CAN接收帧率 */
        uint16_t Cnt_PoleArm_Joint2_2006_Receive;/**<对应电机CAN接收计数 */

        uint16_t Fps_BlockArm_Joint3_3508_Receive;/**<对应电机CAN接收帧率 */
        uint16_t Cnt_BlockArm_Joint3_3508_Receive;/**<对应电机CAN接收计数 */

        uint16_t Fps_PoleArm_Adjustment_2006_Receive;/**<对应电机CAN接收帧率 */
        uint16_t Cnt_PoleArm_Adjustment_2006_Receive;/**<对应电机CAN接收计数 */

        uint16_t Fps_PoleArm_FrictionWheel_3508_Receive;/**<对应电机CAN接收帧率 */
        uint16_t Cnt_PoleArm_FrictionWheel_3508_Receive;/**<对应电机CAN接收计数 */

        uint16_t FPS_BlockArm_Gimbal_J60_Receive;/**<对应电机CAN接收帧率 */
        uint16_t Cnt_BlockArm_Gimbal_J60_Receive;/**<对应电机CAN接收帧率 */

        uint16_t FPS_BlockArm_Joint1_J60_Receive;/**<对应电机CAN接收帧率 */
        uint16_t Cnt_BlockArm_Joint1_J60_Receive;/**<对应电机CAN接收帧率 */

        uint16_t FPS_BlockArm_Joint2_J60_Receive;/**<对应电机CAN接收帧率 */
        uint16_t Cnt_BlockArm_Joint2_J60_Receive;/**<对应电机CAN接收帧率 */

		uint16_t FPS_AirBoard_Receive;			/**< 气动板频率 */
		uint16_t Cnt_AirBoard_Receive;			/**< 气动板计数 */
		
		uint16_t FPS_Communicate_Receive;				/**< 上下板通信频率  */
		uint16_t Cnt_Communicate_Receive;				/**< 上下板通信计数  */
		
		
}ST_SYSTEM_MONITOR;
extern ST_SYSTEM_MONITOR System_Monitor;

typedef enum
{
	RIGHT,
	MOTOR_ERROR,
	CAN_ERROR
}SYSTEM_STATE;






/*所有电机的结构体、枚举、常量*/

/*电机输入*/
typedef struct 
{
    float BlockArm_Joint3_3508;
    float BlockArm_Joint2_J60;
    float BlockArm_Joint1_J60;
    float BlockArm_Gimbal_J60;
    float PoleArm_Joint1_3508;
    float PoleArm_Joint2_2006;
    float PoleArm_FrictionWheel_3508;
    float PoleArm_Adjustment_2006;
}MOTORINPUT;

/*大疆电机结构体、枚举、常量*/
//电机码盘结构体
typedef struct {
    int siRawValue;         /**< 当前编码器原始值 */
    int siPreRawValue;      /**< 上次编码器原始值 */
    int siDiff;             /**< 编码器差值 */
    int siSumValue;         /**< 编码器累计值 */
    float siGearRatio;       /**< 齿轮比 */
    int siNumber;           /**< 编码器线数 */
    float fpSpeed;           /**< 编码器测得的速度，单位：转/分钟 */
		u8 state;								//判断初值是否为0，用于清零大疆电机初始编码器带来的角度
} ST_ENCODER;

//电机总结构体
typedef struct 
{
    ST_ENCODER motor_encoder; /**< 电机编码器信息 */
    float EncoderNum;         /**< 编码器计数值 */
    float encoder_speed;      /**< 编码器测得的速度 */
    float angle;              /**< 电机角度位置 */
    float anglev;             /**< 电机角速度 */
    float motor_current;      /**< 电机电流 */
	int16_t temp;			  /**< 电机温度 */
	
	float target_anglev;
	float target_acc;
    
    float outerTarget;        /**< 外环目标位置 */
    float outerFeedback;      /**< 外环反馈位置 */
    float innerFeedback;      /**< 内环反馈速度 */
	
	float MaxAngle;			  /**< 最大角度限位*/
	float MinAngle;           /**< 最小角度限位*/
} ST_MOTOR;

/*云深处电机结构体、枚举、常量*/
/*常量*/ 
#define CAN_ID_SHIFT_BITS 5

#define POSITION_MIN -40.0f
#define POSITION_MAX 40.0f
#define VELOCITY_MIN -40.0f
#define VELOCITY_MAX 40.0f
#define KP_MIN 0.0f
#define KP_MAX 1023.0f
#define KD_MIN 0.0f
#define KD_MAX 51.0f
#define TORQUE_MIN -40.0f
#define TORQUE_MAX 40.0f

#define GEAR_RATIO_MIN 0.0f
#define GEAR_RATIO_MAX 50.0f

#define MOTOR_TEMP_MIN -20.0f
#define MOTOR_TEMP_MAX 200.0f
#define DRIVER_TEMP_MIN -20.0f
#define DRIVER_TEMP_MAX 200.0f
#define CURRENT_MIN 0.0f
#define CURRENT_MAX 40.0f

#define SEND_POSITION_LENGTH 16
#define SEND_VELOCITY_LENGTH 14
#define SEND_KP_LENGTH 10
#define SEND_KD_LENGTH 8
#define SEND_TORQUE_LENGTH 16
#define SEND_GEAR_RATIO_LENGTH 16
#define SEND_LIMIT_CURRENT_LENGTH 16

#define RECEIVE_POSITION_LENGTH 20
#define RECEIVE_VELOCITY_LENGTH 20
#define RECEIVE_TORQUE_LENGTH 16
#define RECEIVE_TEMP_FLAG_LENGTH 1
#define RECEIVE_TEMP_LENGTH 7

#define ERROR_CODE_LENGTH 16
#define ERROR_VOLTAGE_LENGTH 16
#define ERROR_CURRENT_LENGTH 16
#define ERROR_MOTOR_TEMP_LENGTH 8
#define ERROR_DRIVER_TEMP_LENGTH 8



// Commands即电机的cmd
#define DISABLE_MOTOR 1       //失能
#define ENABLE_MOTOR 2        //使能
#define CALIBRATE_START 3
#define CONTROL_MOTOR 4       //控制
#define RESET_MOTOR 5
#define SET_HOME 6
#define SET_GEAR 7
#define SET_ID 8
#define SET_CAN_TIMEOUT 9
#define SET_BANDWIDTH 10
#define SET_LIMIT_CURRENT 11
#define SET_UNDER_VOLTAGE 12
#define SET_OVER_VOLTAGE 13
#define SET_MOTOR_TEMPERATURE 14
#define SET_DRIVE_TEMPERATURE 15
#define SAVE_CONFIG 16
#define ERROR_RESET 17
#define WRITE_APP_BACK_START 18
#define WRITE_APP_BACK 19
#define CHECK_APP_BACK 20
#define DFU_START 21
#define GET_FW_VERSION 22
#define GET_STATUS_WORD 23
#define GET_CONFIG 24
#define CALIB_REPORT 31



// SendDLC
#define SEND_DLC_DISABLE_MOTOR 0        //失能
#define SEND_DLC_ENABLE_MOTOR 0         //使能
#define SEND_DLC_CALIBRATE_START 0
#define SEND_DLC_CONTROL_MOTOR 8        //控制
#define SEND_DLC_RESET_MOTOR 0
#define SEND_DLC_SET_HOME 0
#define SEND_DLC_SET_GEAR 2
#define SEND_DLC_SET_ID 1
#define SEND_DLC_SET_CAN_TIMEOUT 1
#define SEND_DLC_SET_BANDWIDTH 2
#define SEND_DLC_SET_LIMIT_CURRENT 2
#define SEND_DLC_SET_UNDER_VOLTAGE 2
#define SEND_DLC_SET_OVER_VOLTAGE 2
#define SEND_DLC_SET_MOTOR_TEMPERATURE 2
#define SEND_DLC_SET_DRIVE_TEMPERATURE 2
#define SEND_DLC_SAVE_CONFIG 0
#define SEND_DLC_ERROR_RESET 0
#define SEND_DLC_WRITE_APP_BACK_START 0
#define SEND_DLC_WRITE_APP_BACK 8
#define SEND_DLC_CHECK_APP_BACK 8
#define SEND_DLC_DFU_START 0
#define SEND_DLC_GET_FW_VERSION 0
#define SEND_DLC_GET_STATUS_WORD 0
#define SEND_DLC_GET_CONFIG 0
#define SEND_DLC_CALIB_REPORT 8



// ReceiveDLC
#define RECEIVE_DLC_DISABLE_MOTOR 1         //失能
#define RECEIVE_DLC_ENABLE_MOTOR 1          //使能
#define RECEIVE_DLC_CALIBRATE_START 1
#define RECEIVE_DLC_CONTROL_MOTOR 8          //控制
#define RECEIVE_DLC_RESET_MOTOR 1
#define RECEIVE_DLC_SET_HOME 1
#define RECEIVE_DLC_SET_GEAR 2
#define RECEIVE_DLC_SET_ID 1
#define RECEIVE_DLC_SET_CAN_TIMEOUT 1
#define RECEIVE_DLC_SET_BANDWIDTH 1
#define RECEIVE_DLC_SET_LIMIT_CURRENT 1
#define RECEIVE_DLC_SET_UNDER_VOLTAGE 1
#define RECEIVE_DLC_SET_OVER_VOLTAGE 1
#define RECEIVE_DLC_SET_MOTOR_TEMPERATURE 1
#define RECEIVE_DLC_SET_DRIVE_TEMPERATURE 1
#define RECEIVE_DLC_SAVE_CONFIG 1
#define RECEIVE_DLC_ERROR_RESET 1
#define RECEIVE_DLC_WRITE_APP_BACK_START 1
#define RECEIVE_DLC_WRITE_APP_BACK 1
#define RECEIVE_DLC_CHECK_APP_BACK 2
#define RECEIVE_DLC_DFU_START 1
#define RECEIVE_DLC_GET_FW_VERSION 2
#define RECEIVE_DLC_GET_STATUE_WORD 5
#define RECEIVE_DLC_GET_CONFIG 8
#define RECEIVE_DLC_CALIB_REPORT 8

//返回值转换
#define YSC_MAX_RAW_VALUE 1048575u    // 2^20 - 1
#define YSC_RANGE 80.0f               // 位置和速度的范围 80 (-40到40)
#define YSC_OFFSET -40.0f             // 最小值 -40
//#define YSC_Gear_Ratio 18.36f

#define YSC_TORQUE_RANGE 80.0f        // 力矩范围 80 (-40到40Nm)
#define YSC_TORQUE_OFFSET -40.0f      // 力矩最小值 -40Nm
#define YSC_TORQUE_MAX_RAW_VALUE 65535u // 2^16 - 1

#define YSC_TEMP_RANGE 220.0f         // 温度范围 220 (-20到200度)
#define YSC_TEMP_OFFSET -20.0f        // 最低温度 -20度
#define YSC_TEMP_MAX_RAW_VALUE 127u   // 2^7 - 1

typedef struct
{
    uint8_t motor_id_;
    uint8_t cmd_;
    float position_;
    float velocity_;
    float torque_;
    float kp_;
    float kd_;
}MotorCMD; //电机结构体

typedef struct
{
    uint8_t motor_id_;
    uint8_t cmd_;
	
    float position_;
    float velocity_;
    float torque_;
    bool flag_;
    float temp_;
    uint16_t error_;
    uint8_t state;
}MotorDATA;//用于存储电机返回的数据

#pragma pack(push, 1) // 设置字节对齐
typedef union ReceivedMotionData
{
    uint8_t data[8];
    struct
    {
        uint32_t position : RECEIVE_POSITION_LENGTH;
        uint32_t velocity : RECEIVE_VELOCITY_LENGTH;
        uint32_t torque : RECEIVE_TORQUE_LENGTH;
        uint32_t temp_flag : RECEIVE_TEMP_FLAG_LENGTH;
        uint32_t temperature : RECEIVE_TEMP_LENGTH;
    }fields;
}ReceivedMotionData;
#pragma pack(pop) // 恢复默认字节对齐

typedef enum 
{
    DISABLE_STATE,//失能状态
    ENABLE_STATE//使能状态
}J60_ENABLE_STATE;

/*达妙电机结构体、枚举、常量*/
typedef struct {
    float p_des;    
    float v_des;   
    float kp;      
    float kd;      
    float t_fi;     
}DM_CMD;

typedef struct {
    uint8_t id;         
    uint8_t err;         
    float pos;          
    float vel;         
    float torque;       
    float t_mos;        
    float t_rotor;      
}DM_DATA;

enum Temp_Flag{
    kDriverTempFlag=0,
    kMotorTempFlag=1
};
#pragma pack(push, 1) // 设置字节对齐

typedef struct
{
	uint32_t position ;
  uint32_t velocity ;
  uint32_t torque ;
  uint32_t temp_flag ;
	uint32_t temperature ;
}ReceivedData;

/*常量*/ 
#define FEEDBACK_POSITION_MIN   -12.5f
#define FEEDBACK_POSITION_MAX   12.5f
#define FEEDBACK_VELOCITY_MIN   -45.0f
#define FEEDBACK_VELOCITY_MAX   45.0f
#define FEEDBACK_TORQUE_MIN     -18.0f
#define FEEDBACK_TORQUE_MAX     18.0f

#define FEEDBACK_POSITION_LENGTH  16
#define FEEDBACK_VELOCITY_LENGTH  12
#define FEEDBACK_TORQUE_LENGTH    12

#define DM_SEND_POSITION_LENGTH  16
#define DM_SEND_VELOCITY_LENGTH  12
#define DM_SEND_KP_LENGTH        12
#define DM_SEND_KD_LENGTH        12
#define DM_SEND_TORQUE_LENGTH    12
#define DM_POSITION_MIN   -12.5f     
#define DM_POSITION_MAX   12.5f     
#define DM_VELOCITY_MIN   -45.0f     
#define DM_VELOCITY_MAX   45.0f      
#define DM_KP_MIN         0.0f       
#define DM_KP_MAX         500.0f     
#define DM_KD_MIN         0.0f      
#define DM_KD_MAX         5.0f       
#define DM_TORQUE_MIN     -18.0f   
#define DM_TORQUE_MAX     18.0f      




/*遥控器结构体*/
typedef struct
{
    uint16_t usJsKey;
    uint16_t indepen_usJsKey[8];
    uint16_t usJsLeft_X; // 左摇杆x方向
    uint16_t usJsLeft_Y; // 左摇杆y方向
    uint16_t usJsRight_X;  // 右摇杆x方向
    uint16_t usJsRight_Y;  // 右摇杆y方向
} ST_JS_VALUE;	


typedef enum{
    CARTESIAN,//笛卡尔坐标系
    POLAR    //极坐标系
}COORDINATE;

 //向量结构体 一般用作函数里局部变量
typedef struct
{
    //笛卡尔直角坐标系
    float fpX;     // X方向差
    float fpY;     // Y方向差
    float fpW;      // 旋转速度  叉乘运算？待定
    
    //极坐标系
    float fpLength; // 向量长度（单位mm）
    float fpThetha; // 向量与X轴角度（单位:弧度）
	  COORDINATE type;  //坐标系类型
} ST_VECTOR;

/*定位结构体（机器人整车ST_ROBOT）*/
// 坐标结构体
typedef struct
{
    float fpPosX;  // 横坐标X（单位：mm）
    float fpPosY;  // 竖坐标Y（单位：mm）
    float fpPosQ;  // 航向角Q（单位：0.1度）
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
    ST_POS stPos;     // 机器人中心坐标姿态
    ST_VEL stVelt_global;   // 机器人中心在全场坐标系下的速度
    ST_VEL stVelt_local;        // 机器人中心在局部坐标系下的速度
} ST_ROBOT;


/*随动轮*/
// 随动轮码盘过线计数及随动轮相关结构体
typedef struct
{
    int siCoderACur; // 当前码盘A读数 编码器的值
    int siCoderAPre; // 上一次码盘A读数，判断随动轮旋转方向
    int siCoderBCur; // 当前码盘B读数
    int siCoderBPre; // 上一次码盘B读数
    
	ST_POS stPos;    // 随动轮中心坐标姿态
} ST_FOLLOWER_WHEEL;

/*陀螺仪*/
typedef struct
{
    float fpQ_Cur;     // 陀螺当前数据读数
    float fpQ_Pre;     // 陀螺上一次数据读数，判断旋转方向
} ST_GYRO;

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
    float dt35_voltage_x1, dt35_voltage_x2, dt35_voltage_y1, dt35_voltage_y2;                 // can接收 电压模拟量
    float dt35_x1, dt35_x2, dt35_y1, dt35_y2; // 在locate_algorithm.c里DT35到墙的距离
    float robot_x, robot_y;//机器人中心到墙的距离，在locate_algorithm.c里加上
    float robot_q;//机器人中心姿态角
} ST_DT35;



/*导航结构体（包含）*/
typedef struct
{
	float V_max;   // 最大速度
    float A_up;   // 加速度
    float A_down; // 减速度
} ST_Motion_Path;

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
	ST_Nav_Pid	pos_pid;
    ST_VEL basic_velt;   // 自动路径规划出的目标速度
    uint32_t run_time;         // 运行时间
    uint32_t run_Sumtime;      // 总运行时间
	uint32_t rotation_time;
	uint8_t number;				//自动路径选择第几个
	uint8_t number_permutation;//路径组合选择第几个
} ST_Auto_Path;

//导航标志位 ，自己全部重新写
typedef enum
{
	NAV_INIT,							// 初始化
    NAV_OFF,              // 四轮无输出，卸力
	NAV_LOCK,							// 坐标锁死
    NAV_MANUAL,           // 局部坐标系手操
    NAV_GLOBAL_MANUAL,    // 锁YAW手操
	NAV_AIM_MANUAL, 			// 自瞄模式手操
    NAV_AUTO_PATH,        // 自动路径导航
    NAV_PERMUTATION_PATH, // 组合路径导航
	NAV_VISION_MANUAL,		// 视觉锁YAW手操
	NAV_VISION_AIM_MANUAL,// 视觉自瞄手操
	NAV_VISION_PATH				// 视觉路径导航
} Nav_State; 

// 整个导航系统的主结构体
typedef struct
{
    Nav_State nav_state;//导航系统状态
	ST_VECTOR expect_robot_global_velt;//导航
	ST_Auto_Path auto_path;//自动路径总结构体
} ST_Nav; // 定位 导航共用

//pid状态标志位
typedef enum
{
	VELT_LOOP,//单环 无位置环纠偏
	OPEN_LOOP,//开环控制 不进行pid计算
	DOUBLE_LOOP//双环 
}Pid_State;

//前馈标志位
typedef enum
{
    WITHOUT_FORWARD,//没有前馈
    WITH_FORWARD//有前馈
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

/** -----------------------------
================================
  ##### Extern Viariables #####
================================
 ---------------------------- **/

typedef enum
{
	PATH_INIT,
	PATH_END,
	PATH_ONGOING,
	PATH_FREE
}Path_State;

typedef struct
{
	float x;
	float y;
	float q;
}POINT;

typedef enum
{
	LINE, //直线
	CIRCLE,   //圆弧
	BEZIER 	  //贝塞尔曲线
}PATH_TYPE;

typedef struct
{
	ST_VECTOR Point_Start[10], Point_Inc[10], V_Start[10], V_End[10];//依次存储每段路径的起点坐标，增量坐标，起始速度，结束速度
	fp32 Rotation_Start[10],Rotation_Inc[10],W_Start[10],W_End[10];//没有用到
	PATH_TYPE Path_Type[10];//存储路径类型，直线和圆弧
	fp32 A[10],A_W[10];//存储加速度和角加速度
	fp32 R[10]; //存储圆弧的半径
	fp32 T[10], T_W[10];//存储每段路径的时间
	u8 num_module, num_module_w;//存储路径的数量
	//A_W、T_W和num_module_w都会用在NavRotation(）函数中
	
} PATH_PERMUTATION;

/*板间通信数据包结构体*/
#define UART1_TX_DATA_LEN 23
#define UART1_RX_DATA_LEN 21
typedef struct
{
	uint8_t start1;	//帧头1
	uint8_t start2;	//帧头2
	uint8_t datanum;
	
	u8 num[UART1_TX_DATA_LEN];		//待发送数据
	
	uint8_t tail1;	//帧尾1
	uint8_t tail2;	//帧尾2
	
}uart1_tx_protocol_t;//发送信息报文结构体

typedef struct
{
	uint8_t start1;  //帧头1
	uint8_t start2;  //帧头2
	uint8_t datanum;
	
	u8 num[UART1_RX_DATA_LEN];	//待接收数据
	
	uint8_t tail1;   //帧尾1
	uint8_t tail2;   //帧尾2
	
}uart1_rx_protocol_t;//接收信息报文结构体



#define POSITION_MIN -40.0f
#define POSITION_MAX 40.0f
#define VELOCITY_MIN -40.0f
#define VELOCITY_MAX 40.0f
#define KP_MIN 0.0f
#define KP_MAX 1023.0f
#define KD_MIN 0.0f
#define KD_MAX 51.0f
#define TORQUE_MIN -40.0f
#define TORQUE_MAX 40.0f

#define GEAR_RATIO_MIN 0.0f
#define GEAR_RATIO_MAX 50.0f

#define MOTOR_TEMP_MIN -20.0f
#define MOTOR_TEMP_MAX 200.0f
#define DRIVER_TEMP_MIN -20.0f
#define DRIVER_TEMP_MAX 200.0f
#define CURRENT_MIN 0.0f
#define CURRENT_MAX 40.0f

#define SEND_POSITION_LENGTH 16
#define SEND_VELOCITY_LENGTH 14
#define SEND_KP_LENGTH 10
#define SEND_KD_LENGTH 8
#define SEND_TORQUE_LENGTH 16
#define SEND_GEAR_RATIO_LENGTH 16
#define SEND_LIMIT_CURRENT_LENGTH 16

#define RECEIVE_POSITION_LENGTH 20
#define RECEIVE_VELOCITY_LENGTH 20
#define RECEIVE_TORQUE_LENGTH 16
#define RECEIVE_TEMP_FLAG_LENGTH 1
#define RECEIVE_TEMP_LENGTH 7

#define ERROR_CODE_LENGTH 16
#define ERROR_VOLTAGE_LENGTH 16
#define ERROR_CURRENT_LENGTH 16
#define ERROR_MOTOR_TEMP_LENGTH 8
#define ERROR_DRIVER_TEMP_LENGTH 8




/*随动轮*/


/*dt35*/
//bsp_can.c回调函数里使用
extern uint16_t dt35_distance[5];

extern uint16_t dt35_x1, dt35_y1;
extern uint16_t dt35_x2, dt35_y2;

//locate_algorithm.c里使用


/*定位*/


/*舵机宏定义*/
#define Servo_Num 2

/* SCS舵机串口通信发送结构体 */ //usart_protocol.c里定义为静态变量
typedef struct
{
    uint8_t Head[2];    // 帧头（通常为固定值，例如 0xFF 0xFF）
    uint8_t ID;         // 目标舵机的 ID
    
    uint8_t Length;     // 数据包的总长度（包含 Cmd、Param 和 Sum），即param＋2
    
    uint8_t Cmd;        // 指令码（用于控制舵机的不同操作，0x02读 0x03写）
    uint8_t Param[50];  // 例如舵机角度、速度等数据。
    uint8_t Sum;        // 校验和（用于校验数据包的完整性）
    
} SCS_Buf_TypeDef;

/*舵机结构体*/
typedef struct
{
		struct{
		int16_t Error[Servo_Num];             // 记录每个舵机的误差值（可能为负）
		uint16_t Stab_MaxTime;                // 最小稳定时间要求，必须超过此时间才认为稳定
		uint8_t Stab[Servo_Num];              // 标记舵机是否已稳定（1：稳定，0：未稳定）
		uint8_t StabDomain[Servo_Num];        //允许的稳定误差范围
		}Servo_Stab;

		/*目标值*/
		uint16_t Ref_Position[Servo_Num];/*必要*///期望位置 
		uint16_t Time[Servo_Num];        /*必要*///期望时间
		uint16_t Ref_Speed[Servo_Num];   /*必要*///期望速度 
		//期望速度和期望时间之间有且只有一个0
		//即期望时间为0，期望速度不为0、期望速度为0，则期望时间不为0

		/*反馈值*/
		uint16_t Speed[Servo_Num];       //反馈速度
		uint16_t Position[Servo_Num];    /*必要*///反馈位置

		uint8_t  ID[Servo_Num];          /*必要*///舵机ID，终端执行器夹爪ID为1，后续逐渐增加 
		uint8_t  stable_state;
}Servo;

typedef enum
{
	LOAD_BALL,
	DRIB_BALL
}SERVO_STATE;

typedef struct
{
	float radar_x;//雷达定位X
	float radar_y;//雷达定位Y
	float radar_q;//雷达定位Q
	float nav_pos_x;//路径规划X方向坐标
	float nav_pos_y;//路径规划Y方向坐标
	float nav_v_x;//路径规划X方向速度
	float nav_v_y;//路径规划Y方向速度
	float nav_pos_q;//路径规划YAW坐标
	u8 nav_flag;//路径规划开始标志位
	float aim_chassis_yaw; //雷达自瞄底盘YAW轴
	float aim_gimbal_yaw;	 //雷达自瞄云台YWA轴
	float aim_gimbal_pitch;//雷达自瞄云台PITCH
	float aim_shoot_speed; //雷达自瞄发射速度
	float joy_v_y;//视觉遥控器摇杆X速度
	float joy_v_x;//视觉遥控器摇杆Y速度
	float joy_v_w;//视觉遥控器摇杆YAW速度
	u8 botton_a;//视觉遥控器键值
	u8 botton_b;
	u8 botton_x;
	u8 botton_y;
	u8 botton_left_shoulder;
	u8 botton_right_shoulder;
	u8 botton_minus;
	u8 botton_plus;
	u8 botton_hat_up;
	u8 botton_hat_down;
	u8 botton_hat_left;
	u8 botton_hat_right;
}ST_VISION_DATA;

//extern ST_CascadePID Chassis_Global_Yaw_Pid;

typedef struct {
    float v;          // 状态量 (速度估计)
    float P;          // 状态协方差
    float Q;          // 过程噪声
    float R_ins;      // 惯导观测噪声
    float R_whl_base; // 轮速基础观测噪声
    float slip_thres; // 打滑判断阈值 (mm/s)
    float slip_scale; // 打滑时噪声放大倍数
} KalmanFilter;



/*********************************************************************************************
四轮编码器返回值进行滤波，滤除其高频抖动
**********************************************************************************************/
typedef struct
{
	ST_LPF leftup_velt;
	ST_LPF rightup_velt;
	ST_LPF rightdown_velt;
	ST_LPF leftdown_velt;
}ST_WHEEL_ENCODER_VELT_FILTER;

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


typedef struct
{
	ST_LPF accel_x_filter;
	ST_LPF accel_y_filter;
}ST_BODY_ACCEL;

typedef enum
{
	NORMAL_RC_OPENED,//连正常硬件遥控器
	VISION_RC_OPENED,//连视觉遥控器
	ALL_LOCKED//都没连上，锁住其他任务
}RC_MODE;//遥控器使用模式

typedef enum
{
	CONNECT,
	DISCONNECT
}CONNECT_STATUS;
//遥控器大结构体，包括两个遥控器的连接情况和决策

typedef struct
{
	CONNECT_STATUS normal_rc_status;
	CONNECT_STATUS vision_rc_status;
	int normal_rc_disconnect_cnt;
	RC_MODE rc_mode;
}ST_RC_CTRL;


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
	bool x_sucpect,y_sucpect;//失信度，失信为1，可信为0
	float dt35_voltage_x1,dt35_voltage_y1,dt35_voltage_y2;
	float dt35_x1,dt35_x2,dt35_y1,dt35_y2;
	float dt35_x1_save[5],dt35_x2_save[5],dt35_y1_save[5],dt35_y2_save[5];
	float dt35_x1_save_sum,dt35_x2_save_sum,dt35_y1_save_sum,dt35_y2_save_sum;
	float dt35_robot_x,dt35_robot_y;
}ST_DT35_NEW;

#define CH_COUNT 20
typedef struct Frame {
    float fdata[CH_COUNT];
    unsigned char tail[4];
}VOFA;

extern float theta1_cand[2];
extern float d;
extern float test[6];

/*气动板*/
#define QD_FC 0x2
#define QD_RC 0x1
#define QD_OFF 0x0

extern uint8_t AirCtrl[3];
extern uint8_t DT35_Data[8];//从气动板接收，通过串口一发送给下层板
extern uint16_t DT35_Digital_Data[4];
extern uint8_t AirOperaterCtrlBuf[12];
typedef struct 
{
    uint8_t PoleArm_Upper_Solenoid_Valve_State;//取杆电磁阀状态，0表示关闭，1表示开启
    uint8_t PoleArm_Lower_Solenoid_Valve_State;//存杆电磁阀状态，0表示关闭，1表示开启
    uint8_t BlockArm_Fetch_Air_Pump_State;//取块机械臂气泵状态，0表示关闭，1表示开启
    uint8_t BlockArm_Fetch_Solenoid_Valve_State;//取块机械臂电磁阀状态，0表示关闭，1表示开启
    uint8_t BlockArm_Store1_Air_Pump_State;//车侧边存块气泵状态，0表示关闭，1表示开启
    uint8_t BlockArm_Store0_Air_Pump_State;//车内部存块气泵状态，0表示关闭，1表示开启
    uint8_t BlockArm_Store1_Solenoid_Valve_State;//车侧边存块电磁阀状态，0表示关闭，1表示开启
    uint8_t BlockArm_Store0_Solenoid_Valve_State;//车内部存块电磁阀状态，0表示关闭，1表示开启
}QD_CTRL;
extern QD_CTRL QD_Ctrl;//气动控制结构体


/*CAN1通信接收数据*/
extern uint8_t RxMsg_CAN1[8];
/*CAN2通信接收数据*/
extern uint8_t RxMsg_CAN2[8];
/*串口通信数据*/
#define LEFT_JS_X_MID 0x084C
#define LEFT_JS_X_MAX 0x0FDD
#define LEFT_JS_X_MIN 0x0010 
#define LEFT_JS_Y_MID 0x07F1 
#define LEFT_JS_Y_MAX 0x0F25
#define LEFT_JS_Y_MIN 0x0025
#define RIGHT_JS_MID 0x07DA 
#define RIGHT_JS_MIN 0x0005
#define RIGHT_JS_MAX 0x0FF6//遥控器摇杆常量
extern uint8_t RxMsg_USART1[32];
extern ST_JS_VALUE Communicate_Js_Value;//遥控器摇杆数据结构体变量
extern uint8_t Communicate_KEY;//通信任务的按键变量
/*串口接收字节数*/
extern uint16_t Receive_length; 
/*电机输入*/
extern MOTORINPUT MotorInput;
/*大疆电机发送电流*/
extern int16_t Current1_4[4];
extern int16_t Current5_8[4];
/********************************************************************************************
存取块机械臂的各种变量：存取块状态机枚举、电机变量
*********************************************************************************************/
#define BlockArm_Gimbal_J60_Init_Position -4.f //存取块机械臂控制云台的达妙电机初始化位置

#define BlockArm_Joint3_uiGearRatio 2.18678f  //存取杆机械臂第一个关节的齿轮的减速比

#define BlockArm_Joint3_GfNoBlock 250
#define BlockArm_Joint3_GfWithBlock 3000


//机械臂运动解算结构体
typedef struct 
{
    float L1;          // 第一节机械臂的长度/mm
    float L2;          // 第二节机械臂的长度/mm
    float x0;          // 初始状态第二节机械臂端点水平坐标/mm
    float y0;          // 初始状态第二节机械臂端点竖直坐标/mm
    float theta1_0;    // 初始状态第一节机械臂与水平方向的夹角，逆时针为正/°
    float theta2_0;    // 初始状态第二节机械臂与水平方向的夹角，逆时针为正/°
    float theta3_0;    // 初始状态第三节机械臂与水平方向的夹角，恒为0或-90/°
    
    float x;           // 当前时刻第二节机械臂端点水平坐标/mm
    float y;           // 当前时刻第二节机械臂端点竖直坐标/mm
    float dx;          // 当前时刻末端x方向速度 (mm/s)
    float dy;          // 当前时刻末端y方向速度 (mm/s)
    float ddx;         // 当前时刻末端x方向加速度 (mm/s?)
    float ddy;         // 当前时刻末端y方向加速度 (mm/s?)
    float theta1;      // 当前时刻第一节机械臂与水平方向的夹角/°
    float theta2;      // 当前时刻第二节机械臂与水平方向的夹角/°
    float theta3;      // 当前时刻第三节机械臂与水平方向的夹角/°
    float dtheta1;     // 当前时刻第一节机械臂角速度 (°/s)
    float dtheta2;     // 当前时刻第二节机械臂角速度 (°/s)
    float ddtheta1;    // 当前时刻第一节机械臂角加速度 (°/s?)
    float ddtheta2;    // 当前时刻第二节机械臂角加速度 (°/s?)
} ARM_BACKSOLVING;
extern ARM_BACKSOLVING Arm_BackSolving;//机械臂运动解算结构体

// 云台控制器结构体
typedef struct {
    float ff_gain_1;   // 电机1到yaw轴的映射系数
    float ff_gain_2;   // 电机2到yaw轴的映射系数
    float t_ff;       // 前馈补偿力矩 (A)
} YawController_t;
extern YawController_t Yaw_ff;

typedef struct 
{
    uint8_t First_Init;//第一次初始化
    uint8_t Ready;//1表示收到信息,准备执行任务要存的位置
    uint8_t Block_Height;//方块高度，0，1，2，3对应地，200mm，400mm，600mm
	uint8_t BlockStore_State;//这次存方块要存的位置
    uint8_t BlockStore[3];//第几个存块位置是否有存块，[0]对应车体内的存块位置[1]表示车侧边的存块位置[2]表示存在机械臂上
    uint8_t BlockPut_Num;//存第几个方块,1,2,3
    uint8_t Area_State;//所处区域的标志位，1，2，3对应一区二区三区
	uint8_t Reach_Area3;
}BLOCKARM_TASK_CHANGE;
extern BLOCKARM_TASK_CHANGE BlockArm_Task_Change;//存取块机械臂任务状态切换标志

typedef enum 
{
    BLOCKARM_TASK_INIT = 0,
    BLOCKARM_TASK_MOVE_TO_FETCH_POSITION,//移动到取块位置
    BLOCKARM_TASK_FETCH_BLOCK,//吸取方块
    BLOCKARM_TASK_RETRACT_ARM,//收回机械臂，到达存块位置
    BLOCKARM_TASK_STORAGE_BLOCK,//存储方块
    BLOCKARM_TASK_PICKUP_BLOCK,//三区拿起存储的方块
    BLOCKARM_TASK_PUT_BLOCK//三区放置方块
}BLOCKARM_TASK_STATE;

extern BLOCKARM_TASK_STATE BlockArm_Task_State;//存取块机械臂任务状态机变量
extern BLOCKARM_TASK_STATE BlockArm_Task_StatePre;//存取块机械臂任务状态机变量

extern ST_MOTOR BlockArm_Joint3_3508;//存取块机械臂第三个关节的3508电机,ID4

extern MotorCMD BlockArm_Joint2_J60CMD; //存取块机械臂第二个关节的J60电机
extern MotorDATA BlockArm_Joint2_J60Data; //存取块机械臂第二个关节的J60电机
extern MotorCMD BlockArm_Joint1_J60CMD; //存取块机械臂第一个关节（靠近基座）的J60电机
extern MotorDATA BlockArm_Joint1_J60Data; //存取块机械臂第一个关节（靠近基座）的J60电机
extern float YSC_Tor[3]; //J60MIT控制参数，调整后利用SetMotorCMD函数修改MotorCMD中的对应参数
extern float YSC_Pos[3]; //J60MIT控制参数，调整后利用SetMotorCMD函数修改MotorCMD中的对应参数
extern float YSC_Vel[3]; //J60MIT控制参数，调整后利用SetMotorCMD函数修改MotorCMD中的对应参数
extern J60_ENABLE_STATE J60_Enable_State[3];//J60电机的状态

extern MotorCMD BlockArm_Gimbal_J60CMD; //控制云台的J60电机,ID3
extern MotorDATA BlockArm_Gimbal_J60Data; //控制云台的J60电机,ID3

extern ST_CascadePID BlockArm_Joint3_3508_PID;
extern ST_CascadePID BlockArm_Joint2_J60_PID;
extern ST_CascadePID BlockArm_Joint1_J60_PID;
/********************************************************************************************
存取杆机械臂的各种变量与常量：存取块状态机枚举、电机变量
*********************************************************************************************/
#define PoleArm_Joint1_uiGearRatio 4.f  //存取杆机械臂第一个关节的齿轮的减速比
#define PoleArm_Joint2_uiGearRatio 2.f  //存取杆机械臂第一个关节的齿轮的减速比

extern float PoleArm_Joint1_3508_Init_Position; //存取杆机械臂第一个关节的3508电机初始化位置
extern float PoleArm_Joint1_3508_Fetch_Position; //存取杆机械臂第一个关节的3508电机取杆位置
extern float PoleArm_Joint1_3508_Place_Position; //存取杆机械臂第一个关节的3508电机放杆位置
extern float PoleArm_Joint1_3508_Connect_Position; //存取杆机械臂第一个关节的3508电机连接位置
extern float PoleArm_Joint1_3508_Store_Position; //存取杆机械臂第一个关节的3508电机存储位置
extern float PoleArm_Joint1_3508_Error_Position; //存取杆机械臂第一个关节的3508电机初始化位置

extern float PoleArm_Joint2_2006_Init_Position; //存取杆机械臂第二个关节的2006电机初始化位置
extern float PoleArm_Joint2_2006_Fetch_Position; //存取杆机械臂第二个关节的2006电机取杆位置
extern float PoleArm_Joint2_2006_Place_Position; //存取杆机械臂第二个关节的2006电机放杆位置
extern float PoleArm_Joint2_2006_Connect_Position; //存取杆机械臂第二个关节的2006电机连接位置
extern float PoleArm_Joint2_2006_Store_Position; //存取杆机械臂第二个关节的2006电机存储位置
extern float PoleArm_Joint2_2006_Error_Position; //存取杆机械臂第二个关节的2006电机初始化位置

extern float PoleArm_FrictionWheel_3508_Init_Position; //驱动摩擦轮的3508电机初始化速度
extern float PoleArm_FrictionWheel_3508_Fetch_Position; //驱动摩擦轮的3508电机取杆速度
extern float PoleArm_FrictionWheel_3508_Place_Position; //驱动摩擦轮的3508电机放杆速度
extern float PoleArm_FrictionWheel_3508_Connect_Position; //驱动摩擦轮的3508电机连接速度
extern float PoleArm_FrictionWheel_3508_Store_Position; //驱动摩擦轮的3508电机存储速度
extern float PoleArm_FrictionWheel_3508_Manual_Position; //驱动摩擦轮的3508电机手动上限速度

typedef enum
{
    POLEARM_TASK_INIT = 0,

    POLEARM_TASK_MOVE_TO_FETCH_POSITION,//移动到取杆位置
    POLEARM_TASK_ALREADY_TO_FETCH_POSITION,//已经到达取杆位置

    POLEARM_TASK_MOVE_TO_PLACE_POSITION,//移动到放杆位置
    POLEARM_TASK_ALREADY_TO_PLACE_POSITION,//已经到达放杆位置

    POLEARM_TASK_ALREADY_TO_CONNECT_POSITION,//已经到达连接位置

    POLEARM_TASK_ALREADY_TO_STORE_POSITION,//已经到达存储位置

    POLEARM_TASK_MANUAL_CONTROL,//手操控制

    POLEARM_TASK_ERROR//任务异常
} POLEARM_TASK_STATE;
extern POLEARM_TASK_STATE PoleArm_Task_State;//存取杆机械臂任务状态机变量
extern POLEARM_TASK_STATE PoleArm_Task_State_Pre;//存取杆机械臂任务状态机上一个状态

#define MAX_POLEARM_TASK_PERIOD 4000 //存取杆机械臂任务最大执行周期，单位为毫秒
typedef struct
{   
    uint32_t PoleArm_Task_Ticks;//存取杆机械臂任务定时器变量

    uint32_t POLEARM_TASK_MOVE_TO_FETCH_POSITION;
    uint32_t POLEARM_TASK_ALREADY_TO_FETCH_POSITION;

    uint32_t POLEARM_TASK_MOVE_TO_PLACE_POSITION;
    uint32_t POLEARM_TASK_ALREADY_TO_PLACE_POSITION;

    uint32_t POLEARM_TASK_MOVE_TO_CONNECT_POSITION;
    uint32_t POLEARM_TASK_ALREADY_TO_CONNECT_POSITION;

    uint32_t POLEARM_TASK_MOVE_TO_STORE_POSITION;
    uint32_t POLEARM_TASK_ALREADY_TO_STORE_POSITION;

    uint32_t POLEARM_TASK_ERROR;
}POLEARM_TASK_TIMER;
extern POLEARM_TASK_TIMER PoleArm_Task_Timer;//存取杆机械臂任务定时器变量

typedef struct 
{
    uint8_t Fetch_Pole;
    uint8_t Place_Pole;
    uint8_t Connect_Pole;
    uint8_t Store_Pole;
}POLEARM_TASK_CHANGE;
extern POLEARM_TASK_CHANGE PoleArm_Task_Change;

extern POLEARM_TASK_STATE PoleArm_Task_State;//存取杆机械臂任务状态机变量

extern ST_TD PoleArm_Joint1_3508_TD;//存取杆机械臂第一个关节的3508电机的跟随目标数据结构体
extern ST_TD PoleArm_Joint2_2006_TD;//存取杆机械臂第二个关节的2006电机的跟随目标数据结构体
extern ST_TD PoleArm_FrictionWheel_3508_TD;//驱动摩擦轮的3508电机的跟随目标数据结构体

extern ST_MOTOR PoleArm_Joint1_3508;//存取杆机械臂第一个关节（靠近基座）的3508电机,ID2
extern ST_MOTOR PoleArm_Joint2_2006;//存取杆机械臂第二个关节的2006电机,ID1
extern ST_MOTOR PoleArm_FrictionWheel_3508;//驱动摩擦轮的3508电机,ID3
extern ST_MOTOR PoleArm_Adjustment_2006;//用于对接微调的电机,ID5

extern ST_CascadePID PoleArm_Joint1_3508_PID;
extern ST_CascadePID PoleArm_Joint2_2006_PID;
extern ST_CascadePID PoleArm_FrictionWheel_3508_PID;
extern ST_CascadePID PoleArm_Adjustment_2006_PID;
extern ST_PID PoleArm_Adjustment_PID;

extern ST_LPF PoleArm_FrictionWheel_3508_LFP;

/*前馈变量与常量*/
#define MAX_DMFrictionFeedForward 0.1f
#define MAX_DJIFrictionFeedForward 300.f
#define MAX_J60FrictionFeedForward 0.1f

extern GRAVITYPARAM Gravity_PoleArm;
extern FRICTIONPARAM Friction_PoleArm_Joint1;
extern FRICTIONPARAM Friction_PoleArm_Joint2;
extern FRICTIONPARAM Friction_PoleArm_FrictionWheel;
extern FRICTIONPARAM Friction_PoleArm_Adjustment;

extern GRAVITYPARAM Gravity_BlockArm;
extern FRICTIONPARAM Friction_BlockArm_Joint1;
extern FRICTIONPARAM Friction_BlockArm_Joint2;
extern FRICTIONPARAM Friction_BlockArm_Joint3;
extern FRICTIONPARAM Friction_BlockArm_Gimbal;

/*运动规划*/
// 运动规划器（已知位移、最大速度、总时间，自动计算加速度）
typedef struct {
    float q0;           // 初始位置 (°)
    float q1;           // 目标位置 (°)
    float v_max;        // 最大速度 (°/s) 正值
    float T;            // 总运动时间 (s)

    // 内部推导出的参数
    float a;            // 加速度大小 (°/s^2) 正值
    float t_acc;        // 加速/减速时间 (s)
    float t_const;      // 匀速时间 (s)
    float v_peak;       // 实际峰值速度 (°/s) 带符号
    float sign;         // 运动方向 (+1 或 -1)
    float t_last;       // 上次更新时间 (s)
} AngleTraj;

// // 笛卡尔空间直线轨迹参数
// typedef struct {
//     float xs, ys;        // 起点
//     float xe, ye;        // 终点
//     float L;             // 总弧长
//     float ux, uy;        // 单位方向向量
//     float V_max;         // 期望最大速度
//     float a_max;         // 加速度
//     float Vp;            // 实际能达到的最大速度
//     float t_acc;         // 加速时间
//     float t_const;       // 匀速时间
//     float s_acc;         // 加速段位移
//     float T_total;       // 总时间
//     float t_last;       // 上次更新时间 (s)
// } LineTraj;

// // 笛卡尔空间圆弧轨迹参数
// typedef struct {
//     // 几何参数
//     float cx, cy;        // 圆心坐标 (m)
//     float r;             // 圆弧半径 (m)，正值
//     float start_angle;   // 起始角度 (rad)，从圆心到起点的向量角
//     float sweep_angle;   // 扫过角度 (rad)，正值逆时针，负值顺时针

//     // 运动参数
//     float L;             // 总弧长 = r * |sweep_angle|
//     float V_max;         // 期望最大线速度 (m/s)
//     float a_max;         // 期望最大线加速度 (m/s?)

//     // 梯形速度推导参数
//     float Vp;            // 实际峰值速度 (m/s)
//     float t_acc;         // 加速时间 (s)
//     float t_const;       // 匀速时间 (s)
//     float s_acc;         // 加速段弧长 (m)
//     float T_total;       // 总运动时间 (s)

//     // 运行时状态
//     float t_last;        // 上次更新时间 (s)
// } ArcTraj;

// 直线轨迹参数结构体扩展
typedef struct {
    float xs, ys;       // 起点
    float xe, ye;       // 终点
    float ux, uy;       // 单位方向向量
    float L;            // 总弧长
    float V_max;        // 最大允许速度
    float a_max;        // 最大允许加速度
    float Vs;           // 初速度（标量，非负）
    float Ve;           // 末速度（标量，非负）
    float Vp;           // 实际峰值速度
    float t_acc;        // 加速段时间
    float t_const;      // 匀速段时间
    float t_dec;        // 减速段时间
    float T_total;      // 总时间
    float t_last;       // 当前时间
} LineTraj;

// 圆弧轨迹参数结构体同样扩展
typedef struct {
    float cx, cy;       // 圆心
    float r;            // 半径
    float start_angle;  // 起始角度（弧度）
    float sweep_angle;  // 扫过角度（弧度）
    float L;            // 弧长
    float V_max;
    float a_max;
    float Vs;
    float Ve;
    float Vp;
    float t_acc;
    float t_const;
    float t_dec;
    float T_total;
    float t_last;
} ArcTraj;

// 轨迹类型枚举
typedef enum {
    TRAJ_TYPE_JOINT = 0,
    TRAJ_TYPE_LINE = 1,
    TRAJ_TYPE_ARC = 2  
} TrajType;

typedef enum 
{
    
	TRAJ_NOINIT = 0,
	TRAJ_IDLE,
    TRAJ_ACCEL,
    TRAJ_CONST,
    TRAJ_DECEL,
    TRAJ_DONE,
}TrajState;

// 联合体包装
typedef struct 
{
    TrajState State;      // 当前轨迹状态
    TrajType Type;       // 标识当前使用的轨迹类型
    union {
        AngleTraj Angle;
        LineTraj Line;
        ArcTraj Arc;
    } Data;
} Trajectory;


extern Trajectory Traj_BlockArm_Joint3;
extern Trajectory Traj_Gimbal;
extern Trajectory Traj_Arm;
extern float Traj_tLast;
extern uint8_t Traj_Flag;
extern float pos;
extern float vel;
extern float acc;

extern uint8_t MotorCtrl_Flag;


/*对接微调*/
/* ---------- 机械与安装参数（需根据实际标定） ---------- */
//#define QR_OFFSET_X        ( 0.f)   // 二维码在目标点右侧 5mm
#define CAM_TO_CENTER_X    ( 0.f)   // 模块中心在摄像头右侧 0m,
#define ALIGN_TOLERANCE    ( 1.f)  // 允许偏差 (1mm)
#define MOVE_SPEED         ( 0.02f)   // 平移速度 (m/s)，正为右
#define TIMEOUT_MS         ( 10000 )  // 最大执行时间 10 秒

extern float QR_OFFSET_X;

typedef struct 
{
    float radar_x;
    float radar_y;
    float radar_yaw;
    float x2;
    float y2;
    float yaw2;
    float yaw3;
    float distance;
}P_VISION_DATA;
extern P_VISION_DATA p_vision_data;//对接微调视觉数据结构体变量

#endif
