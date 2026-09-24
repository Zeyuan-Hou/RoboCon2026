#ifndef __ROBOT_H
#define __ROBOT_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "stm32f4xx.h"

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
#define RADIAN_15 0.261799387f
#define RADIAN_45 0.785398163f
#define RADIAN_75 1.308996939f
#define RADIAN_100 0.000174532922f
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

//遥控器相关变量
#define LEFT_JS_X_MID 2180 
#define LEFT_JS_X_MAX 4095 
#define LEFT_JS_X_MIN 182 
#define LEFT_JS_Y_MID 2030 
#define LEFT_JS_Y_MAX 4089 
#define LEFT_JS_Y_MIN 64 
#define RIGHT_JS_MID 2152 
#define RIGHT_JS_MIN 140 
#define RIGHT_JS_MAX 4091  

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

/*系统自检结构体*/
typedef struct
{
		uint16_t can_send_fps;					/**< 发送电流频率  */
		uint16_t can_send_cnt;					/**< 发送电流计数  */
    
    uint16_t can_rec_fps[2];        /**< CAN 接收帧率，分别对应CAN1和CAN2 */
    uint16_t can_rec_cnt[2];        /**< CAN 接收计数，分别对应CAN1和CAN2 */
    
    uint16_t motor_LU_fps;          /**< 电机频率 */
    uint16_t motor_LU_cnt;          /**< 电机计数 */
    
		uint16_t motor_RU_fps;          /**< 电机频率 */
    uint16_t motor_RU_cnt;          /**< 电机计数 */
	
		uint16_t motor_RD_fps;          /**< 电机频率 */
    uint16_t motor_RD_cnt;          /**< 电机计数 */
	
		uint16_t motor_LD_fps;          /**< 电机频率 */
    uint16_t motor_LD_cnt;          /**< 电机计数 */
	
		uint16_t Lift_Left_fps;					/**< 电机频率 */
		uint16_t Lift_Left_cnt;					/**< 电机计数 */
	
		uint16_t Lift_Right_fps;				/**< 电机频率 */
		uint16_t Lift_Right_cnt;				/**< 电机计数 */
		
		uint16_t Drib_Left_fps;          /**< 电机频率 */
    uint16_t Drib_Left_cnt;          /**< 电机计数 */
		
		uint16_t Drib_Right_fps;          /**< 电机频率 */
    uint16_t Drib_Right_cnt;          /**< 电机计数 */
		
		uint16_t motor_J60_fps;          /**< 电机频率 */
    uint16_t motor_J60_cnt;          /**< 电机计数 */
		
		uint16_t air_board_fps;					/**< 气动板频率 */
		uint16_t air_board_cnt;					/**< 气动板计数 */
		
		uint16_t uart1_fps;							/**< 上下板通信频率  */
		uint16_t uart1_cnt;							/**< 上下板通信计数  */
		
    uint16_t remote_control_fps;    /**< 遥控器频率 */
    uint16_t remote_control_cnt;    /**< 遥控器计数 */
	
		uint16_t uart2_fps;							/**< 陀螺仪通信频率  */
		uint16_t uart2_cnt;							/**< 陀螺仪通信计数  */
		
		uint16_t gyro_fps;							/**< 陀螺仪处理频率  */
		uint16_t gyro_cnt;							/**< 陀螺仪处理计数  */
		
		uint16_t uart5_fps;							/**< 视觉通信频率  */
		uint16_t uart5_cnt;							/**< 视觉通信计数  */
		
		uint16_t location_fps;							/**< 定位频率  */
		uint16_t location_cnt;							/**< 定位计数  */
		
		uint16_t navigation_fps;						/**< 导航频率  */
		uint16_t navigation_cnt;						/**< 导航计数  */
		
		uint16_t fun_task_fps;							/**<中心任务频率*/
		uint16_t fun_task_cnt;							/**<中心任务计数*/
		
		uint16_t communication_fps;					/**< 通信频率 */
		uint16_t communication_cnt;					/**< 通信计数 */
		
		uint16_t lwip_rec_fps;							/**< 网口通信频率 */
		uint16_t lwip_rec_cnt;							/**< 网口通信计数 */
		
		uint16_t lwip_send_fps;							/**< 网口通信频率 */
		uint16_t lwip_send_cnt;							/**< 网口通信计数 */
}ST_SYSTEM_MONITOR;

typedef enum
{
	RIGHT,
	MOTOR_ERROR,
	CAN_ERROR
}SYSTEM_STATE;

/*电机结构体*/
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
    
    float outerTarget;        /**< 外环目标位置 */
    float outerFeedback;      /**< 外环反馈位置 */
    float innerFeedback;      /**< 内环反馈速度 */
} ST_MOTOR;

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

typedef struct
{
    uint8_t motor_id_;
    uint8_t cmd_;
    float position_;
    float velocity_;
    float torque_;
    float kp_;
    float kd_;
}MotorCMD; //云深处电机结构体
//用于存储电机返回的数据
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
}MotorDATA;

enum Temp_Flag{
    kDriverTempFlag=0,
    kMotorTempFlag=1
};
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
    };
}ReceivedMotionData;
#pragma pack(pop) // 恢复默认字节对齐
typedef struct
{
	uint32_t position ;
  uint32_t velocity ;
  uint32_t torque ;
  uint32_t temp_flag ;
	uint32_t temperature ;
}ReceivedData;


/*随动轮*/
extern float degreeA;
extern float degreeB;

/*dt35*/
//bsp_can.c回调函数里使用
extern uint16_t dt35_distance[5];

extern uint16_t dt35_x1, dt35_y1;
extern uint16_t dt35_x2, dt35_y2;

//locate_algorithm.c里使用
extern fp32 q_now_fix_x, q_now_fix_y;
extern fp32 tem_y_dt35, tem_x_dt35;

extern fp32 q_fix_x1, q_fix_x2;
extern fp32 q_fix_y1, q_fix_y2;

extern bool flag_x_dt35, flag_y_dt35;
extern int flag_x, flag_y;

/*定位*/
extern fp32 fpPosXOffset; // X方向纠偏量，勿动
extern fp32 fpPosYOffset; // Y方向纠偏量，勿动
extern fp32 fpQOffset;    // 角度Q纠偏量，勿动 调整机器人初始姿态或补偿陀螺仪的系统误差。

extern fp32 fpStartX; // 
extern fp32 fpStartY; // 

extern ST_SYSTEM_MONITOR system_monitor;;

extern ST_ROBOT stRobot;

extern ST_Chassis_Run chassis_run;
extern ST_Nav nav;;

extern ST_FOLLOWER_WHEEL stFollowerWheel;
extern ST_GYRO stGyro;
extern ST_DT35 dt35_save,dt35_now;

extern PATH_PERMUTATION Path_Permuta;

extern ST_JS_VALUE Js_Value;

extern Path_State path_state;
extern POINT point_end;

extern bool flag_lock;
extern bool flag_permutation_path;
extern u8 flag_global_manual;
extern ST_GYRO stGyro;


extern uart1_tx_protocol_t uart1_eft;
extern uart1_rx_protocol_t uart1_efr;

extern ST_MOTOR leftup_motor ;
extern ST_MOTOR leftdown_motor ;
extern ST_MOTOR rightup_motor ;
extern ST_MOTOR rightdown_motor ;

extern float target_v[4];

extern u8 FLAG_NRF;

extern SYSTEM_STATE System_State;

extern float target_v[4];

extern u8 dt35_location;

#define BUFF_SIZE 100
extern uint8_t UDP_rx_data[BUFF_SIZE];
extern uint8_t UDP_tx_data[BUFF_SIZE];

/********************************************************************************************
运球的各种变量：运球状态机枚举、运球电机变量
*********************************************************************************************/
typedef enum
{
	MOTOR_SPEED_UP,
	CYLINDER_LOOSE,
	MOTOR_REVERSE,
	MOTOR_READY,
	CYLINDER_CLAMP,
	DRIBBLE_VIC,
	DRIBBLE_STICK,
	DRIBBLE_LOSE
}DRIBBLE_STATE;
extern DRIBBLE_STATE Dribble_State;
extern ST_MOTOR Drib_Motor_Left; //3508
extern ST_MOTOR Drib_Motor_Right; //3508
extern ST_PID Drib_Left_Pid;
extern ST_PID Drib_Right_Pid;
extern float Drib_Left_Targetv;
extern float Drib_Right_Targetv;
extern ST_SMC Drib_Left_Smc;
extern ST_SMC Drib_Right_Smc;

/********************************************************************************************
运球单项赛的各种变量：14次运球加七段路径，重启枚举
*********************************************************************************************/
typedef enum
{
	DRIB_EXIT,
	DRIB_PATH1,
	DRIB_PATH2,
	DRIB_PATH3,
	DRIB_PATH4,
	DRIB_PATH5,
	DRIB_PATH6,
	DRIB_PATH7,
	DRIB_RESET1_1,
	DRIB_RESET1_2,
	DRIB_RESET2_1,
	DRIB_RESET2_2,
	DRIB_RESET3_1,
	DRIB_RESET3_2,
	DRIB_RESET4_1,
	DRIB_RESET4_2,
	DRIB_RESET5_1,
	DRIB_RESET5_2,
	DRIB_RESET6_1,
	DRIB_RESET6_2,
	DRIB_RESET7,
}DRIBBLE_CHALLENGE;
typedef enum
{
	RESET_GO,
	RESET_BACK
}DRIB_RESET;
extern DRIBBLE_CHALLENGE Dribble_Challenge;
extern DRIBBLE_CHALLENGE Drib_Chal_Record;

/*运完球之后装球到发射机构的状态*/
extern u8 Load_State;
#define LOAD_END 3

extern ST_MOTOR Lift_Motor_Left;
extern ST_MOTOR Lift_Motor_Right;
extern ST_CascadePID Lift_Left_Pid;
extern ST_CascadePID Lift_Right_Pid;
extern float Lift_target_pos;
extern ST_TD Lift_Left_td;
extern ST_TD Lift_Right_td;

/*接球机构的状态*/
typedef enum
{
	J60_LIFT,
	REC_WAIT,
	DRIB_READY,
	REC_END
}REC_BALL_STATE;
extern REC_BALL_STATE Rec_Ball_State;
extern u8 Rec_Ball_flag;

typedef enum
{
	DRIB_LOAD_INIT,
	DRIB_INIT,
	DRIB,
	LOAD,
	DRIB_LOAD_END
}DRIB_LOAD;
extern DRIB_LOAD Drib_Load;

extern u8 Drib_flag;
extern u8 Load_flag;


extern u8 switch_on_1;
extern u8 switch_on_2;
extern ST_LPF gyro_fliter;
extern ST_GYRO Gyro_Data_Test;
extern int num_circle;//记录陀螺仪转的总圈数
extern fp32 fpSumPosQ;


typedef enum
{
	SHOOT_PATH1,
	SHOOT_PATH2,
	SHOOT_PATH3,
	SHOOT_PATH4,
	SHOOT_PATH5,
	SHOOT_PATH6,
	SHOOT_PATH7,
	SHOOT_PATH_FIXED,
	ROTATION_READY,
	SHOOT_READY,
	RESET_LOAD,
}SHOOT_CHALLENGE;
extern SHOOT_CHALLENGE Shoot_Challenge;


typedef struct
{
	float chassis_yaw;
	float aim_yaw;
	float aim_pitch;
	float aim_speed;
}ST_AUTO_AIM;


typedef enum
{
    LOCKED,             					//挂起状态，处于这种状态时该任务不进行
    READY,              					//准备状态，处于该状态时各功能模式回到统一位置，即存矿位置
    OPENED,             					//进行状态，处于该状态时任务朝着最终目标进行
    BACK,               					//回退状态，处于该状态时回到READY状态
}TASK_STATE;
typedef enum
{
    CHOOSE_EMPTY_MODE,
    CHOOSE_RESET_MODE,
    CHOOSE_MANUAL_MODE,
    CHOOSE_VISION_MODE,
    CHOOSE_SHOOTCHAL_MODE,
    CHOOSE_DRIBCHAL_MODE,
}TASK_CHOICE;
typedef enum
{
	NORMAL_MANUAL,
	GLOBA_MANUAL,
	AIM_MANUAL,
	VISION_NORMAL_MANUAL,
	VISION_AIM_MANUAL
}MANUAL_MODE;
extern TASK_STATE Reset_Task_State;
extern TASK_STATE Manual_Task_State;
extern TASK_STATE Vision_Task_State;
extern TASK_STATE ShootChal_Task_State;
extern TASK_STATE DribChal_Task_State;
extern TASK_CHOICE Task_Choice;
extern MANUAL_MODE Manual_Mode;


/*J60电机的各种使用变量，包括使能状态，TD插值，目标角度，反馈角度，电机温度，电机保护计数器*/
extern MotorCMD J60_Motor_Ctrl;
extern MotorDATA J60_Motor_Data;
typedef enum
{
	NO_TORQUE_MODE,
	CONTROL_MODE
}J60_CTRL_MODE;
typedef enum
{
	DISABLE_STATE,
	ENABLE_STATE
}J60_ENABLE_STATE;
extern double J60_angle;
extern J60_ENABLE_STATE J60_Enable_State;
extern J60_CTRL_MODE J60_Ctrl_Mode;
extern ST_TD J60_td;
extern float J60_target_pos;
extern float J60_motor_temp;
extern float J60_board_temp;
extern int J60_start_protect;
extern int J60_protect_cnt;


/*舵机宏定义*/
#define Servo_Num 2
extern int8_t uart4_tx_buffer[50];
extern uint8_t uart4_rx_buffer[8];
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
extern Servo Servo_Set;

typedef enum
{
	LOAD_BALL,
	DRIB_BALL
}SERVO_STATE;
extern SERVO_STATE Servo_State;

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
extern ST_VISION_DATA Vision_Data;
extern ST_PID Chassis_Aim_Yaw_Pid;
extern ST_PID Chassis_Global_Yaw_Pid;
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

extern ST_LESO_1order leso_leftup;
extern ST_LESO_1order leso_rightup;
extern ST_LESO_1order leso_rightdown;
extern ST_LESO_1order leso_leftdown;
extern fp32 compensation_leftup;
extern fp32 compensation_rightup;
extern fp32 compensation_rightdown;
extern fp32 compensation_leftdown;
extern int num_circle;//记录陀螺仪转的总圈数

extern ST_TD posX_veltX;
extern ST_TD posY_veltY;
extern ST_TD posW_veltW;

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
extern ST_WHEEL_ENCODER_VELT_FILTER wheel_encoder_velt_filter;
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
extern ST_GLOBAL_VELT_FILTER global_velt_filter;

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
extern ST_WHEEL2BODY_VELT Wheelvelt_To_Bodyvelt;

//typedef struct
//{
//	ST_LPF accel_x_filter;
//	ST_LPF accel_y_filter;
//}ST_BODY_ACCEL;

//extern ST_BODY_ACCEL Accel_Data;

extern KalmanFilter KF_Vx;
extern KalmanFilter KF_Vy;
extern KalmanFilter KF_W;

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
extern ST_RC_CTRL RC_Ctrl;


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

extern ST_DT35_NEW DT35_NEW;

extern ST_TD radar_x_td;
extern ST_TD radar_y_td;

extern ST_LPF radar_x_lpf;
extern ST_LPF radar_y_lpf;


typedef struct
{
	u8 Up_Motor_State;
	u8 Lift_Flag;
	float Lift_Motor;//升降电机的状态
	float FrictionWheel_vel[3];//三个摩擦轮的真实速度
	float Real_Pitch;//发射实时的PITCH值
}ST_UPBOARD_DATA;//上板发给下板的数据，用于遥控器实时显示摩擦轮速度

extern ST_UPBOARD_DATA Upboard_Data;

typedef struct
{
	float aim_yaw;//自瞄YAW轴，视觉为与篮筐的误差，底盘为整个上电的目标YAW轴
	float aim_distance;//车中心距离篮筐的距离
	float aim_pitch;//发射的PITCH
	float aim_speed;//发射的速度
}ST_AUTO_AIM_DATA;
extern ST_AUTO_AIM_DATA Auto_Aim_Data;
/*场地参数*/
#define FIELD_WIDTH 8000
#define FIELD_HEIGHT 15000

#define CH_COUNT 20
typedef struct Frame {
    float fdata[CH_COUNT];
    unsigned char tail[4];
}VOFA;
extern VOFA vofa;
#endif



