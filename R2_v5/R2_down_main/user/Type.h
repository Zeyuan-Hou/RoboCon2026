#ifndef __TYPE_H__
#define __TYPE_H__

#include "main.h"
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

typedef unsigned char  		UCHAR8;                    /* defined for unsigned 8-bits integer variable 	    */
typedef signed   char  		SCHAR8;                    /* defined for signed 8-bits integer variable	 */
typedef unsigned short 		USHORT16;                  /* defined for unsigned 16-bits integer variable 	*/
typedef signed   short 		SSHORT16;                  /* defined for signed 16-bits integer variable 	 */
typedef unsigned int   		UINT32;                    /* defined for unsigned 32-bits integer variable 	*/
typedef int   				    SINT32;                    /* defined for signed 32-bits integer variable 	*/
typedef float          		FP32;                      /* single precision floating point variable (32bits) */
typedef double         		DB64;                      /* double precision floating point variable (64bits)  */
typedef FP32              fp32;

#define WAYPOINT_SIZE   20
#define RADIAN	0.0174532922f	//PI/180 
#define PI 3.14159265368f
#define pi 3.14159265358f
#define R_ROBOT 235.07f //轮子到车中心距离
#define RUN_GEAR_RATIO 5.35f
#define R_WHEEL 63.5f
#define PI2 6.2831853072f
#define RADIAN_10 0.00174532922f

#define GM6020_uiGearRatio    1
#define M3508_uiGearRatio     19
#define M2006_uiGearRatio     36
#define M3508_siNumber 8192

#define offset_x 0
#define offset_y 0

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

/************GO1电机结构体*************/

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
	
		float Target_Delta_Deg; //将电机输入改为增量式
		uint8_t flag_init;
		float init_rad;
		float target_td;  //用于调试 
		float feedback_test; //反馈换算成初始位置为零
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

    uint16_t calc_crc;
    uint32_t timeout;       // 通讯超时 数量
    uint32_t bad_msg;       // CRC校验错误 数量
    RIS_MotorData_t motor_recv_data; // 电机接收数据结构体


} MotorData_t;


/************GO1电机结构体*************/


 //一阶低通滤波
typedef struct
{
    float preout;     //上一个输出值，用于保持滤波器状态，以便在连续调用之间维持滤波效果
    float out;        //当前输出值，即经过低通滤波处理后的信号
    float in;         //输入值，这是将要被滤波的原始信号
    float off_freq;   // 截止频率或称为权重，它决定了哪些频率成分可以通过滤波器
    float samp_tim;   //采样步长（时间），两次采样之间的时间，它对确定滤波器的时间常数至关重要
} ST_LPF;             //定义了一阶低通滤波器的结构体，一阶意味着它的传递函数有一阶多项式
 

//PID结构体
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
		
		float fpPreFB; //上一次反馈
    float fpKffv; // 前馈系数Kff
    float fpUff; // 前馈输出
		float fpKffa; ////二阶前馈系数
		float prev_des; //上一次目标，用于计算二阶前馈
		float friction; //摩擦前馈，正比于速度
		uint8_t flag_forward_precise;
		
		float fpforward_0; //0阶摩擦前馈，给定值，小速度微调要用
		float slope_forward;
		float test_a;  //测试目标加速度
		
		float ramp_foward;//斜坡前馈	
		float fpU_before;//存储值
		
		float fpUff_before;
		float fpUffMax;
		float fpfriEmin;
		float fpkfri;
		float fpfriMax;
			
} ST_PID;


// 串级PID
typedef struct
{
    ST_PID inner;                                             
    ST_PID outer;                                              
    float output;	
		float forward;
	  float G_forward;
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




//航模电机角度处理结构体
typedef struct {
    float pre_angle;
    float total_angle;
    uint8_t init;
} Angle_Processor;


//底盘航模电机结构体
typedef struct 
{
    float angle;              /**< 电机角度位置 */
    float anglev;             /**< 电机角速度 */
    float motor_current;      /**< 电机电流 */
	  Angle_Processor process;
} ST_MOTOR_;


 //向量结构体，用于底盘结算
typedef struct
{
    //笛卡尔直角坐标系
    float fpX;     // X方向差
    float fpY;     // Y方向差
    float fpW;      // 旋转速度  
    
    //极坐标系
    float fpLength; // 向量长度（单位mm）
    float fpThetha; // 向量与X轴角度（单位:弧度）
} ST_VECTOR;

typedef struct
{
    float fpPosX; // 横坐标X（单位：mm）
    float fpPosY; // 竖坐标Y（单位：mm）
    float fpPosZ; // 纵坐标Z (单位：mm)
    float fpPosQ; // 航向角Q（单位：rad）
} ST_POS;


//大疆编码器结构体
typedef struct {
    int siRawValue;        
    int siPreRawValue;     
    int siDiff;             
    int siSumValue;        
    float siGearRatio;      
    int siNumber;         
    float fpSpeed;           
		u8 state;								
} ST_ENCODER;

//大疆电机结构体
typedef struct 
{
    ST_ENCODER motor_encoder;
    float EncoderNum;         
    float encoder_speed;      
    float angle;              
    float anglev;            
    float motor_current;    
    
    float outerTarget;        
    float outerFeedback;     
    float innerFeedback;     
} ST_MOTOR;


//两个2006结构体
typedef struct 
{
	ST_MOTOR DJI_1;
	ST_MOTOR DJI_2;
} ST_DJI_MOTOR;


//轮子电机总结构体
typedef struct 
{
	ST_MOTOR_ wheel_1;
	ST_MOTOR_ wheel_2;
	ST_MOTOR_ wheel_3;
	ST_MOTOR_ wheel_4;
    
} MOTOR_WHEEL;



//帧率检测
typedef struct {
	
uint8_t  wheel_201;
uint8_t  wheel_202;
uint8_t  wheel_203;
uint8_t  wheel_204;
	
uint8_t  lift_201;
uint8_t  lift_202;
uint8_t  lift_203;
uint8_t  lift_204;
	
uint8_t  remote;
uint8_t  key;
uint8_t  DT35;

uint8_t  DJI_201;
uint8_t  DJI_202;
	
uint8_t  vision;
uint8_t  inner; //板间通信
	
}MONITOR_ERROR;



//帧率检测
typedef struct {
	
uint16_t wheel_201;
uint16_t wheel_202;
uint16_t wheel_203;
uint16_t wheel_204;
	
uint16_t lift_201;
uint16_t lift_202;
uint16_t lift_203;
uint16_t lift_204;
	
uint16_t remote;
uint16_t key;
uint16_t DT35;

uint16_t DJI_201;
uint16_t DJI_202;
	
uint16_t vision;
uint16_t inner; //板间通信
	
}MONITOR;


typedef struct
{
	MONITOR rate_cnt;
	MONITOR rate_fps;
	uint8_t error_fps;
} ST_SYSTEM_MONITOR;



/*底盘单环pid结构体*/
typedef struct
{
  ST_PID wheel_1;
  ST_PID wheel_2;
  ST_PID wheel_3;
  ST_PID wheel_4;
	
}ST_Chassis_Run;

typedef struct
{
  ST_PID DJI_1;
  ST_PID DJI_2;
	uint8_t flag_2006_V; //2006计算标志位
}ST_DJI_RUN;

// 动态调整滤波系数的LPF
typedef struct {
    float alpha_dynamic;  // 动态滤波系数
    float y_prev;
    float error_threshold; // 误差阈值
} AdaptiveLPF;


//控制层标志位
typedef struct
{
	uint8_t chassis_flag; //底盘发送标志位，为0时发0电流
	uint8_t dji_flag;			//两个2006发送标志位，为0时发0电流
	uint8_t remote_flag;  //遥控标志位，为0时遥控无法控制
}ST_FLAG;


typedef struct
{
  uint16_t  Num_1;
	uint16_t  Num_2;
	uint16_t  Num_3; //DT35测距结构体
	uint16_t  Num_4;
	
}DT35_distanceT;

//视觉结构体
typedef struct {
	
    float radar_x;
    float radar_y;
    float radar_z;
    float radar_yaw;
	
    uint8_t entry_kfs_id; //入口点序号
    uint8_t path_number; //路径编号
	
    uint8_t state_1; //状态数组 0不吸 1正吸 2左侧吸 3右侧吸	4正左侧吸 5正右侧吸
    uint8_t state_2;
    uint8_t state_3;
    uint8_t state_4;
	
    float s1_x; //第一块正吸x
    float s1_y;//第一块正吸y
    float s1_side_left_x;//第一块左侧吸x
    float s1_side_left_y;
    float s1_side_right_x;
    float s1_side_right_y;
	
    float s2_x;
    float s2_y;
    float s2_side_left_x;
    float s2_side_left_y;
    float s2_side_right_x;
    float s2_side_right_y;
		
    float s3_x;
    float s3_y;
    float s3_side_left_x;
    float s3_side_left_y;
    float s3_side_right_x;
    float s3_side_right_y;
		
    float s4_x;
    float s4_y;
    float s4_side_left_x;
    float s4_side_left_y;
    float s4_side_right_x;
    float s4_side_right_y;
		
		float s1_center_x;
		float s1_center_y;
		float s2_center_x;
		float s2_center_y;
		float s3_center_x;		
		float s3_center_y;
		float s4_center_x;		
		float s4_center_y;		
		
		
		uint8_t header_num_1; //第几个头
		uint8_t header_num_2; 		
		uint8_t header_num_3; 
		uint8_t header_num_4; 
		uint8_t header_num_5; 
		uint8_t header_num_6; 
		
		uint8_t aruco_detect_flag; // 二维码识别标志位
		uint8_t KFS_number;
		
		uint8_t state_1_repeat;  // 回到一区重试再走一遍用 0不吸 1左吸 2右吸 3左右吸
		uint8_t state_2_repeat;
		uint8_t state_3_repeat;
		uint8_t state_4_repeat;	

		uint8_t arcuo_left_or_right;  //三区视觉识别，左板卡 or 右电脑
		uint8_t R1_two;    //R1有无在2号梅林
		
} vision_Data_t;


//导航的位置pid
	typedef struct {
    ST_PID pid_x;      /* X轴位置PID */
    ST_PID pid_y;      /* Y轴位置PID */
    ST_PID pid_w;      /* 角度位置PID */
} POS_TRACKER;
	
// 速度结构体
typedef struct
{
    float fpVx; // Ｘ方向速度（单位mm/s）
    float fpVy; // Y方向速度（单位：mm/s）
    float fpW;  // 角速度（单位0.1度/s）
} ST_VEL;

typedef struct
{
    // 定义自动路径数据，包含位置 PID 数据
    // 包含位置控制的 PID 数据
		POS_TRACKER  pos_pid;
    ST_VEL auto_path_vel; // 自动路径规划出的目标速度
    ST_VEL basic_velt;    // 自动路径规划出的目标速度
    uint32_t run_time;    // 运行时间
    uint8_t run_time_flag;
    uint32_t run_Sumtime;        // 总运行时间
    uint32_t rotation_time;
    uint8_t number;             // 自动路径选择第几个
    uint8_t number_permutation; // 路径组合选择第几个
    uint8_t number_point;       // 微调路径选择第几个
    uint8_t get_block;
} ST_Auto_Path;


// 路线类型
typedef struct
{
    ST_VECTOR point[5]; // 一段组合微调中有若干个目标点
    ST_VECTOR velt[5];  // 到达每个目标点对应的期望速度
    int32_t time[5];    // 走每一段路径的期望时间
    uint8_t point_num;  // 一段组合微调中小段路径的数量

    uint8_t point_inx; // 第几段小路径，从0计数
    uint8_t flag_point_to_point; // 用于navigate.c第一次进入选择路径
    uint8_t flag_cube_set;  // 第一次进入小段路径时计算三次多项式系数
    uint32_t point_tim;     // 最后一段小路径到达死区后过了多久
} PATH_POINT;


typedef struct
{
    float k3, k2, k1, k0; //三次项系数
} Cube_Line;


typedef enum
{
		NAV_INIT = 0,
    NAV_LOCK = 1,               // 速度锁死
    NAV_POINT_TO_POINT = 2,     // 仅需要目标位置的实时反馈位置的点到点导航
		NAV_DT35,  									//靠DT35纠正取头位置，角度用雷达
		NAV_DT35_radar_AREA,				//在三区放置KFS，Y轴用DT35，X轴和角度用雷达
		NAV_only_point_area2, 			//纯点到点二区梅林上
		NAV_LOCK_POS,								//用于二区位置锁定
		NAV_LOCK_POS_3area,         //用于三区位置锁定
		NAV_LOCK_up_down = 8,      			//用于二区上下台阶锁YAW，三区站立锁前后
		NAV_SLOPE,    							//用于上三区斜坡的点到点
		NAV_RAMP,    							//用于上三区上坡状态机
		NAV_ONE_AREA_LOCK,       //用于一区对接保持一个强硬的底盘
		NAV_REMOTE,								//用于底盘遥控
		NAV_TEST,  //13
} Nav_State;


// 整个导航系统的主结构体
typedef struct
{
    Nav_State nav_state;                // 导航系统状态
    ST_VECTOR expect_robot_global_velt; // 目标全局速度
		ST_Auto_Path auto_path;             // 自动路径总结构体
} ST_Nav;   


//轮子速度分配
typedef struct
{
	fp32 wheel_1, wheel_2, wheel_3, wheel_4;
	fp32 dji_left,dji_right;
}chassis_run_des;


//夹头坐标
typedef struct
{
	float header_1_x,header_1_y;
	float header_2_x,header_2_y;
	float header_3_x,header_3_y;
	float header_4_x,header_4_y;
	float header_5_x,header_5_y;
	float header_6_x,header_6_y;
} header_pos_t;   


//用于点到点的目标值
typedef struct
{
	float target_x,target_y,target_rad;
}target_t;

//用于导航容错
typedef struct
{
	float allow_x,allow_y,allow_rad;
}allow_t;




extern uint8_t ramp_state ;
//帧率检测
extern ST_SYSTEM_MONITOR monitor;

extern uint8_t ramp_test_flag;
//气动板
extern uint8_t airCtrl[1];
extern uint8_t AirOperaterCtrlBuf[6];
extern uint8_t travel_switch;
extern uint8_t travel_switch_mode[8];
extern DT35_distanceT DT35_temp;
extern DT35_distanceT DT35_fact;
extern float DT35_X_fact,DT35_Y_fact;

//GO1 + TD
extern RIS_MotorData_t uart_rx_data;
extern MotorCmd_t cmd_1,cmd_2,cmd_3,cmd_4;
extern MotorData_t data_1,data_2,data_3,data_4;
extern ST_TD GO1_td_1,GO1_td_2,GO1_td_3,GO1_td_4;

//大疆电机 + PID
extern ST_DJI_MOTOR DJI_MOTOR;
extern ST_DJI_RUN dji_run;

//底盘轮子电机 + PID
extern uint8_t flag_wheel_slope;	//轮子上斜坡时的前馈标志位
extern MOTOR_WHEEL motor_wheel;
extern ST_Chassis_Run chassis_run;

//视觉
extern uint8_t vision_rec_test[166];
extern uint8_t vision_rec[166];
extern vision_Data_t vision_data_recieve;

//导航
extern POS_TRACKER NAV_DT35_PID;  //一区DT35 PID 
extern ST_Nav nav;
extern POS_TRACKER point_only;   //点到点pid
extern POS_TRACKER point_2_area; //二区pid
extern POS_TRACKER NAV_LOCK_POS_PID; //锁位置pid
extern ST_POS robot_pos; 					/*机器人反馈坐标（实时更新）*/
extern header_pos_t header_pos;		//夹头坐标（按照视觉发来的信息排序）
extern target_t area_one_target;	//一区点到点目标值
extern target_t area_two_target;	//二区点到点目标值
extern target_t area_three_target;//三区点到点目标值
extern float K_VEL_X,K_VEL_Y,K_VEL_W; //导航速度规划前馈的系数
extern chassis_run_des straight_des ,rotation_des; //轮子速度分配
extern allow_t allow_nav;  //导航容许范围
extern uint8_t reach_kfs_num;  //经过了规划的第几个KFS
extern Cube_Line cube_x, cube_y, cube_w; //三次曲线样条系数
extern PATH_POINT Path_Point;      //路线类型
extern uint8_t point_only_state; //纯点到点坐标选取状态
extern allow_t allow_point;  //点到点容许范围
extern uint8_t only_point_range_state; //点到点范围标志位
extern float offset_2006; //三区站起来用2006小轮纠位置
extern uint8_t gyro_or_radar; //当前导航yaw反馈基于雷达还是陀螺仪标志位  0为陀螺仪 1为雷达


//上下台阶标志位
extern uint8_t lift_state;//四条腿站起状态
extern uint8_t lift_state_two;//前两条腿垫高状态
extern uint8_t up_down_state;//总标志位
extern uint8_t up_state_200; 
extern uint8_t up_state_400;
extern uint8_t down_state_200;
extern uint8_t down_state_400;
extern uint8_t combine_state; //合体
//GO1参数切换标志位
extern uint8_t GO1_state_change;
//导航精度切换标志位
extern uint8_t nav_reach_state;
//底盘和遥控标志位
extern ST_FLAG ctrl_flag;
//导航路径切换标志位
extern uint8_t flag_lock;  //到达目标锁住
extern uint8_t all_path_state; //总路径状态
extern uint8_t path_state_1;  //一区
extern uint8_t path_state_2	;	//二区
extern uint8_t path_state_3;	//三区 
extern uint8_t path_state_4;  //三区回一区重试
extern uint8_t first_path_state;  //路径切换
//一区可以执行准备动作标志位
extern uint8_t flag_one_area_ready;
//导航区域切到点到点的标志位
extern uint8_t flag_area_to_point;


//串口通信
extern uint8_t uart4_receive[21];	//板间串口
extern uint8_t inner_send[8];    //板间发送
extern uint8_t inner_receive[21]; //板间接收
extern uint8_t uart_ctrl_receive[4]; //硬件键盘串口
extern uint8_t key_receive[4];		//硬件键盘接收
extern uint8_t key_send[5];		//硬件键盘发送
extern float yaw_gyro;//陀螺仪yaw角
extern float yaw_gyro_rad;
extern float gyro_w; //上层传回角速度
extern uint8_t vision_send[3]; //向视觉发送标志位
 


//测试用
extern uint8_t remote_up_down; //遥控上下台阶标志位
extern float ucGateX;
extern float ucGateY;
extern float ucGateW;
extern float ssXSpedLimit;
extern float ssYSpedLimit;
extern float ssWSpedLimit;
extern float test_rad;
extern float test_forward;
extern uint8_t flag_up;
extern uint16_t cnt_test;
extern uint16_t fps_test;
extern float cmd_1_T,cmd_2_T,cmd_3_T,cmd_4_T;
extern uint8_t test_flag;
extern float test_f;


//适应性训练遥控发送
extern uint8_t remote_send[25];
extern MONITOR_ERROR monitor_error;
extern uint8_t remote_state;
extern float test_remote_x;
extern float test_remote_y;
extern float test_wheel_1 ;
extern target_t target_test;


#endif
