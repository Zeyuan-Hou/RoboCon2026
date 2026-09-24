#ifndef ___TYPES_H___
#define ___TYPES_H___

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "math.h"


/*------------------------------常�?�常�?--------------------------------*/
#define PI 3.1415926536f
#define PI2 6.2831853072f
#define PISHORT 3.14f
#define RADIAN_10 0.00174532922f // PI/1800
#define RADIAN_100 0.000174532922f
#define DEG 57.29578f
#define DEG_10 572.9578f      // DEG*10
#define RADIAN 0.0174532922f  // PI/180
#define RAD10 572.9578f
#define PI_2 1.570796326795f  // pi/2
#define PI_3 1.0471975512f
#define PI_4 0.785398163f
#define PI_6 0.523598775f
#define RADIAN15 0.261799387f // 15rad
#define RADIAN45 0.785398163f
#define RADIAN75 1.308996939f
#define RADIAN105 1.832595715f
#define RADIAN135 2.356194490f
#define RADIAN165 2.879793266f
#define EPS 1e-5

#define TRUE		(1)
#define FALSE		(0)

#define DEC			(10)
#define HEX			(16)


#define LENGTH 400.f  //半车�?
#define WIDTH 400.f	//半车�?
#define R_ROBOT Geometric_mean(LENGTH,WIDTH) //�?子到车中心距�?
#define R_WHEEL 65.f//63.5f
#define RUN_GEAR_RATIO 6.f


// typedef uint32_t  u32;
// typedef uint16_t u16;
 typedef uint8_t  u8;
// typedef int32_t  s32;
// typedef int16_t s16;
// typedef int8_t  s8;
// typedef int bool;
typedef unsigned char  		UCHAR8;                    /* defined for unsigned 8-bits integer variable 	    */
typedef signed   char  		SCHAR8;                    /* defined for signed 8-bits integer variable	 */
typedef unsigned short 		USHORT16;                  /* defined for unsigned 16-bits integer variable 	*/
typedef signed   short 		SSHORT16;                  /* defined for signed 16-bits integer variable 	 */
typedef unsigned int   		UINT32;                    /* defined for unsigned 32-bits integer variable 	*/
typedef int   				    SINT32;                    /* defined for signed 32-bits integer variable 	*/
typedef float          		FP32;                      /* single precision floating point variable (32bits) */
// typedef double         		DB64;                      /* double precision floating point variable (64bits)  */
 typedef FP32              fp32;
fp32 Geometric_mean(fp32 a,fp32 b);

#define GM6020_uiGearRatio    1
#define M3508_uiGearRatio     19
#define M2006_uiGearRatio     36
#define M3508_siNumber 8192

// 红色方场地边界（单位：mm�?
// 假�?�场地坐标系：X向右增长，Y向前增长
//定位常量
#define FIELD_Y      5998    // Y方向场地总长�?
#define FIELD_X     8199    // X方向场地总长�?
#define REGION1_X 1992 //一区X方向宽度
#define REGION2_Y 1198 //二区�?一/二部分Y方向宽度
#define REGION2_3_X 1449 //二区�?三部分X方向宽度
#define REGION_TOLERANCE 25//定位区域容忍�?
#define REGION3_X 2500 //三区X方向长度



//这是不加导轮的情�?
// #define length_x 800 //初�?�情况下车在X方向上长�?
// #define length_y 800 //初�?�情况下车在Y方向上长�?
// #define length_differX 400//初�?�状态下全向�?�?心沿着X轴长�?
// #define length_differY 400//初�?�状态下全向�?�?心沿着Y轴长�?

//这是加上导轮的情�?
#define length_x (810+19)//832//825 //初�?�情况下车在X方向上长�?
#define length_y (830+20)//834//825 //初�?�情况下车在Y方向上长�?
#define length_differX_1 405//初�?�状态下全向�?�?心沿着X轴�?�方向长�?
#define length_differY_1 415//初�?�状态下全向�?�?心沿着Y轴�?�方向长�?
#define length_differX_2 (405+19)//425//初�?�状态下全向�?�?心沿着X轴负方向长度
#define length_differY_2 (415+20)//425//初�?�状态下全向�?�?心沿着Y轴负方向长度



/*******************************************上下板通信*****************************************/
extern uint8_t uart1_rx_buff[6];
extern uint8_t Usart1_Receive_length;
extern uint8_t uart1_tx_buffer[26];
extern uint8_t vision_tx[3];	// ����
/*******************************************上下板通信*****************************************/




typedef struct
{
    float fpDes; // ���Ʊ���Ŀ��ֵ
    float fpFB;  // ���Ʊ�������ֵ
    float fpE;   // ����ƫ��

    float fpKp; // ����ϵ��Kp
    float fpKi; // ����ϵ��Ki

    float fpU; // �����
    float fpUKp; // ���������
    float fpUKi; // ���������

    float fpUMax;  // ��������ֵ
    float fpUpMax; // ����������޷�
    float fpUiMax; // ����������޷�
    float fpEMax;  // ƫ���޷�

    float fpKffv; // ǰ��ϵ��Kff
    float fpKffa; // ǰ��ϵ��Kff
    float last_target; // �ϴ�Ŀ��ֵ
    float fpUff; // ǰ�����
    float fpUffMax; // ǰ��������ֵ
} ST_PI_Feedforward;

typedef struct
{
    ST_PI_Feedforward leftup; // �Ѱ���Ħ������
    ST_PI_Feedforward rightup;
    ST_PI_Feedforward leftdown;
    ST_PI_Feedforward rightdown;
} ST_Chassis_Run2;

extern ST_Chassis_Run2 chassis_run;

/*******************************************nRF24L01*******************************************************************/
/*遥控器结构体*/
typedef struct
{
    uint16_t usJsKey;//合并按键
    uint16_t indepen_usJsKey[8];//8�?�?立按�?
    uint16_t usJsLeft_X; // 左摇杆x方向
    uint16_t usJsLeft_Y; // 左摇杆y方向
    uint16_t usJsRight_X;  // 右摇杆x方向
    uint16_t usJsRight_Y;  // 右摇杆y方向
} ST_JS_VALUE;
extern ST_JS_VALUE Js_Value;
//应答信号ACK数据�?
typedef struct
{
	uint8_t		Ack_Buf[5];
	uint8_t		Ack_Len;
	uint8_t		Ack_Channel;
	uint8_t		Ack_Status;
}ACK_PAYLOAD;
 extern uint8_t nRF24L01_Tick;                             
 extern ACK_PAYLOAD nRF24L01_ack_pay;
 extern uint8_t nRF24L01_RxBuf[32];
 extern uint16_t Uart4_Receive_length ;
 extern uint8_t REMOTE_BUFFER[3];
/*******************************************nRF24L01*******************************************************************/



















/*******************************************通用*******************************************************************/
typedef enum{
    CARTESIAN,//笛卡尔坐标系
    POLAR    //极坐标系
}COORDINATE;


typedef struct
{
    //笛卡尔直角坐标系
    float fpX;     // X方向�?
    float fpY;     // Y方向�?
    float fpW;      // 旋转速度  叉乘运算？待�?
    
    //极坐标系
    float fpLength; // 向量长度（单位mm�?
    float fpThetha; // 向量与X轴�?�度（单�?:弧度�?
	  COORDINATE type;  //坐标系类�?
} ST_VECTOR;


// ��̬�����˲�ϵ����LPF
typedef struct {
    float alpha_dynamic;  // ��̬�˲�ϵ��
    float y_prev;
    float error_threshold; // �����ֵ
} AdaptiveLPF;


typedef struct
{
    float fpDes; // 控制变量�?标�?
    float fpFB;  // 控制变量反�?��?

    float fpE;    // �?次偏�?
    float fpPreE; // 上�?�偏�?
    float fpSumE; // 总偏�?

    float fpKp; // 比例系数Kp
    float fpKi; // �?分系数Ki
    float fpKd; // �?分系数Kd

    float fpU; // �?�?PID运算结果
    float fpUp; // 比例输出
    float fpUi; // �?分输�?
    float fpUd; // �?分输�?
    float fpPreUd; // 上�?�微分输�?

    float fpUMax;  // 总输出限�?
    float fpUpMax; // 比例项输出限�?
		float fpSumEMax; //总�??�?限幅
    float fpUdMax; // �?分项输出上限

    float fpEMax; // 偏差限幅
    float fpEMin; // 偏差死区
		
		float feedforward;
		float forward_rub; //�����Ħ������ֵ
    
		AdaptiveLPF LPF;//����Ӧ��̬�˲�
} ST_PID;


typedef struct
{
    ST_PID inner; //内环
    ST_PID outer; //外环
    FP32 output; //串级输出，等于inner.output
}CascadePID;

// TD（微分跟�?�?）参数结构体
typedef struct
{
	float m_x1;		            //TD（微分跟�?�?）生成的�?标位�?
	float m_x2;								//速度
	float m_x;								//位移
	float m_r;								//TD阻尼因子（决定跟�?速度，r越大跟得越快，�?�果追求�?速响应，�?分�?�测的滤波效果会变差�?
	float m_h;								//TD滤波因子（算法式�?的h0，h0越大�?分�?�测的滤波效果越好）
	float m_T;								//TD�?分�?�长（h为�?�长,h越小滤波效果越好，这�?值应该与采样周期一致）
	float m_aim;							//�?标位�?
}ST_TD;

//pid状态标志位
typedef enum
{
	VELT_LOOP,//单环 无位�?�?纠偏
  OPEN_LOOP,//开�?控制 不进行pid计算
  DOUBLE_LOOP//双环 
}Pid_State;
//enum State_ControlLoop {DIS,OPEN_LOOP,SPEED_LOOP,POSITION_LOOP,MULTIPLE_LOOP};
enum State_Input {ZREO_Target,STEP_Target,SIN_Target};
/*******************************************通用*******************************************************************/















/*******************************************电机*******************************************************************/
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
typedef struct
{
    ST_PID inner;                                             
    ST_PID outer;                                              
    float output;                           
} ST_CascadePID;

typedef struct 
{
    ST_ENCODER motor_encoder; 
    float EncoderNum;         
    float encoder_speed;      
    float angle;              
    float anglev;             
    float motor_current;      
    
	  ST_CascadePID motor_pid;
    float outerTarget;        
    float outerFeedback;     
    float innerFeedback;     
} ST_MOTORT;







//�?模电机中间结构体，因为控制航模电机用到的结构体不止这一�?，这�?�?次�?�的�?用到了很少几�?�?
typedef struct
{
  ST_ENCODER motor_encoder;	
  CascadePID motor_pid;

  FP32 Encoder_pos;			
  FP32 Encoder_vel;		
  FP32 angle;				 
  FP32 anglev;				 
  FP32 motor_current;		

  FP32 outerTarget;				  
  FP32 outerFeedback;			  
  FP32 innerFeedback;			  

  FP32 Pos;
  FP32 MaxPos; 
  FP32 MinPos;  
  SSHORT16 temp; //tempreture
}MOTOR;
//四个�?模电机本�?运�?�的结构体的一部分
extern MOTOR leftup_motor ;
extern MOTOR leftdown_motor ;
extern MOTOR rightup_motor ;
extern MOTOR rightdown_motor;

typedef enum
{
    WITHOUT_FORWARD,//没有前�??
    WITH_FORWARD//有前�?
}Feed_Forward_State;

typedef struct
{
        ST_PID leftup;
        ST_PID rightup;
        ST_PID leftdown;
        ST_PID rightdown;
//        enum State_ControlLoop pid_state;
	      Pid_State pid_state;
        Feed_Forward_State feed_forward_state;  
}ST_Chassis_Run;
//extern ST_Chassis_Run chassis_run;
typedef struct
{
	fp32 rightup, rightdown, leftup, leftdown;
}chassis_run_des;

/***************************************起重�?3508***************************************************/
typedef enum
{
  Crane_Init,//起重机初始状�?
	Crane_Up,  //起重机上�?
	Crane_Down //起重机下�?
}CRANE_STATE;
extern CRANE_STATE Crane_State;
extern ST_MOTORT Crane_3508_1;
extern ST_MOTORT Crane_3508_2 ;
extern ST_TD Crane_3508_TD_1;
extern ST_TD Crane_3508_TD_2;
extern float Input_3508 ;
extern uint16_t Crane_Time;
/***************************************起重�?3508***************************************************/
/*******************************************电机*******************************************************************/






































/*******************************************定位*******************************************************************/
/*定位*/

/*******************************************雷达*********************************************/
extern uint8_t Radar_RxBuf[38];

typedef struct{
float radar_x;
float radar_y;
float radar_z;
float radar_yaw;
float x2;
float y2;
float yaw2;
float yaw3;
float distance;
}ST_VISION_DATA;

extern ST_VISION_DATA Vision_Data;



typedef struct
{
    float x[5];
    float y[5];
	  float z[5];
    float yaw[5];
    float lpf_k;
    int32_t tim;
    uint8_t flag;
} Location_Filter;

extern Location_Filter location_filter;

/*******************************************雷达*********************************************/

/*******************************************陀螺仪***********************************/
/*陀螺仪*/
typedef struct
{
    float fpQ_Cur;     // 陀螺当前数�?读数
    float fpQ_Pre;     // 陀螺上一次数�?读数，判�?旋转方向
} ST_GYRO;

typedef struct
{
	float pitch;			// 陀螺仪数据结构�?
	float roll;
	float yaw;
}GYRO_DATA;

//陀螺仪
extern uint8_t g_Usart2_Rx_buf[17];
extern uint16_t g_Usart2_Rx_cnt ; 
extern uint8_t g_Usart2_Tx_buf[1];
extern bool gyro_reset_flag;
extern uint8_t g_Decode_Data[512];	
extern uint16_t g_Decode_Data_pos;	
extern GYRO_DATA gyro_data;
extern ST_GYRO Gyro_Data_Test;
extern uint16_t gyro_tick;
extern int num_circle;//记录陀螺仪�?的总圈�?
extern fp32 fpSumPosQ;//记录陀螺仪�?的总�?�度
/*******************************************陀螺仪********************************/
typedef enum
{
	FACE_0,						//逆时�?0�?
	FACE_90,					//逆时�?90�?
	FACE_180,					//逆时�?180�?
	FACE_270,					//逆时�?270�?
	OTHER							//其他
} ST_DT35_GYRO_T;	  //定位车头朝向,逆时针为�?

typedef enum
{
	REGION1,			//一�?
	REGION2_1,		//二区
	REGION2_2,
	REGION2_3,
	REGION3,			//三区
	OTHERS,				//�?挡或者�?�区
  REGION3_RESET		//三区导航重试
} ST_REGION_T;		//判断机器人所在区�?


/*******
二区共三�?区域

/			/					/			/
									
/RE2_1/					/RE2_2/

/			/					/			/
							
/ /	/   /	 /	 /	/ /	/

/				REGION2_3			/

/	 / 	/  /  /  /  / 	/ 

********/
extern uint8_t FLAG_REGION3; //�?否上到三区标志位
extern fp32 fpPosXOffset;//X方向纠偏�?
extern fp32 fpPosYOffset;//Y方向纠偏�?
extern fp32 fpQOffset ;    // 角度Q纠偏量，勿动 调整机器人初始姿态或补偿陀螺仪的系统�??�?�?
extern fp32 fpStartX; // 5557.5f;//319 底盘半�??+导轮
extern fp32 fpStartY; // 545.0f;//361
extern float degreeA;
extern float degreeB;
extern uint8_t flag_DT35_locate; //DT35反�?�坐标是否可信标�?
extern float DT35_y;	//DT35判断�?信之后的�?/后方向车到场地边缘距�?
extern float DT35_x;	//DT35判断�?信之后的�?/右方向车到场地边缘距�?
extern float DT35_x_;//DT35X负方向距�?
extern float DT35_y_;//DT35Y负方向距�?
extern float LENGTH_Y;
extern float LENGTH_X;
extern uint8_t DT35_xFlag;//DT35�?否可信标�?
extern uint8_t DT35_x_Flag ;
extern uint8_t DT35_yFlag;
extern uint8_t DT35_y_Flag;
extern bool DT35_Correct_Flag;
extern bool DT35_Turn_Flag;
extern ST_REGION_T REGION_STATE;
extern ST_DT35_GYRO_T DT35_GYRO_STATE;
extern uint8_t region3_state;
extern uint8_t REGION3_temp;
extern uint8_t FLAG_REGION3_manual;

extern int16_t length_differx_1;	
extern int16_t length_differy_1;
extern int16_t length_differx_2;	
extern int16_t length_differy_2;






/********************************************************************************************
利用四轮速逆解算出车体速度，可以将�?子编码器逆解算出的车体速度和惯导系统里的车体速度进�?�比较，判断打滑
直接加了滤波�?，逆解算出的车体速度更加平滑一�?
*********************************************************************************************/



// 一阶低通滤�?
typedef struct{
    float preout;   // 上一�?输出值，用于保持滤波器状态，以便在连�?调用之间维持滤波效果
    float out;      // 当前输出值，即经过低通滤波�?�理后的信号
    float in;       // 输入值，这是将�?��??滤波的原始信�?
    float off_freq; // �?止�?�率或称为权重，它决定了�?些�?�率成分�?以通过滤波�?
    float samp_tim; // 采样步长（时间），两次采样之间的时间，它对确定滤波器的时间常数至关重�?
} ST_LPF;           // 定义了一阶低通滤波器的结构体，一阶意味着它的传递函数有一阶�?�项�?
/********************************************************************************************
�?导系统得出来的车体速度
直接加了滤波�?，解算出的车体速度更加平滑一�?
*********************************************************************************************/
typedef struct
{
	ST_LPF global_vx;
	ST_LPF global_vy;
	ST_LPF global_w;
}ST_GLOBAL_VELT_FILTER;
typedef struct 
{
	ST_LPF Vx;
	ST_LPF Vy;
	ST_LPF W;
}ST_WHEEL2BODY_VELT;

extern ST_TD posX_veltX;
extern ST_TD posY_veltY;
extern ST_TD posW_veltW;
extern ST_GLOBAL_VELT_FILTER global_velt_filter;
extern ST_WHEEL2BODY_VELT Wheelvelt_To_Bodyvelt;


//DT35测距结构�?
typedef struct
{
  uint16_t  Num_1;
	uint16_t  Num_2;
	uint16_t  Num_3; 
	uint16_t  Num_4;
	
}DT35_distanceT;
//DT35
extern DT35_distanceT DT35_distance;
extern DT35_distanceT DT35_dis;
extern DT35_distanceT DT35_distance_pre;

extern float DT35_Delta[4];
extern float DT35_X1_Count;
extern float DT35_Y1_Count;
extern float DT35_X2_Count;
extern float DT35_Y2_Count;

extern ST_TD td_Vx;
extern ST_TD td_Vy;
extern ST_LPF lpf_Vx;
extern ST_LPF lpf_Vy;

/*******************************************定位*******************************************************************/











/*******************************************system_monitor*******************************************************************/

typedef struct
{   
  	uint16_t can_send_fps_chassis;  //四个底盘�?模电机CAN发送�?�率
	  uint16_t can_send_cnt_chassis;	//四个底盘�?模电机CAN发送�?�数
	
	  uint16_t can_send_fps_3508;					/**< 起重�?3508发送电流�?�率  */
	  uint16_t can_send_cnt_3508;					/**< 起重�?3508发送电流�?�数  */
    
    uint16_t can_rec_fps[2];        /**< CAN 接收帧率，分�?对应CAN1和CAN2 */
    uint16_t can_rec_cnt[2];        /**< CAN 接收计数，分�?对应CAN1和CAN2 */
    
    uint16_t motor_LU_fps;          /**< 电机频率 */
    uint16_t motor_LU_cnt;          /**< 电机计数 */
    
		uint16_t motor_RU_fps;          /**< 电机频率 */
    uint16_t motor_RU_cnt;          /**< 电机计数 */
	
		uint16_t motor_RD_fps;          /**< 电机频率 */
    uint16_t motor_RD_cnt;          /**< 电机计数 */
	
		uint16_t motor_LD_fps;          /**< 电机频率 */
    uint16_t motor_LD_cnt;          /**< 电机计数 */
	
		
		
		
		
    uint16_t remote_control_fps;    /**< 遥控器�?�率 */
    uint16_t remote_control_cnt;    /**< 遥控器�?�数 */
	
		uint16_t flw_fps;							/**< 随动�?通信频率  */
		uint16_t flw_cnt;							/**< 随动�?通信计数  */
		
		uint16_t gyro_fps;							/**< 陀螺仪处理频率  */
		uint16_t gyro_cnt;							/**< 陀螺仪处理计数  */
		
		
		
		uint16_t location_fps;							/**< 定位频率  */
		uint16_t location_cnt;							/**< 定位计数  */
		
		uint16_t navigation_fps;						/**< 导航频率  */
		uint16_t navigation_cnt;						/**< 导航计数  */
		
		uint16_t VOFA_cnt;
		uint16_t VOFA_fps;
		
		uint16_t communicate_tx_fps;      /**< 向上板发送�?�率  */
		uint16_t communicate_tx_cnt;      /**< 向上板发送�?�数  */
		
		uint16_t communicate_rx_fps;      /**< 从上板接收�?�率  */
		uint16_t communicate_rx_cnt;      /**< 从上板接收�?�数  */
		
		uint16_t Crane_3508_fps[2];          /**< 起重�?3508接收频率  */
		uint16_t Crane_3508_cnt[2];          /**< 起重�?3508接收计数  */
		
		uint16_t AirBoard_Receive_fps;
		uint16_t AirBoard_Receive_cnt;
		
		uint16_t Vision_Receive_fps;
		uint16_t Vision_Receive_cnt;
		
		
		

}ST_SYSTEM_MONITOR;
extern ST_SYSTEM_MONITOR system_monitor;
extern float tim_sum_monitor;
/*******************************************system_monitor*******************************************************************/








/********************************************������************************************************************/
extern uint8_t AirCtrl_1[3];
extern uint8_t AirCtrl_2[1];
extern uint8_t AirCtrl_3[1];
extern uint8_t DT35_Data[8];//����������գ�ͨ������һ���͸��²��
extern uint8_t AirOperaterCtrlBuf[6];
/********************************************������************************************************************/














/*******************************************机器人整�?*******************************************************************/

// 坐标结构�?
typedef struct
{
    float fpPosX;  // �?坐标X（单位：mm�?
    float fpPosY;  // 竖坐标Y（单位：mm�?
	  float fpPosZ;
    float fpPosQ;  // �?向�?�Q（单位：0.1度）
} ST_POS;

// 速度结构�?
typedef struct
{
    float fpVx; // Ｘ方向速度（单位mm/s�?
    float fpVy; // Y方向速度（单位：mm/s�?
    float fpW;  // 角速度（单�?0.1�?/s�?
} ST_VEL;

typedef struct
{
    ST_POS stPos;     // 机器人中心坐标姿态由随动�?单独得出
    ST_VEL stVelt_global;   // 机器人中心在全场坐标系下的速度
    ST_VEL stVelt_local;        // 机器人中心在局部坐标系下的速度
	  ST_POS Robot_location;// 机器人由DT35和随动轮共同作用得出的坐�?
} ST_ROBOT;
extern ST_ROBOT stRobot;
extern ST_ROBOT test_stRobot;
/*******************************************机器人整�?*******************************************************************/







/*******************************************随动�?*******************************************************************/
/*随动�?*/
#define ALPHA_A_Inc  -0.77407108916767641826339740873664//-0.78026052238001541994094623078126//-0.77407108916767641826339740873664//-0.77777992382485450217899369818042//-0.77738273277868386035294179237098//-0.77367098602956463349045179711538//0.27284626765250036273258160690602//-0.77727430011218567873498841436231//-0.7752729142318951494061707307992//-0.76507119804614998059832942089997//-0.76780621719827824023241191753186//-0.76911915981337208858548137868638//-0.77828811516783336088565192767419//2.3352671070518087326206568832276//2.3299730370293469938758335047169//2.3369086014672064699482234573225//2.3389329284310309553518436587183//2.328207804614841336388053605333//2.3445690793269533536147264385363//2.3374583285431000945209234487265//2.3426722296324729022387600707589//2.3415428002961022890815456776181;
#define ALPHA_A_Dec -0.77303888091278283312135499727447//-0.77924082168575015128197946978617//-0.22233665327126320154782490590151//-0.77303888091278283312135499727447//-0.7767552283927603440361053799279//-0.77817337251394558261807787857833//-0.77446742260369128008079542269115//-0.77296911546160218531298369271099//-0.7708659034647596941525193869893//-0.76838696112798954285239005912445//-0.7633974672676618888900179626944//-0.76545801640246990960037010154338//-0.76442066977574618658053395847674// -0.77634810482849969659469024918508//2.3364458630704962871504903887399//2.3303710542976698860684336978011//2.3382282704214349955407215020387//2.3392762120206804787869714346016//2.3293424694089730486723510693992//2.3444069469593142862606782728108//2.3395176409699431907540656538913//2.3447538846488189534511548117734//2.3441657564300339977592102513881;
#define ALPHA_B_Inc -2.34216992961535241946080532216//-2.3481391396804940363551850168733//-2.34216992961535241946080532216//-2.3457468568733848002239028573968//-2.3398200634429606736830464797094//-2.3362862629798946123571568023181//-2.3353450838745213324898486462189//-2.33432842910105309286450392392//-2.3370207296011638042898539424641//-2.3398496320258637126698886277154//-2.3384514135368998921649108524434//-2.3410651205658608020598876464646//-2.3295633303579545980710463481955//-2.3715596331438066890484606119571//-2.3769415665283362137927269941429//-2.3702920730136525229170274542412//-2.3689877425889238615752674377291//-2.3790715684507737215369616023963//-2.3631326407235717645960448862752//-2.3790991474812290817908433382399//-2.3734736892293564203271216683788//-2.372578599010998878782174870139;
#define ALPHA_B_Dec -2.3352749266947081530076957278652//-2.3411595853758195850957690709038//-2.3352749266947081530076957278652//-2.3388013144953325728181425802177//-2.3356152542834194285603643947979//-2.3321120425609049853221677039983//-2.3324993966529956601618778222473//-2.3263252398035572099388446076773//-2.3279744004689031378063646116061//-2.3380653972060101430940903810551//-2.3377608056104550726672641758341//-2.3389674686856367635812148364494//-2.3283279939662686608414787770016//-2.3697625370528987431839595956262//-2.3756591956666452958302215847652//-2.3701918602098017707646704366198//-2.3677385282982377212590563431149//-2.3779436005745027138402747368673//-2.3621001499464462369815009878948//-2.3779179994132344511115206842078//-2.3723052145142262681076772423694//-2.3708004755161469212509928183863;


//// matlab标定的KA_UP KA_DOWN KB_UP KB_DOWN等放在这�?
////  随动�?编码器数值到实际物理位移的转换系�?
////  位移(mm)=编码器值差×�?换系�?(mm/脉冲)
#define FW_Len_A_Inc -0.22369445424012232570554203903157//-0.22369445424012232570554203903157//-0.22288185788746384119463073147926//-0.2228626233362607511612196731221//-0.22367513850983758438850657057628//-0.074536712360530041343587015489902//-0.22370927278593641385207035909843//-0.22440992980532648015667973595555//-0.22488607309204924900036814960913//-0.22944865218570517639840034007648//-0.22644405712585199963449156257411//-0.22757485024104331938765710674488//-0.22131555283950218870892001632456//-0.22163624255201455626185236269521//-0.22118376539758979881789002774894//-0.22158432437806438453087309881084//-0.22121734552734947931718068048212//-0.22178236521586117135917959330982//-0.22047324411983412750792865608673//-0.22166367030759026590658322675154//-0.22207429218687166350143513682269;
#define FW_Len_A_Dec -0.2236905280514981109174499351866//-0.22233279658490257002512180406484//-0.2236905280514981109174499351866//-0.22287797352809970985454413039406//-0.22270561500913174657334536732378//-0.22351757836641708565572628231166//-0.22352877889500910590925286669517//-0.22205809096670373059545511296164//-0.22286526002015527581079368246719//-0.22477317746455136027172727608558//-0.22721255674246007560945770364924//-0.22539663420827929507517239926528//-0.2261715035132495221770199123057//-0.22171699823280302532602092924208//-0.22190516994222861391072854075901//-0.22163534057635589369361639455747//-0.22196649174604851650904890902893//-0.22164513010416972416116720978607//-0.22226057002736432366418739547953//-0.22082961452889393605403256515274//-0.2220221241966606751105928196921//-0.22234915470799876402452355250716;
#define FW_Len_B_Inc -0.22452401108521444217380746977142//-0.22589836248615344227452794712008//-0.22452401108521444217380746977142//-0.2253485268725865242611661187766//-0.22503803301122471403061808814527//-0.22421491859519918077303657355515//-0.22441680326216376095693760817085//-0.22207732454840245495120143459644//-0.21958138793119660880925891888182//-0.22179335627937390995612076949328//-0.22190575998285824721456549468712//-0.22165718137399004206145036732778//-0.22410505549482118148318932071561//-0.22312919466154310099703650394076//-0.22292459254795890521982926202327//-0.22267177270279012168963106432784//-0.22348493651412401628242321294238//-0.22320968111401393030313045073854//-0.22346586654289765649927801405283//-0.22331443870396205930717314913636//-0.22211093451402638687675050732651//-0.22283682762043030556320388768654;
#define FW_Len_B_Dec -0.22545871250033766575171512158704//-0.22683825142501001792538772861008//-0.22545871250033766575171512158704//-0.22628634317253890073651234615681//-0.22597013406889512876496439730545//-0.22514385676339976338766746266629//-0.22486936309461472149884286864108//-0.2231068004050308084540432673748//-0.22145338094826616526233920012601//-0.22174489028550473102008311343525//-0.22188496458292042423643408710632//-0.22180132392483353775958221376641//-0.22428545457774484717106133757625//-0.22318343219589042991657379388926//-0.22290789595862525729330627655145//-0.22267515746808261289935160220921//-0.22333359457770907141593852429651//-0.22331510143208546659288060709514//-0.22343660434712378504151786273724//-0.22318286483566071587425483357947//-0.22197995476332654796181031997548//-0.22269589111839205308740474720253;


#define FW_Rob_Len   0.f                    
#define FW_rob_Alpha 0.f 
// 随动�?码盘过线计数及随动轮相关结构�?
typedef struct
{
    int siCoderACur; // 当前码盘A读数 编码器的�?
    int siCoderAPre; // 上一次码盘A读数，判�?随动�?旋转方向
    int siCoderBCur; // 当前码盘B读数
    int siCoderBPre; // 上一次码盘B读数
    
		ST_POS stPos;    // 随动�?�?心坐标姿�?
} ST_FOLLOWER_WHEEL;
extern ST_FOLLOWER_WHEEL stFollowerWheel;
/*******************************************随动�?*******************************************************************/






















/*******************************************KalmanFilter*******************************************************************/

typedef struct {
    float v;          // 状态量 (速度估�??)
    float P;          // 状态协方差
    float Q;          // 过程�?�?
    float R_ins;      // �?导�?�测�?�?
    float R_whl_base; // �?速基础观测�?�?
    float slip_thres; // 打滑判断阈�? (mm/s)
    float slip_scale; // 打滑时噪声放大倍数
} KalmanFilter;
float KalmanUpdate(KalmanFilter* kf, float v_ins, float v_whl) ;
extern KalmanFilter KF_Vx;
extern KalmanFilter KF_Vy;
extern KalmanFilter KF_W;
/*******************************************KalmanFilter*******************************************************************/






/***************************************************navagation**********************************************************/
// �?动�?�航�?径的信息
typedef struct 
{
	ST_PID x;
	ST_PID y;
	ST_PID w;
}ST_Nav_Pid;

//导航标志�?
typedef enum
{
		NAV_INIT,							  // 初�?�化
    NAV_OFF,                // 四轮无输出，卸力
    NAV_LOCK,	              // 锁在当前位置，允许旋转,
	  NAV_MANUAL,
    NAV_GLOBAL_MANUAL,      // 全局坐标系手�?
	  NAV_AREA_1,             // 一区路径�?�航
		NAV_AREA_2,             // 二区�?径�?�航
	  NAV_AREA_3,             // 三区�?径�?�航
    NAV_AREA_3_RESET,       // 三区�?径�?�航重试
	  NAV_AREA_3_SINGLE,

} Nav_State; 

typedef struct
{
    // 定义�?动路径数�?，包�?位置 PID 数据
    // 包含位置控制�? PID 数据
		ST_Nav_Pid	pos_pid; //位置pid
    ST_VEL basic_velt;   // �?动路径�?�划出的�?标速度
    uint32_t run_time;         // 运�?�时�?
    uint32_t run_Sumtime;      // 总运行时�?
		uint32_t rotation_time;
		uint8_t number;				//�?动路径选择�?几个
		uint8_t number_permutation;//�?径组合选择�?几个
	  uint8_t number_permutation_assemble_set;//�?径组合选择�?几个
} ST_Auto_Path;
// 整个导航系统的主结构�?

typedef struct
{
    Nav_State nav_state;//导航系统状�?
		ST_VECTOR expect_robot_global_velt;//导航
		ST_Auto_Path auto_path;//�?动路径总结构体
} ST_Nav; // 定位 导航共用




extern ST_Nav nav;

typedef struct
{
  float a1;
	float a2;
	float t1;//单位s
	float t2;
  float alpha;
}	Speed;

typedef struct
{
	float x;
	float y;
	float q;
}POINT;
extern POINT point_end;

extern uint8_t state1;
extern uint16_t state3;
extern uint16_t state3_RESET;
extern uint16_t state3_SINGLE;
extern uint8_t Region3_Spot;

typedef struct
{
	float circle_x;
	float circle_y;
	float theta_start;
	float theta_end;
  float R;//�뾶
}Arc;//��Բ�ĺͳ�ĩ�Ƕ�ȷ��һ��Բ��

extern uint16_t spot_pre;
extern uint16_t spot;
extern POINT point_init[14];
extern u8 enter;
extern float LineAccelStep; //�ֲ�б�����벽��  
extern bool flag_lock;
extern uint8_t uphill;
extern Speed speed;
extern uint8_t uphilling;
extern float t_run;
extern u8 get_fb;
extern u8 first_up;
extern u8 flag_global_manual;
extern double Q;
extern int8_t flag_record;
extern float Ts;
extern uint8_t one_turn_state;
extern uint8_t two_turn_state;
extern int8_t way;
extern float DELTA_X, DELTA_Y, DELTA_Q;
extern float StartX, StartY, StartQ;//起�?�位�?和�?�度在�?�航�?�?频繁使用，因此放在全局变量里，避免每�?�调用函数时都�?�重新获取和计算
/***************************************************navagation**********************************************************/

#endif
