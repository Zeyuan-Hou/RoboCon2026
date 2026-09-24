#include "Types.h"

/********************************************气动板************************************************************/
uint8_t AirCtrl_1[3];
uint8_t AirCtrl_2[1];
uint8_t AirCtrl_3[1];

uint8_t DT35_Data[8] = {0};			 // 从气动板接收，通过串口一发送给下层板
uint8_t AirOperaterCtrlBuf[6] = {0}; // 控制电磁阀，[0]-[5]对应电磁阀开关1-6，分别对应取块机械臂气泵，车屁股存块气泵，车侧边存块气泵，取块机械臂电磁阀,存杆电磁阀,取杆电磁阀

/********************************************气动板************************************************************/

/*******************************************机器人总结构体*******************************************************************/
ST_ROBOT stRobot;
ST_ROBOT test_stRobot;
/*******************************************机器人总结构体*******************************************************************/

/*******************************************system_monitor*******************************************************************/
// system_monitor的计数器初始化
float tim_sum_monitor = 0;
ST_SYSTEM_MONITOR system_monitor;
/*******************************************system_monitor*******************************************************************/

/*******************************************nRF24L01*******************************************************************/
ST_JS_VALUE Js_Value;
uint8_t nRF24L01_Tick = 0;		  // 遥控器tick（freertos要用到，不然频率是1000hz）
ACK_PAYLOAD nRF24L01_ack_pay;	  // 应答信号ACK数据包
uint8_t nRF24L01_RxBuf[32] = {0}; // 接收数据缓存
uint16_t Uart4_Receive_length = 0;
uint8_t REMOTE_BUFFER[3] = {0}; // 发回遥控器的数据缓存，内部东西打包到ack_payload里用于发回
/*******************************************nRF24L01*******************************************************************/

/*******************************************底盘电机*******************************************************************/
ST_Chassis_Run chassis_run =
	{
		.leftup =
			{
				.fpKp = 70.f,
				.fpKi = 0.6f,
				.fpKd = 5.f,

				.fpUMax = 12000.f,
				.fpUpMax = 12000.f,
				.fpUdMax = 1000.f,
				.fpSumEMax = 5000.f,

				.fpEMax = 200.f,
				.fpEMin = 0.f,
				.feedforward = 120.f,
				.forward_rub = 2800.f,
				.LPF =
					{
						.error_threshold = 10.f,
					}},
		.rightup =
			{
				.fpKp = 70.f,
				.fpKi = 0.6f,
				.fpKd = 5.f,

				.fpUMax = 12000.f,
				.fpUpMax = 12000.f,
				.fpUdMax = 1000.f,
				.fpSumEMax = 5000.f,

				.fpEMax = 200.f,
				.fpEMin = 0.f,
				.feedforward = 120.f,
				.forward_rub = 1500.f,
				.LPF =
					{
						.error_threshold = 10.f,
					}},
		.leftdown =
			{
				.fpKp = 70.f,
				.fpKi = 0.6f,
				.fpKd = 5.f,

				.fpUMax = 12000.f,
				.fpUpMax = 12000.f,
				.fpUdMax = 1000.f,
				.fpSumEMax = 5000.f,

				.fpEMax = 200.f,
				.fpEMin = 0.f,
				.feedforward = 120.f,
				.forward_rub = 2300.f,
				.LPF =
					{
						.error_threshold = 10.f,
					}},
		.rightdown =
			{
				.fpKp = 70.f,
				.fpKi = 0.6f,
				.fpKd = 5.f,

				.fpUMax = 12000.f,
				.fpUpMax = 12000.f,
				.fpUdMax = 1000.f,
				.fpSumEMax = 5000.f,

				.fpEMax = 200.f,
				.fpEMin = 0.f,
				.feedforward = 120.f,
				.forward_rub = 600.f,
				.LPF =
					{
						.error_threshold = 10.f,
					}},
		.pid_state = VELT_LOOP,
		.feed_forward_state = WITHOUT_FORWARD};

MOTOR leftup_motor = {.motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0}};
MOTOR leftdown_motor = {.motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0}};
MOTOR rightup_motor = {.motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0}};
MOTOR rightdown_motor = {.motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0}};
/*******************************************底盘电机*******************************************************************/

/*******************************************上下板通信*****************************************/
uint8_t uart1_rx_buff[6] = {0};
uint8_t Usart1_Receive_length = 0;
uint8_t uart1_tx_buffer[26] = {0};
uint8_t pre_key = 0;
uint16_t RC_Key_pre_value = 0;

/*******************************************上下板通信*****************************************/

/*******************************************随动轮*******************************************************************/
float degreeA = 0;
float degreeB = 0;
ST_FOLLOWER_WHEEL stFollowerWheel;
/*******************************************随动轮*******************************************************************/

/*******************************************陀螺仪*******************************************************************/
// 陀螺仪收到的数据
uint8_t g_Usart2_Rx_buf[17];
uint8_t g_Usart2_Tx_buf[1] = {1};
uint16_t g_Usart2_Rx_cnt = 0;
bool gyro_reset_flag = 1;

uint8_t g_Decode_Data[512];
uint16_t g_Decode_Data_pos = 0;

GYRO_DATA gyro_data = {0};
ST_GYRO Gyro_Data_Test = {0};

int num_circle; // 初始化已经旋转的圈数，用于方便计算角度
fp32 fpSumPosQ;
uint16_t gyro_tick = 0;
/*******************************************陀螺仪*******************************************************************/

/*******************************************定位*******************************************************************/
/*******************************************雷达*********************************************/
uint8_t Radar_RxBuf[38] = {0};
ST_VISION_DATA Vision_Data = {0};
uint8_t vision_tx[3]; // 发送
Location_Filter location_filter = {.x = {0}, .y = {0}, .yaw = {0}, .lpf_k = 0.1f, .tim = 0, .flag = 1};
/*******************************************雷达*********************************************/

fp32 fpPosXOffset = 0; // X方向总纠偏量，会被一直使用
fp32 fpPosYOffset = 0; // Y方向总纠偏量，会被一直使用

fp32 fpQOffset = 0; // 角度Q纠偏量，勿动 调整机器人初始姿态或补偿陀螺仪的系统误差。

uint8_t flag_DT35_locate = 0;
float DT35_y = 0;
float DT35_x = 0;
float DT35_x_ = 0;
float DT35_y_ = 0;
float LENGTH_Y = 0;
float LENGTH_X = 0;

uint8_t DT35_xFlag = 0;
uint8_t DT35_x_Flag = 0;
uint8_t DT35_yFlag = 0;
uint8_t DT35_y_Flag = 0;
bool DT35_Correct_Flag = 0; // 这个是用于手操重新刷新DT35数据的标志位
bool DT35_Turn_Flag = 0;	// 这个是用于判断车体是否进入人工给予的拐弯导航内的标志位，=1的时候不刷新DT35
ST_REGION_T REGION_STATE = OTHERS;
ST_DT35_GYRO_T DT35_GYRO_STATE = FACE_0;
uint8_t region3_state = 0; // 上完坡进入平地后判断为1，定位刷新后变2
uint8_t REGION3_temp = 0;  // 在三区上坡时检测到俯仰角变化变成1，定位刷新后变2
uint8_t FLAG_REGION3 = 0;
uint8_t FLAG_REGION3_manual = 0;

float DT35_Delta[4] = {0};
float DT35_X1_Count = 0;
float DT35_Y1_Count = 0;
float DT35_X2_Count = 0;
float DT35_Y2_Count = 0;

DT35_distanceT DT35_distance_pre = {0};

// 开始时的X,Y轴坐标，都设为0
fp32 fpStartX = 0; // 5557.5f;//319
fp32 fpStartY = 0; // 545.0f;//361

ST_WHEEL2BODY_VELT Wheelvelt_To_Bodyvelt =
	{
		.Vx = {0, 0, 0, 50, 0.001},
		.Vy = {0, 0, 0, 50, 0.001},
		.W = {0, 0, 0, 50, 0.001}};
ST_TD posX_veltX =
	{
		.m_r = 5000,
		.m_h = 0.001,
		.m_T = 0.001};

ST_TD posY_veltY =
	{
		.m_r = 5000,
		.m_h = 0.001,
		.m_T = 0.001};

ST_TD posW_veltW =
	{
		.m_r = 5000,
		.m_h = 0.001,
		.m_T = 0.001};
ST_GLOBAL_VELT_FILTER global_velt_filter =
	{
		.global_vx = {0, 0, 0, 50, 0.001},
		.global_vy = {0, 0, 0, 50, 0.001},
		.global_w = {0, 0, 0, 50, 0.001}};

ST_TD td_Vx =
	{
		.m_x1 = 0,
		.m_x2 = 0,
		.m_x = 0,
		.m_r = 5000,
		.m_T = 0.001,
		.m_h = 0.001};

ST_TD td_Vy =
	{
		.m_x1 = 0,
		.m_x2 = 0,
		.m_x = 0,
		.m_r = 5000,
		.m_T = 0.001,
		.m_h = 0.001};

ST_LPF lpf_Vx = {0, 0, 0, 50, 0.001};
ST_LPF lpf_Vy = {0, 0, 0, 50, 0.001};

// DT35定位数据结构体
DT35_distanceT DT35_distance = {0}; // 这个是用原始数据算出实际数据的 {x1，y1，x2，y2}单位是mm
DT35_distanceT DT35_dis = {0};		// 这个是拿来接收原始数据的

int16_t length_differx_1 = 0;
int16_t length_differy_1 = 0;
int16_t length_differx_2 = 0;
int16_t length_differy_2 = 0;
/*******************************************定位*******************************************************************/

/*******************************************KalmanFilter*******************************************************************/
KalmanFilter KF_Vx =
	{
		.v = 0.f,
		.P = 1.0f,
		.Q = 0.01f,
		.R_ins = 0.1f,
		.R_whl_base = 0.05f,
		.slip_thres = 150.0f,
		.slip_scale = 10.f};
KalmanFilter KF_Vy =
	{
		.v = 0.f,
		.P = 1.0f,
		.Q = 0.01f,
		.R_ins = 0.1f,
		.R_whl_base = 0.05f,
		.slip_thres = 150.0f,
		.slip_scale = 10.f};
KalmanFilter KF_W =
	{
		.v = 0.f,
		.P = 1.0f,
		.Q = 0.01f,
		.R_ins = 0.1f,
		.R_whl_base = 0.05f,
		.slip_thres = 15.0f,
		.slip_scale = 10.f};
/*******************************************KalmanFilter*******************************************************************/

/*******************************************navigation*******************************************************************/
ST_Nav nav =
	{
		.auto_path.pos_pid.x =
			{
				.fpKp = 4.f,	 // 1.5f,//5.f,
				.fpKi = 0.0015f, // 0.0008f,
				.fpKd = 0.4f,

				.fpUMax = 2000.f,
				.fpUpMax = 1000.f,
				.fpUdMax = 500.f,
				.fpSumEMax = 20000.f,

				.fpEMax = 500.f,
				.fpEMin = 2.f},
		.auto_path.pos_pid.y =
			{
				.fpKp = 4.f, // 1.5f,//2.f,//5.f,
				.fpKi = 0.00015f,
				.fpKd = 0.4f,

				.fpUMax = 2000.f,
				.fpUpMax = 1000.f,
				.fpUdMax = 500.f,
				.fpSumEMax = 20000.f,

				.fpEMax = 500.f,
				.fpEMin = 2.f},
		.auto_path.pos_pid.w =
			{
				.fpKp = 4.f, // 5.2f,//12.f,
				.fpKi = 0.001f,
				.fpKd = 2.f,

				.fpUMax = 90.f,
				.fpUpMax = 90.f,
				.fpUdMax = 40.f,
				.fpSumEMax = 2000.f,

				.fpEMax = 90.f,
				.fpEMin = 0.4f}};

bool flag_lock = 1;

POINT point_end = {0, 0, 0}; // 用于点对点走路径
float Ts = 0.001;

float DELTA_X = 0, DELTA_Y = 0, DELTA_Q = 0; // 路径规划过程中目标点和当前位置的差值，单位是mm和度

uint8_t uphill = 0;													  //  标志是否已经完成上坡，已经完成上坡为1
uint8_t uphilling = 0;												  // 标志是否正在上坡，正在上坡为1
Speed speed = {.a1 = 2000, .t1 = 2, .t2 = 0, .a2 = 4000, .alpha = 0}; //  alpha=0，向x正方向走

uint8_t state1 = 0; //  在一区导航的阶段

uint16_t state3 = 0;
uint16_t state3_RESET = 0;
uint16_t state3_SINGLE = 0;

uint8_t Region3_Spot = 2;
uint8_t one_turn_state = 0; // 为0时不启动，用的时候赋值为1，用完归0
uint8_t two_turn_state = 0; // 为0时不启动，用的时候赋值为1，用完归0

u8 flag_global_manual = 1; // 用于在进入全局手操的时候锁住当前角度

float t_run = 0;		  // 每次路径规划完成之后清零，在导航过程中不断增加，直到再次路径规划了才会清零
float LineAccelStep = 10; // 手操斜坡输入步长
u8 first_up = 0;		  // 用于上坡的第一次加速，第一次加速之后这个值就变成1了
u8 get_fb = 0;			  // 没记录的时候是0，记录完成之后是1					        // 进入三区的时候记录初始位置
double Q;
int8_t flag_record = 1; // 主要用于每次清空一下运行时间nav的run_time，1代表要记录初始位置，0代表不记录初始位置，在路径规划的过程中不更新初始位置，直到完成一个路径或者切换到其他模式了才会再次记录初始位置

int8_t way = 0; // 1表示顺时针，-1表示逆时针

float StartX, StartY, StartQ = 0; // 起始位置和角度在导航中被频繁使用，因此放在全局变量里，避免每次调用函数时都要重新获取和计算
/*******************************************navigation*******************************************************************/

/***************************************起重机3508***************************************************/

ST_MOTORT Crane_3508_1 = {.motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0, .state = 1},
						  .motor_pid.outer = {.fpEMin = 1, .fpSumEMax = 20000, .fpUMax = 600, .fpUpMax = 12000.f, .fpUdMax = 4000.f, .fpEMax = 10000, .fpKp = 5, .fpKi = 0.002, .fpKd = 0},
						  .motor_pid.inner = {.fpEMin = 1, .fpSumEMax = 600, .fpUMax = 8000, .fpUpMax = 8000.f, .fpUdMax = 4000.f, .fpEMax = 100000, .fpKp = 120, .fpKi = 1, .fpKd = 0}};

ST_MOTORT Crane_3508_2 = {.motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0, .state = 1},
						  .motor_pid.outer = {.fpEMin = 1, .fpSumEMax = 20000, .fpUMax = 600, .fpUpMax = 12000.f, .fpUdMax = 4000.f, .fpEMax = 10000, .fpKp = 5, .fpKi = 0.002, .fpKd = 0},
						  .motor_pid.inner = {.fpEMin = 1, .fpSumEMax = 600, .fpUMax = 8000, .fpUpMax = 8000.f, .fpUdMax = 4000.f, .fpEMax = 100000, .fpKp = 120, .fpKi = 1, .fpKd = 0}};

ST_TD Crane_3508_TD_1 = {.m_h = 0.1, .m_r = 12000, .m_T = 0.001};
ST_TD Crane_3508_TD_2 = {.m_h = 0.1, .m_r = 12000, .m_T = 0.001};

CRANE_STATE Crane_State = Crane_Init; // 起重机状态标志位
float Input_3508 = 0;
uint16_t Crane_Time = 2; // 起重机走一段需要的时间，单位秒
/***************************************起重机3508***************************************************/
