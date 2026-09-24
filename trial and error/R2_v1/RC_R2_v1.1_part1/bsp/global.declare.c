#include "global_declare.h"

//LIL_FEET_TEST
uint32_t cyt_cnt,cyt_flag,cyt_vel,K_Vel=8;
int16_t backfoot_current=0;
float backfoot_vel,K_backfoot_vel;
uint8_t backfoot_reset=0;
float RampSignal(float start,float end,int32_t cnt,int32_t length){
	if(cnt<0){
		return start;
	}else if(cnt>length){
		return end;
	}else{
		return start+(end-start)*cnt/length;
	}
}

ST_MOTOR backfoot_motor = {.motor_id = 5, .motor_type = M2006, .uiGearRatio = M3508_uiGearRatio, .ControlLoop_State = SPEED_LOOP, .Input = 0.0f, .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0, .state = 0}, .pid_inner = {.fpKp = 180.f, .fpKi = 0.005f, .fpKd = 0.002f, .fpUMax = 123000.f, .fpUpMax = 12000.f, .fpUdMax = 5000.f, .fpSumEMax = 5000.f, .fpEMax = 500.f, .fpEMin = 0.0f}, .pid_outer = {.fpKp = 5.f, .fpKi = 0.02f, .fpKd = 1.5f, .fpUMax = 400.f, .fpUpMax = 400.f, .fpUdMax = 80.f, .fpSumEMax = 500.f, .fpEMax = 500.f, .fpEMin = 0.f}, .td = {.r = 10000.f, .h = 0.001f, .T = 0.001f}};
//ST_PID backfoot_pid={.fpKp = 70.f, .fpKi = 0.2f, .fpKd = 5.f, .fpUMax = 15000.f, .fpUpMax = 15000.f, .fpUdMax = 5000.f, .fpSumEMax = 2000.f, .fpEMax = 300.f, .fpEMin = 1.f};
	
	
// 舵轮电机
ST_MOTOR leftup_motor = {.motor_id = 2, .motor_type = M6C18, .ControlLoop_State = SPEED_LOOP, .Input = 0.0f, .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0}, .pid_inner = {.fpKp = 200.f, .fpKi = 1.f, .fpKd = 20.f, .fpUMax = 14000.f, .fpUpMax = 12000.f, .fpUdMax = 10000.f, .fpSumEMax = 8000.f, .fpEMax = 500.f, .fpEMin = 1.0f}};
ST_MOTOR leftdown_motor = {.motor_id = 2, .motor_type = M6C18, .ControlLoop_State = SPEED_LOOP, .Input = 0.0f, .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0}, .pid_inner = {.fpKp = 200.f, .fpKi = 1.f, .fpKd = 20.f, .fpUMax = 14000.f, .fpUpMax = 12000.f, .fpUdMax = 10000.f, .fpSumEMax = 8000.f, .fpEMax = 500.f, .fpEMin = 1.0f}};
ST_MOTOR rightup_motor = {.motor_id = 4, .motor_type = M6C18, .ControlLoop_State = SPEED_LOOP, .Input = 0.0f, .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0}, .pid_inner = {.fpKp = 200.f, .fpKi = 1.f, .fpKd = 20.f, .fpUMax = 14000.f, .fpUpMax = 12000.f, .fpUdMax = 10000.f, .fpSumEMax = 8000.f, .fpEMax = 500.f, .fpEMin = 1.0f}};
ST_MOTOR rightdown_motor = {.motor_id = 4, .motor_type = M6C18, .ControlLoop_State = SPEED_LOOP, .Input = 0.0f, .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0}, .pid_inner = {.fpKp = 200.f, .fpKi = 1.f, .fpKd = 20.f, .fpUMax = 14000.f, .fpUpMax = 12000.f, .fpUdMax = 10000.f, .fpSumEMax = 8000.f, .fpEMax = 500.f, .fpEMin = 1.0f}};

// ST_MOTOR leftup_turn_motor = {.motor_id = 1, .motor_type = M2006,.uiGearRatio = M2006_uiGearRatio, .ControlLoop_State = SPEED_LOOP, .Input = 0.0f, .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0, .state = 0},
//     .pid_inner = {.fpKp = 180.f, .fpKi = 0.5f, .fpKd = 20.f, .fpUMax = 9000.f, .fpUpMax = 9000.f, .fpUdMax = 5000.f, .fpSumEMax = 3000.f, .fpEMax = 500.f, .fpEMin = 1.0f},
//     .pid_outer = {.fpKp = 8.f, .fpKi = 0.f, .fpKd = 2.f, .fpUMax = 150.f, .fpUpMax = 180.f, .fpUdMax = 100.f, .fpSumEMax = 1000.f, .fpEMax = 500.f, .fpEMin = 0.5f},
//     .td = {.r = 10000.f, .h = 0.001f, .T = 0.001f}};
// ST_MOTOR rightup_turn_motor = {.motor_id = 3, .motor_type = M2006,.uiGearRatio = M2006_uiGearRatio, .ControlLoop_State = SPEED_LOOP, .Input = 0.0f, .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0, .state = 0},
//     .pid_inner = {.fpKp = 180.f, .fpKi = 0.5f, .fpKd = 20.f, .fpUMax = 9000.f, .fpUpMax = 9000.f, .fpUdMax = 5000.f, .fpSumEMax = 3000.f, .fpEMax = 500.f, .fpEMin = 1.0f},
//     .pid_outer = {.fpKp = 8.f, .fpKi = 0.f, .fpKd = 2.f, .fpUMax = 150.f, .fpUpMax = 180.f, .fpUdMax = 100.f, .fpSumEMax = 1000.f, .fpEMax = 500.f, .fpEMin = 0.5f},
//     .td = {.r = 10000.f, .h = 0.001f, .T = 0.001f}};
// ST_MOTOR rightdown_turn_motor = {.motor_id = 3, .motor_type = M2006,.uiGearRatio = M2006_uiGearRatio, .ControlLoop_State = SPEED_LOOP, .Input = 0.0f, .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0, .state = 0},
//     .pid_inner = {.fpKp = 180.f, .fpKi = 0.5f, .fpKd = 20.f, .fpUMax = 9000.f, .fpUpMax = 9000.f, .fpUdMax = 5000.f, .fpSumEMax = 3000.f, .fpEMax = 500.f, .fpEMin = 1.0f},
//     .pid_outer = {.fpKp = 8.f, .fpKi = 0.f, .fpKd = 2.f, .fpUMax = 150.f, .fpUpMax = 180.f, .fpUdMax = 100.f, .fpSumEMax = 1000.f, .fpEMax = 500.f, .fpEMin = 0.5f},
//     .td = {.r = 10000.f, .h = 0.001f, .T = 0.001f}};
// ST_MOTOR leftdown_turn_motor = {.motor_id = 1, .motor_type = M2006,.uiGearRatio = M2006_uiGearRatio, .ControlLoop_State = SPEED_LOOP, .Input = 0.0f, .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0, .state = 0},
//     .pid_inner = {.fpKp = 180.f, .fpKi = 0.5f, .fpKd = 20.f, .fpUMax = 9000.f, .fpUpMax = 9000.f, .fpUdMax = 5000.f, .fpSumEMax = 3000.f, .fpEMax = 500.f, .fpEMin = 1.0f},
//     .pid_outer = {.fpKp = 8.f, .fpKi = 0.f, .fpKd = 2.f, .fpUMax = 150.f, .fpUpMax = 180.f, .fpUdMax = 100.f, .fpSumEMax = 1000.f, .fpEMax = 500.f, .fpEMin = 0.5f},
//     .td = {.r = 10000.f, .h = 0.001f, .T = 0.001f}};
ST_MOTOR leftup_turn_motor = {.motor_id = 1, .motor_type = M2006, .uiGearRatio = M2006_uiGearRatio, .ControlLoop_State = SPEED_LOOP, .Input = 0.0f, .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0, .state = 0}, .pid_inner = {.fpKp = 180.f, .fpKi = 0.005f, .fpKd = 0.f, .fpUMax = 9000.f, .fpUpMax = 9000.f, .fpUdMax = 5000.f, .fpSumEMax = 5000.f, .fpEMax = 500.f, .fpEMin = 0.0f}, .pid_outer = {.fpKp = 5.f, .fpKi = 0.f, .fpKd = 1.5f, .fpUMax = 400.f, .fpUpMax = 400.f, .fpUdMax = 80.f, .fpSumEMax = 500.f, .fpEMax = 500.f, .fpEMin = 0.f}, .td = {.r = 10000.f, .h = 0.001f, .T = 0.001f}};
ST_MOTOR rightup_turn_motor = {.motor_id = 3, .motor_type = M2006, .uiGearRatio = M2006_uiGearRatio, .ControlLoop_State = SPEED_LOOP, .Input = 0.0f, .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0, .state = 0}, .pid_inner = {.fpKp = 180.f, .fpKi = 0.005f, .fpKd = 0.f, .fpUMax = 9000.f, .fpUpMax = 9000.f, .fpUdMax = 5000.f, .fpSumEMax = 5000.f, .fpEMax = 500.f, .fpEMin = 0.0f}, .pid_outer = {.fpKp = 5.f, .fpKi = 0.f, .fpKd = 1.5f, .fpUMax = 400.f, .fpUpMax = 400.f, .fpUdMax = 80.f, .fpSumEMax = 500.f, .fpEMax = 500.f, .fpEMin = 0.f}, .td = {.r = 10000.f, .h = 0.001f, .T = 0.001f}};
ST_MOTOR rightdown_turn_motor = {.motor_id = 3, .motor_type = M2006, .uiGearRatio = M2006_uiGearRatio, .ControlLoop_State = SPEED_LOOP, .Input = 0.0f, .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0, .state = 0}, .pid_inner = {.fpKp = 180.f, .fpKi = 0.005f, .fpKd = 0.f, .fpUMax = 9000.f, .fpUpMax = 9000.f, .fpUdMax = 5000.f, .fpSumEMax = 5000.f, .fpEMax = 500.f, .fpEMin = 0.0f}, .pid_outer = {.fpKp = 5.f, .fpKi = 0.f, .fpKd = 1.5f, .fpUMax = 400.f, .fpUpMax = 400.f, .fpUdMax = 80.f, .fpSumEMax = 500.f, .fpEMax = 500.f, .fpEMin = 0.f}, .td = {.r = 10000.f, .h = 0.001f, .T = 0.001f}};
ST_MOTOR leftdown_turn_motor = {.motor_id = 1, .motor_type = M2006, .uiGearRatio = M2006_uiGearRatio, .ControlLoop_State = SPEED_LOOP, .Input = 0.0f, .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0, .state = 0}, .pid_inner = {.fpKp = 180.f, .fpKi = 0.005f, .fpKd = 0.f, .fpUMax = 9000.f, .fpUpMax = 9000.f, .fpUdMax = 5000.f, .fpSumEMax = 5000.f, .fpEMax = 500.f, .fpEMin = 0.0f}, .pid_outer = {.fpKp = 5.f, .fpKi = 0.f, .fpKd = 1.5f, .fpUMax = 400.f, .fpUpMax = 400.f, .fpUdMax = 80.f, .fpSumEMax = 500.f, .fpEMax = 500.f, .fpEMin = 0.f}, .td = {.r = 10000.f, .h = 0.001f, .T = 0.001f}};

float leftup_init_angle = 0.f, rightup_init_angle = 0.f, leftdown_init_angle = 0.f, rightdown_init_angle = 0.f;
uint8_t leftup_init_flag = 0, rightup_init_flag = 0, leftdown_init_flag = 0, rightdown_init_flag = 0;
// 舵轮电机

// 小脚电机
J60_MotorDATA j60_motor_data_down;
J60_MotorCMD j60_motor_cmd_down = {.motor_id_ = 1, .position_ = 0.0f, .velocity_ = 0.0f, .torque_ = 0.0f, .kp_ = 0.0f, .kd_ = 0.0f, .j60_td = {.r = 10000000, .h = 0.001, .T = 0.001}};
uint8_t j60_Tx[8];

MotorCmd_t go1_send_left = {.id = 0x02, .mode = 1, .T = 0.f, .W = 0.f, .Pos = 0.f, .K_P = 0.f, .K_W = 0.f, .td = {.r = 20000.f, .h = 0.001f, .T = 0.001f}};
MotorData_t go1_recv_left;
MotorCmd_t go1_send_right = {.id = 0x01, .mode = 1, .T = 0.f, .W = 0.f, .Pos = 0.f, .K_P = 0.f, .K_W = 0.f, .td = {.r = 20000.f, .h = 0.001f, .T = 0.001f}};
MotorData_t go1_recv_right;
RIS_MotorData_t uart_rx_data;
float go1_left_0 = 0.f, go1_right_0 = 0.f;
uint8_t go1_left_init_flag = 0, go1_right_init_flag = 0;
// 小脚电机

ST_ROBOT stRobot;

float LineAccelStep = 20.f;

// 反馈速度滤波
ST_WHEEL_ENCODER_VELT_FILTER wheel_encoder_velt_filter = {
	.leftup_velt = {0, 0, 0, 2000, 0.001},
	.rightup_velt = {0, 0, 0, 2000, 0.001},
	.rightdown_velt = {0, 0, 0, 2000, 0.001},
	.leftdown_velt = {0, 0, 0, 2000, 0.001}};

// 底盘速度分配
ST_VECTOR expect_robot_local_Velt;
ST_Chassis_Run chassis_run = {
	.leftup = {.fpKp = 110.f, .fpKi = 0.f, .fpKd = 0.f, .fpUMax = 15000.f, .fpUpMax = 15000.f, .fpUdMax = 5000.f, .fpSumEMax = 2000.f, .fpEMax = 300.f, .fpEMin = 1.f},
	.rightup = {.fpKp = 110.f, .fpKi = 0.f, .fpKd = 0.f, .fpUMax = 15000.f, .fpUpMax = 15000.f, .fpUdMax = 5000.f, .fpSumEMax = 2000.f, .fpEMax = 300.f, .fpEMin = 1.f},
	.leftdown = {.fpKp = 110.f, .fpKi = 0.f, .fpKd = 0.f, .fpUMax = 15000.f, .fpUpMax = 15000.f, .fpUdMax = 5000.f, .fpSumEMax = 2000.f, .fpEMax = 300.f, .fpEMin = 1.f},
	.rightdown = {.fpKp = 110.f, .fpKi = 0.f, .fpKd =0.f, .fpUMax = 15000.f, .fpUpMax = 15000.f, .fpUdMax = 5000.f, .fpSumEMax = 2000.f, .fpEMax = 300.f, .fpEMin = 1.f},
};
chassis_run_des steer_velt, steer_pos, steer_pos_pre, friction_feedforward, cur_steer_velt;
chassis_run_des fric_k_1 = {.leftup = 15.f, .rightup = 15.f, .leftdown = 15.f, .rightdown = 15.f};
chassis_run_des fric_k_2 = {.leftup = 35.f, .rightup = 35.f, .leftdown = 35.f, .rightdown = 35.f};
float v_x, v_y, v_w;

// 小脚
FOOT foot = {.foot_state = -1, .foot_up = UP1, .foot_down = DOWN1, .up_tor_feedforward_flag = 0, .down_tor_feedforward_flag = 0, .up_foot_circle_count = 0};
FOOT_G_feedforward foot_g_feedforward = {.leftup = 0.f, .rightup = 0.f, .down = 0.f};
uint8_t foot_up_G_feedforward_flag = 0, foot_down_G_feedforward_flag = 0;
uint8_t foot_motor_runtime_flag = 0;
int foot_motor_runtime = 0;

// DT35
uint16_t dt35_distance[5];
ST_DT35 dt35_now;

//陀螺仪

ST_GYRO gyro;

// 行程开关 舵轮转向电机找零点
uint8_t travel_switch;
uint8_t travel_switch_mode[8];

// 遥控器
uint8_t FLAG_NRF;
ST_JS_VALUE Js_Value = {.usJsLeft_X = LEFT_JS_X_MID, .usJsLeft_Y = LEFT_JS_Y_MID, .usJsRight_X = RIGHT_JS_MID};
uint16_t RC_Key_pre_value = 0;

long long int tim_ms = 0;
long long int time_cnt, time_cnt_pre;
ST_SYSTEM_MONITOR monitor;
float vofa[20] = {0};

// 状态机
volatile UP_D_STATE up_d_state;
volatile UP_S_STATE up_s_state = -1;
DOWN_ROBOT_STATE down_robot_state;
uint8_t down_flag;

// 视觉
uint8_t vision_rec[62]; // 接收
uint8_t vision_tx[3];	// 发送
VISION_DATA_AUTO vision_data_recieve;
Location_Filter location_filter = {.x = {0}, .y = {0}, .yaw = {0}, .lpf_k = 0.1f, .tim = 0, .flag = 1};
BLOCK_POSITION block_position;
BLOCK_STATE block_state;
uint8_t flag_path_1 = 1;	 // 1前往取武器   2前往梅林
uint8_t part_over_flag = -1; // 1执行一区  2执行二区123  3执行2区  4执行3区
uint8_t flag_part_2 = 1;	 // 1判断转弯  2是否取块  3上下台阶 4转弯
bool flag_turn = 0;
uint8_t turn_action = 0; // 0中1左2右
uint8_t judge_123 = 0;
uint8_t flag_part_3;
uint8_t flag_vision_update = 0;
// 导航
ST_Nav nav =
	{
		.nav_state = 0,
		.auto_path =
			{
				.up_down_state = UP_DOWN_STATE_OFF,
				.pos_pid =
					{
						.x =
							{
								.fpKp = 4.f,
								.fpKi = 0.03f,
								.fpKd = 0.2f,

								.fpUMax =50000.f,
								.fpUpMax = 50000.f,
								.fpUiMax = 2000.f,
								.fpUdMax = 50.f,
								.fpSumEMax = 3000.f,
								.fpElimit = 100.f,
								.fpEforID = 7.f,

								.fpEMax = 2000,
								.fpEMin = 1.f

							},
						.y =
							{
								.fpKp = 4.f,
								.fpKi = 0.03f,
								.fpKd = 0.2f,

								.fpUMax = 50000.f,
								.fpUpMax = 50000.f,
								.fpUiMax = 2000.f,
								.fpUdMax = 50.f,
								.fpSumEMax = 3000.f,
								.fpElimit = 100.f,
								.fpEforID = 7.f,

								.fpEMax = 2000,
								.fpEMin = 1.f},
						.w =
							{
								.fpKp = 4.f,
								.fpKi = 0.01f,
								.fpKd = 0.f,

								.fpUMax = 250.f,
								.fpUpMax = 300.f,
								.fpUiMax = 350.f,
								.fpUdMax = 20.f,
								.fpSumEMax = 2000.f,
								.fpElimit = 10.f,
								.fpEforID = 0.1f,

								.fpEMax = 200,
								.fpEMin = 0.1f

							},
						.td_x = {.r = 100000.f, .h = 0.001f, .T = 0.001f},
						.td_y = {.r = 100000.f, .h = 0.001f, .T = 0.001f},
						.td_w = {.r = 1000.f, .h = 0.001f, .T = 0.001f}},
			},
};
float pos_x, pos_y, yaw;

Path_State path_state;
bool flag_lock = 0;
uint32_t TIM2_CNT = 0;	 // ms
uint32_t TIM2_CNT_1 = 0; // s
// 自动路径结构体
PATH_PERMUTATION Path_Permuta;
PATH_POINT Path_Point;
int point_cnt = 0;
uint8_t flag_point_permutation = 1;
uint8_t flag_permutation_path = 1; // 组合路径参数初始化标志位
uint8_t flag_point_to_point = 1;   // 微调选择路径标志位
uint8_t flag_choose = 1;
uint8_t flag_zero = 1;
uint8_t flag_rotation = 0;
uint8_t flag_point_end = 0;
uint32_t point_tim = 0;
uint8_t flag_point_block = 0;
uint8_t flag_part_2_to_3 = 1;
uint8_t flag_2_begin = 1;
uint8_t flag_3_to_R1 = 1;
uint8_t flag_part_3_begin = 1;
uint8_t flag_part_3_begin_2 = 1;
uint8_t flag_part_3_retry = 1;
uint8_t flag_3_choose=1;
// 板间通信
uint8_t down_tx[18]; // 88 66
uint8_t down_rx[18]; // 33 55
uint8_t tx_complete = 1;

//电控路径规划逻辑
STEP_STATE step_state;//3行4列，(x-1)*3+y=id//id=1~12
uint8_t vision_judge[12]={0};//judge=0 无//judge=1 有R2//judge=2 假//judge=3 有R1
ROUTE_PLAN route_plan;
uint8_t flag_route_plan=0;
uint8_t flag_path_to_zero=0;
uint8_t flag_use_vision=0;
