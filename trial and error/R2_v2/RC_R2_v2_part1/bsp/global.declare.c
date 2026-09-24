#include "global_declare.h"

// 舵轮电机
ST_MOTOR leftup_motor = {.motor_id = 2, .motor_type = M4219, .ControlLoop_State = SPEED_LOOP, .Input = 0.0f, .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0}, .pid_inner = {.fpKp = 200.f, .fpKi = 1.f, .fpKd = 20.f, .fpUMax = 14000.f, .fpUpMax = 12000.f, .fpUdMax = 10000.f, .fpSumEMax = 8000.f, .fpEMax = 500.f, .fpEMin = 0.0f}};
ST_MOTOR rightup_motor = {.motor_id = 4, .motor_type = M4219, .ControlLoop_State = SPEED_LOOP, .Input = 0.0f, .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0}, .pid_inner = {.fpKp = 200.f, .fpKi = 1.f, .fpKd = 20.f, .fpUMax = 14000.f, .fpUpMax = 12000.f, .fpUdMax = 10000.f, .fpSumEMax = 8000.f, .fpEMax = 500.f, .fpEMin = 0.0f}};
ST_MOTOR down_motor = {.motor_id = 2, .motor_type = M4219, .ControlLoop_State = SPEED_LOOP, .Input = 0.0f, .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0}, .pid_inner = {.fpKp = 200.f, .fpKi = 1.f, .fpKd = 20.f, .fpUMax = 14000.f, .fpUpMax = 12000.f, .fpUdMax = 10000.f, .fpSumEMax = 8000.f, .fpEMax = 500.f, .fpEMin = 0.0f}};

ST_MOTOR leftup_turn_motor = {.motor_id = 1, .motor_type = M2006, .uiGearRatio = M2006_uiGearRatio, .ControlLoop_State = SPEED_LOOP, .Input = 0.0f, .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0, .state = 0}, 
	.pid_inner = {.fpKp = 200.f, .fpKi = 0.005f, .fpKd = 0.002f, .fpUMax = 9000.f, .fpUpMax = 9000.f, .fpUdMax = 5000.f, .fpSumEMax = 5000.f, .fpEMax = 500.f, .fpEMin = 0.0f}, 
	.pid_outer = {.fpKp = 5.f, .fpKi = 0.02f, .fpKd = 1.5f, .fpUMax = 400.f, .fpUpMax = 400.f, .fpUdMax = 80.f, .fpSumEMax = 500.f, .fpEMax = 500.f, .fpEMin = 0.f}, 
	.td = {.r = 10000.f, .h = 0.001f, .T = 0.001f}};
ST_MOTOR rightup_turn_motor = {.motor_id = 3, .motor_type = M2006, .uiGearRatio = M2006_uiGearRatio, .ControlLoop_State = SPEED_LOOP, .Input = 0.0f, .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0, .state = 0}, 
	.pid_inner = {.fpKp = 200.f, .fpKi = 0.005f, .fpKd = 0.002f, .fpUMax = 9000.f, .fpUpMax = 9000.f, .fpUdMax = 5000.f, .fpSumEMax = 5000.f, .fpEMax = 500.f, .fpEMin = 0.0f}, 
	.pid_outer = {.fpKp = 5.f, .fpKi = 0.02f, .fpKd = 1.5f, .fpUMax = 400.f, .fpUpMax = 400.f, .fpUdMax = 80.f, .fpSumEMax = 500.f, .fpEMax = 500.f, .fpEMin = 0.f}, 
	.td = {.r = 10000.f, .h = 0.001f, .T = 0.001f}};
ST_MOTOR down_turn_motor = {.motor_id = 1, .motor_type = M2006, .uiGearRatio = M2006_uiGearRatio, .ControlLoop_State = SPEED_LOOP, .Input = 0.0f, .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0, .state = 0}, 
	.pid_inner = {.fpKp = 200.f, .fpKi = 0.005f, .fpKd = 0.002f, .fpUMax = 9000.f, .fpUpMax = 9000.f, .fpUdMax = 5000.f, .fpSumEMax = 5000.f, .fpEMax = 500.f, .fpEMin = 0.0f}, 
	.pid_outer = {.fpKp = 5.f, .fpKi = 0.02f, .fpKd = 1.5f, .fpUMax = 400.f, .fpUpMax = 400.f, .fpUdMax = 80.f, .fpSumEMax = 500.f, .fpEMax = 500.f, .fpEMin = 0.f}, 
	.td = {.r = 10000.f, .h = 0.001f, .T = 0.001f}};
ST_MOTOR foot_motor = {.motor_id = 3, .motor_type = M2006, .uiGearRatio = M2006_uiGearRatio, .ControlLoop_State = SPEED_LOOP, .Input = 0.0f, .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0, .state = 0}, 
	.pid_inner = {.fpKp = 300.f, .fpKi = 0.1f, .fpKd = 0.002f, .fpUMax = 9000.f, .fpUpMax = 8500.f, .fpUdMax = 5000.f, .fpSumEMax = 10000.f, .fpEMax = 500.f, .fpEMin = 0.0f}, 
	.pid_outer = {.fpKp = 5.f, .fpKi = 0.02f, .fpKd = 1.5f, .fpUMax = 400.f, .fpUpMax = 400.f, .fpUdMax = 80.f, .fpSumEMax = 500.f, .fpEMax = 500.f, .fpEMin = 0.f}, 
	.td = {.r = 10000.f, .h = 0.001f, .T = 0.001f}};

float leftup_init_angle = 0.f, rightup_init_angle = 0.f, down_init_angle = 0.f;
uint8_t leftup_init_flag = 0, rightup_init_flag = 0, down_init_flag = 0;
// 舵轮电机

// 小脚电机
J60_MotorDATA j60_motor_data_down1, j60_motor_data_down2;
J60_MotorCMD j60_motor_cmd_down1 = {.motor_id_ = 1, .position_ = 0.0f, .velocity_ = 0.0f, .torque_ = 0.0f, .kp_ = 0.0f, .kd_ = 0.0f, .j60_td = {.r = 10000000, .h = 0.001, .T = 0.001}};
J60_MotorCMD j60_motor_cmd_down2 = {.motor_id_ = 1, .position_ = 0.0f, .velocity_ = 0.0f, .torque_ = 0.0f, .kp_ = 0.0f, .kd_ = 0.0f, .j60_td = {.r = 10000000, .h = 0.001, .T = 0.001}};
uint8_t j60_Tx[8];
ST_CASCADE_PID j60_pid = {.inner = {.fpKp = 15.f, .fpKi = 0.f, .fpKd = 2.f, .fpUMax = 20.f, .fpUpMax = 20.f, .fpUdMax = 6.f, .fpSumEMax = 500.f, .fpEMax = 10.f, .fpEMin = 0.0f}, 
	.outer = {.fpKp = 5.f, .fpKi = 0.f, .fpKd = 0.f, .fpUMax = 3.f, .fpUpMax = 3.f, .fpUdMax = 6.f, .fpSumEMax = 100.f, .fpEMax = 20.f, .fpEMin = 0.f}, 
	.td = {.r = 1000.f, .h = 0.001f, .T = 0.001f},
	.final_fpU = 0.f};

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
	.down_velt = {0, 0, 0, 2000, 0.001}};

// 底盘速度分配
ST_VECTOR expect_robot_local_Velt;
chassis_run_des steer_velt, steer_pos, steer_pos_pre, friction_feedforward, cur_steer_velt;
chassis_run_des fric_k = {.leftup = 8.f, .rightup = 8.f, .down = 8.f};
ST_Chassis_Run2 chassis_run2 = {
	.leftup = {.fpKp = 70.f, .fpKi = 0.1f, .fpUMax = 15000.f, .fpUpMax = 10000.f, .fpUiMax = 1000.f, .fpKffv = 15.f, .fpKffa = 1000.f, .fpUffMax = 10000.f, .fpEMax = 300.f},
	.rightup = {.fpKp = 75.f, .fpKi = 0.1f, .fpUMax = 15000.f, .fpUpMax = 10000.f, .fpUiMax = 1000.f, .fpKffv = 16.f, .fpKffa = 1100.f, .fpUffMax = 10000.f, .fpEMax = 300.f},
	.down = {.fpKp = 70.f, .fpKi = 0.1f, .fpUMax = 15000.f, .fpUpMax = 10000.f, .fpUiMax = 1000.f, .fpKffv = 15.f, .fpKffa = 1000.f, .fpUffMax = 10000.f, .fpEMax = 300.f},
	.feed_forward_state = WITHOUT_FORWARD};

// 小脚
FOOT foot = {.foot_state = 255, .foot_up_lv1 = UP11, .foot_up_lv2 = UP21, .foot_down_lv1 = DOWN11, .foot_down_lv2 = DOWN21, .up_tor_feedforward_flag = 0, .down_tor_feedforward_flag = 0, .up_foot_circle_count = 0, .runtime_flag = 0, .runtime = 0};
FOOT_G_feedforward foot_g_feedforward = {.leftup = 0.f, .rightup = 0.f, .down = 0.f};
uint8_t foot_up_G_feedforward_flag = 0, foot_down_G_feedforward_flag = 0; // 是否额外改变参数，比如携带方块或上层姿态改变时

// DT35
// uint16_t dt35_distance[5];
// ST_DT35 dt35_now;

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

// // 视觉
uint8_t vision_rec[34]; // 接收
VISION_DATA_AUTO vision_data_recieve;
Location_Filter location_filter = {.x = {0}, .y = {0}, .yaw = {0}, .lpf_k = 1.f, .tim = 0, .flag = 1};

// // 导航
ST_Nav nav = {
	.nav_state = 255,
	.auto_path = {
		.up_down_state = UP_DOWN_STATE_OFF,
		.pos_pid = {
			.x = {.kp_init = 2.2f, .upmax_init = 1500.f, .kp_boost = 1.5f, .upmax_boost = 1.2f, .const_far = 1500.f, .const_near = 300.f, 
				.fpKi = 0.0025f, .fpKd = 0.f, .fpPreE = 0.f, .fpSumE = 0.f, .fpEMin = 50.f, .fpEMax = 2000.f, .fpUMax = 4000.f, .fpUiMax = 60.f, .fpUdMax = 500.f},
			.y = {.kp_init = 2.2f, .upmax_init = 1500.f, .kp_boost = 1.5f, .upmax_boost = 1.2f, .const_far = 1500.f, .const_near = 300.f, 
				.fpKi = 0.0025f, .fpKd = 0.f, .fpPreE = 0.f, .fpSumE = 0.f, .fpEMin = 50.f, .fpEMax = 2000.f, .fpUMax = 4000.f, .fpUiMax = 60.f, .fpUdMax = 500.f},
			.w = {.kp_init = 6.f, .upmax_init = 150.f, .kp_boost = 1.5f, .upmax_boost = 1.5f, .const_far = 120.f, .const_near = 75.f, 
				.fpKi = 0.002f, .fpKd = 0.f, .fpPreE = 0.f, .fpSumE = 0.f, .fpEMin = 5.f, .fpEMax = 180.f, .fpUMax = 300.f, .fpUiMax = 10.f, .fpUdMax = 10.f},
			.td_x = {.r = 1000.f, .h = 0.001f, .T = 0.001f},
			.td_y = {.r = 1000.f, .h = 0.001f, .T = 0.001f},
			.td_w = {.r = 1000.f, .h = 0.001f, .T = 0.001f}},
	},
};
uint8_t flag_lock = 0;
PATH_POINT Path_Point = {.point_inx = 0, .flag_point_to_point = 1, .flag_point_end = 0, .point_tim = 0};

// 主状态机
ACE_Manipulator ace = {.state = 255, .across = 0, .flag_ = 0, .path_inx = 255, .flag = 1};
