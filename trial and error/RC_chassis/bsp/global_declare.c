#include "global_declare.h"

ST_MOTOR leftup_motor = {.motor_id = 1, .motor_type = M6C18, .ControlLoop_State = SPEED_LOOP, .Input = 0.0f,
    .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0},
    .pid_inner = {.fpKp = 500.f, .fpKi = 5.f, .fpKd = 5.f, .fpUMax = 6000.f, .fpUpMax = 1000.f, .fpUdMax = 1000.f, .fpSumEMax = 2000.f, .fpEMax = 500.f, .fpEMin = 1.0f}};
ST_MOTOR leftdown_motor = {.motor_id = 2, .motor_type = M6C18, .ControlLoop_State = SPEED_LOOP, .Input = 0.0f,
    .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0},
    .pid_inner = {.fpKp = 500.f, .fpKi = 5.f, .fpKd = 5.f, .fpUMax = 6000.f, .fpUpMax = 1000.f, .fpUdMax = 1000.f, .fpSumEMax = 2000.f, .fpEMax = 500.f, .fpEMin = 1.0f}};
ST_MOTOR rightup_motor = {.motor_id = 3, .motor_type = M6C18, .ControlLoop_State = SPEED_LOOP, .Input = 0.0f,
    .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0},
    .pid_inner = {.fpKp = 500.f, .fpKi = 5.f, .fpKd = 5.f, .fpUMax = 6000.f, .fpUpMax = 1000.f, .fpUdMax = 1000.f, .fpSumEMax = 2000.f, .fpEMax = 500.f, .fpEMin = 1.0f}};
ST_MOTOR rightdown_motor = {.motor_id = 4, .motor_type = M6C18, .ControlLoop_State = SPEED_LOOP, .Input = 0.0f,
    .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0},
    .pid_inner = {.fpKp = 500.f, .fpKi = 5.f, .fpKd = 5.f, .fpUMax = 6000.f, .fpUpMax = 1000.f, .fpUdMax = 1000.f, .fpSumEMax = 2000.f, .fpEMax = 500.f, .fpEMin = 1.0f}};

ST_MOTOR leftup_motor_angle = {.motor_id = 2, .motor_type = M6020, .uiGearRatio = GM6020_uiGearRatio, .ControlLoop_State = MULTIPLE_LOOP, .Input = 0.0f,
    .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0},
    .pid_inner = {.fpKp = 100.f, .fpKi = 1.f, .fpKd = 100.f, .fpUMax = 12000.f, .fpUpMax = 10000.f, .fpUdMax = 5000.f, .fpSumEMax = 10000.f, .fpEMax = 500.f, .fpEMin = 1.0f},
    .pid_outer = {.fpKp = 20.f, .fpKi = 0.1f, .fpKd = 40.f, .fpUMax = 1000.f, .fpUpMax = 1000.f, .fpUdMax = 100.f, .fpSumEMax = 10000.f, .fpEMax = 500.f, .fpEMin = 1.0f}};
ST_MOTOR leftdown_motor_angle = {.motor_id = 6, .motor_type = M6020, .uiGearRatio = GM6020_uiGearRatio, .ControlLoop_State = MULTIPLE_LOOP, .Input = 0.0f,
    .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0},
    .pid_inner = {.fpKp = 300.f, .fpKi = 2.4f, .fpKd =170.f, .fpUMax = 20000.f, .fpUpMax = 20000.f, .fpUdMax = 20000.f, .fpSumEMax = 20000.f, .fpEMax = 500.f, .fpEMin = 1.0f},
    .pid_outer = {.fpKp = 50.f, .fpKi = 0.f, .fpKd = 70.f, .fpUMax = 2000.f, .fpUpMax = 2000.f, .fpUdMax = 2000.f, .fpSumEMax = 10000.f, .fpEMax = 500.f, .fpEMin = 1.0f},
    .td = {.r = 70000, .h = 0.05, .T = 0.001}};
ST_MOTOR rightup_motor_angle = {.motor_id = 3, .motor_type = M6020, .uiGearRatio = GM6020_uiGearRatio, .ControlLoop_State = MULTIPLE_LOOP, .Input = 0.0f,
    .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0},
    .pid_inner = {.fpKp = 100.f, .fpKi = 1.f, .fpKd = 100.f, .fpUMax = 12000.f, .fpUpMax = 10000.f, .fpUdMax = 5000.f, .fpSumEMax = 10000.f, .fpEMax = 500.f, .fpEMin = 1.0f},
    .pid_outer = {.fpKp = 20.f, .fpKi = 0.1f, .fpKd = 40.f, .fpUMax = 1000.f, .fpUpMax = 1000.f, .fpUdMax = 100.f, .fpSumEMax = 10000.f, .fpEMax = 500.f, .fpEMin = 1.0f}};
ST_MOTOR rightdown_motor_angle = {.motor_id = 1, .motor_type = M6020, .uiGearRatio = GM6020_uiGearRatio, .ControlLoop_State = MULTIPLE_LOOP, .Input = 0.0f,
    .motor_encoder = {.siNumber = 8192, .siSumValue = 0, .siRawValue = 0},
    .pid_inner = {.fpKp = 360.f, .fpKi = 2.7f, .fpKd = 240.f, .fpUMax = 20000.f, .fpUpMax = 20000.f, .fpUdMax = 20000.f, .fpSumEMax = 20000.f, .fpEMax = 500.f, .fpEMin = 1.0f},
    .pid_outer = {.fpKp = 70.f, .fpKi = 0.f, .fpKd = 95.f, .fpUMax = 2000.f, .fpUpMax = 2000.f, .fpUdMax = 2000.f, .fpSumEMax = 10000.f, .fpEMax = 500.f, .fpEMin = 1.0f},
    .td = {.r = 70000, .h = 0.05, .T = 0.001}};

ST_WHEEL_ENCODER_VELT_FILTER wheel_encoder_velt_filter ={      
    .leftup_velt = {0, 0, 0, 50, 0.001},
    .rightup_velt = {0, 0, 0, 50, 0.001},
    .rightdown_velt = {0, 0, 0, 50, 0.001},
    .leftdown_velt = {0, 0, 0, 50, 0.001}
};

ST_WHEEL2BODY_VELT Wheelvelt_To_Bodyvelt;

// DT35
uint16_t dt35_distance[5];
uint16_t dt35_x1, dt35_y1, dt35_x2, dt35_y2;
ST_DT35 dt35_save, dt35_now;
ST_DT35_NEW DT35_NEW;

/*随动轮*/
ST_FOLLOWER_WHEEL stFollowerWheel;
float degreeA = 0, degreeB = 0;

// 陀螺仪
ST_GYRO Gyro_Data_Test;
int num_circle; // 记录陀螺仪转的总圈数
float fpSumPosQ; // 记录陀螺仪总角度，单位0.1度

/*定位*/
float fpPosXOffset = 0; // X方向纠偏量
float fpPosYOffset = 0; // Y方向纠偏量
float fpQOffset = 0;    // 角度Q纠偏量，勿动 调整机器人初始姿态或补偿陀螺仪的系统误差。
float fpStartX = 0; // 5557.5f;//319 底盘半宽+导轮
float fpStartY = 0; // 545.0f;//361

ST_TD posX_veltX ={.r = 5000, .h = 0.001, .T = 0.001};
ST_TD posY_veltY ={.r = 5000, .h = 0.001, .T = 0.001};   
ST_TD posW_veltW ={.r = 5000, .h = 0.001, .T = 0.001};

ST_GLOBAL_VELT_FILTER global_velt_filter ={.global_vx = {0, 0, 0, 50, 0.001}, .global_vy = {0, 0, 0, 50, 0.001}, .global_w = {0, 0, 0, 50, 0.001}};

KalmanFilter KF_Vx ={.v = 0.f, .P = 1.0f, .Q = 0.01f, .R_ins = 0.1f, .R_whl_base = 0.05f, .slip_thres = 150.0f, .slip_scale = 10.f};
KalmanFilter KF_Vy ={.v = 0.f, .P = 1.0f, .Q = 0.01f, .R_ins = 0.1f, .R_whl_base = 0.05f, .slip_thres = 150.0f, .slip_scale = 10.f};
KalmanFilter KF_W ={.v = 0.f, .P = 1.0f, .Q = 0.01f, .R_ins = 0.1f, .R_whl_base = 0.05f, .slip_thres = 15.0f, .slip_scale = 10.f};

// ST_Chassis_Run chassis_run = {
//     .leftup = {.fpKp = 350.f, .fpKi = 0.2f, .fpKd = 5.f, .fpUMax = 12300.f, .fpUpMax = 10000.f, .fpUdMax = 2000.f, .fpSumEMax = 8000.f, .fpEMax = 300.f, .fpEMin = 0.1f},
//     .rightup = {.fpKp = 200.f, .fpKi = 0.2f, .fpKd = 50.f, .fpUMax = 12300.f, .fpUpMax = 10000.f, .fpUdMax = 2000.f, .fpSumEMax = 8000.f, .fpEMax = 300.f, .fpEMin = 0.1f},
//     .leftdown = {.fpKp = 390.f, .fpKi = 0.45f, .fpKd = 40.f, .fpUMax = 12300.f, .fpUpMax = 10000.f, .fpUdMax = 2000.f, .fpSumEMax = 8000.f, .fpEMax = 300.f, .fpEMin = 0.1f},
//     .rightdown = {.fpKp = 250.f, .fpKi = 0.35f, .fpKd = 40.f, .fpUMax = 12300.f, .fpUpMax = 10000.f, .fpUdMax = 2000.f, .fpSumEMax = 8000.f, .fpEMax = 300.f, .fpEMin = 0.1f},
//     .pid_state = VELT_LOOP,
//     .feed_forward_state = WITHOUT_FORWARD
// };

ST_Chassis_Run chassis_run = {
    .rightup = {.fpKp = 350.f, .fpKi = 0.2f, .fpKd = 5.f, .fpUMax = 12300.f, .fpUpMax = 10000.f, .fpUdMax = 2000.f, .fpSumEMax = 8000.f, .fpEMax = 300.f, .fpEMin = 0.1f},
    .rightdown = {.fpKp = 200.f, .fpKi = 0.2f, .fpKd = 50.f, .fpUMax = 12300.f, .fpUpMax = 10000.f, .fpUdMax = 2000.f, .fpSumEMax = 8000.f, .fpEMax = 300.f, .fpEMin = 0.1f},
    .leftdown = {.fpKp = 390.f, .fpKi = 0.45f, .fpKd = 40.f, .fpUMax = 12300.f, .fpUpMax = 10000.f, .fpUdMax = 2000.f, .fpSumEMax = 8000.f, .fpEMax = 300.f, .fpEMin = 0.1f},
    .leftup = {.fpKp = 250.f, .fpKi = 0.35f, .fpKd = 40.f, .fpUMax = 12300.f, .fpUpMax = 10000.f, .fpUdMax = 2000.f, .fpSumEMax = 8000.f, .fpEMax = 300.f, .fpEMin = 0.1f},
    .pid_state = VELT_LOOP,
    .feed_forward_state = WITHOUT_FORWARD};

// ST_Chassis_Run chassis_run = {
//     .leftup = {.fpKp = 450.f, .fpKi = 2.f, .fpKd = 10.f, .fpUMax = 20000.f, .fpUpMax = 15000.f, .fpUdMax = 8000.f, .fpSumEMax = 10000.f, .fpEMax = 300.f, .fpEMin = 0.5f},
//     .rightup = {.fpKp = 450.f, .fpKi = 2.f, .fpKd = 10.f, .fpUMax = 20000.f, .fpUpMax = 15000.f, .fpUdMax = 8000.f, .fpSumEMax = 10000.f, .fpEMax = 300.f, .fpEMin = 0.5f},
//     .leftdown = {.fpKp = 500.f, .fpKi = 6.f, .fpKd = 3.f, .fpUMax = 18000.f, .fpUpMax = 15000.f, .fpUdMax = 8000.f, .fpSumEMax = 10000.f, .fpEMax = 300.f, .fpEMin = 0.5f},
//     .rightdown = {.fpKp = 450.f, .fpKi = 2.f, .fpKd = 3.f, .fpUMax = 18000.f, .fpUpMax = 15000.f, .fpUdMax = 8000.f, .fpSumEMax = 10000.f, .fpEMax = 300.f, .fpEMin = 0.5f},
//     .pid_state = VELT_LOOP,
//     .feed_forward_state = WITHOUT_FORWARD};

ST_Chassis_Turn chassis_steer_angle = {
    .leftup_in = {.fpKp = 130.f, .fpKi = 0.1f, .fpKd = 2.f, .fpUMax = 12000.f, .fpUpMax = 10000.f, .fpUdMax = 2000.f, .fpSumEMax = 8000.f, .fpEMax = 300.f, .fpEMin = 0.5f},
    .leftup_out = {.fpKp = 10.f, .fpKi = 0.05f, .fpKd = 1.f, .fpUMax = 300.f, .fpUpMax = 150.f, .fpUdMax = 100.f, .fpSumEMax = 3000.f, .fpEMax = 190.f, .fpEMin = 0.5f},
    .leftup_td = {.r = 10000.f, .h = 0.001f, .T = 0.001f},
    .rightup_in = {.fpKp = 130.f, .fpKi = 0.1f, .fpKd = 2.f, .fpUMax = 12000.f, .fpUpMax = 10000.f, .fpUdMax = 2000.f, .fpSumEMax = 8000.f, .fpEMax = 300.f, .fpEMin = 0.5f},
    .rightup_out = {.fpKp = 10.f, .fpKi = 0.05f, .fpKd = 1.f, .fpUMax = 300.f, .fpUpMax = 150.f, .fpUdMax = 100.f, .fpSumEMax = 3000.f, .fpEMax = 190.f, .fpEMin = 0.5f},
    .rightup_td = {.r = 10000.f, .h = 0.001f, .T = 0.001},
    .leftdown_in = {.fpKp = 130.f, .fpKi = 0.1f, .fpKd = 2.f, .fpUMax = 12000.f, .fpUpMax = 10000.f, .fpUdMax = 2000.f, .fpSumEMax = 8000.f, .fpEMax = 300.f, .fpEMin = 0.5f},
    .leftdown_out = {.fpKp = 10.f, .fpKi = 0.05f, .fpKd = 1.f, .fpUMax = 300.f, .fpUpMax = 150.f, .fpUdMax = 100.f, .fpSumEMax = 3000.f, .fpEMax = 190.f, .fpEMin = 0.5f},
    .leftdown_td = {.r = 10000.f, .h = 0.001f, .T = 0.001},  
    .rightdown_in = {.fpKp = 130.f, .fpKi = 0.1f, .fpKd = 2.f, .fpUMax = 12000.f, .fpUpMax = 10000.f, .fpUdMax = 2000.f, .fpSumEMax = 8000.f, .fpEMax = 300.f, .fpEMin = 0.5f},
    .rightdown_out = {.fpKp = 10.f, .fpKi = 0.05f, .fpKd = 1.f, .fpUMax = 300.f, .fpUpMax = 150.f, .fpUdMax = 100.f, .fpSumEMax = 3000.f, .fpEMax = 190.f, .fpEMin = 0.5f},
    .rightdown_td = {.r = 10000.f, .h = 0.001f, .T = 0.001},
    .feed_forward_state = WITHOUT_FORWARD};

ST_Nav nav = {
    .auto_path.pos_pid.x = {.fpKp = 1.3f, .fpKi = 0.003f, .fpKd = 2.f, .fpUMax = 2000.f, .fpUpMax = 1000.f, .fpUdMax = 500.f, .fpSumEMax = 10000.f, .fpEMax = 500.f, .fpEMin = 10.0f},
    .auto_path.pos_pid.y = {.fpKp = 1.3f, .fpKi = 0.003f, .fpKd = 2.f, .fpUMax = 2000.f, .fpUpMax = 1000.f, .fpUdMax = 500.f, .fpSumEMax = 10000.f, .fpEMax = 500.f, .fpEMin = 10.0f},
    .auto_path.pos_pid.w = {.fpKp = 6.f, .fpKi = 0.001f, .fpKd = 2.f, .fpUMax = 90.f, .fpUpMax = 90.f, .fpUdMax = 10.f, .fpSumEMax = 2000.f, .fpEMax = 90.f, .fpEMin = 0.1f}
};
uint8_t flag_lock = 1, end_flag=0;
uint8_t flag_global_manual = 1;
ST_PID Chassis_Global_Yaw_Pid ={
    .fpKp = 7.f, .fpKi = 0.001f, .fpKd = 2.f, .fpUMax = 90.f, .fpUpMax = 90.f, .fpUdMax = 10.f, .fpSumEMax = 2000.f, .fpEMax = 90.f, .fpEMin = 0.1f};
uint8_t flag_permutation_path = 1;

ST_ROBOT stRobot;
float v=0, v_pre=0, v_filtered=0, dx=0, dy=0, dx_pre=0, dy_pre=0;
float V=0, A;

ST_JS_VALUE Js_Value;
uint8_t FLAG_NRF = 0;

Path_State path_state;
POINT point_end;
PATH_PERMUTATION Path_Permuta;

float delta_x, delta_y, delta_q, v_max, a_max, w, alpha;
// float end_x, end_y, end_q, current_x, current_y, current_q;

