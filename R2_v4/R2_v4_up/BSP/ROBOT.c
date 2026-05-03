#include "ROBOT.h"
Action actionFlag = KEEP_QUIET; // 执行什么动作
State stateFlag = STAND_BY;     // 保持何种状态
uint8_t actionCpltFlag = 1,     // 动作是否完成
    chassisMoveFlag = 0,        // 是否允许地盘移动
    completeActionFlag = 1;     // 是否完成动作最后一步
int32_t sm_cnt = 0;
uint8_t KFS_height;          // 0:lower 1:higher 2:top
uint8_t KFS_orientation;     // 0:straight 1:left 2:right
uint8_t KFS_level;           // 0:middle 1:high
uint8_t use_KFS_orientation; // 0:LEFT 1:RIGHT 2:STRAIGHT
uint8_t noWeapon = 0;
uint8_t init_mode = 0;

// system monitor
SYSTEM_MONITOR system_monitor;
// CAN Communicate
uint8_t CAN1_RxBuf[8], CAN1_TxBuf[8], CAN2_RxBuf[8], CAN2_TxBuf[8];

// Within Board Communicate
uint8_t RxBufFromZGT[6], TxBufToZGT[6];
uint8_t WBC_TxCplt_Flag = 1; // 板间通讯发送完成标志位

// gravity Compensation
uint8_t gravityCompensation_state; // 0:without KFS;1:with KFS

// AirOperator
AIR_OPERATOR airOperator;

// DJI variables
ST_DJI_MOTOR stretch_2006 = {.motor_encoder = {.siGearRatio = M2006_uiGearRatio, .siNumber = M2006_siNumber}, .motor_td = {.r = 8000, .h = 0.1, .T = 0.001}, .id = 0x202, .getStartPos = 0},
             // move_2006 = {.motor_encoder = {.siGearRatio = M2006_uiGearRatio, .siNumber = M2006_siNumber}, .motor_td = {.r = 8000, .h = 0.1, .T = 0.001}, .id = 0x201, .getStartPos = 0},
             DJI_3508 = {.motor_encoder = {.siGearRatio = M3508_uiGearRatio, .siNumber = M3508_siNumber}, .motor_td = {.r = 8000, .h = 0.1, .T = 0.001}, .id = 0x201, .getStartPos = 0};

// A1 variables
A1_STRUCTRUE Motor_A1 =
    {
        .Ctrl_Data =
            {
                .ID = 1,
                .mode = ENABLE_MODE,
                .K_P = 0.f,
                .K_W = 0.f}};
uint8_t usartBufForA1[78];
u8 A1_TxCplt_Flag = 1; // A1发送完成标志位
fp32 gTorqueA1;

// DM variables
Motor_DM stretch_DM = {

    .id = 0x01,
    .mst_id = 0x11,
    .ctrl.mode = 1,
    .ctrl.vel_set = 0.0f,
    .ctrl.pos_set = 0.0f,
    .ctrl.kd_set = 0.0f,
    .ctrl.kp_set = 0.0f};
fp32 gTorqueDM;

// J60 variables
DEEP_MOTOR Gimbal_J60 = {.motor_id_ = 0x01, .MaxPos = MY_J60_MAX_POS, .MinPos = MY_J60_MIN_POS};
MotorCMD Gimbal_J60_CMD;
MotorDATA Gimbal_J60_Receive;
