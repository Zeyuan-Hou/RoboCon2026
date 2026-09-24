#include "Global_Variables.h"

// system monitor
SYSTEM_MONITOR system_monitor;
OBSERVATION obs;

// CAN Communicate
uint8_t rx_flag = 0;
uint8_t CAN1_RxBuf[8], CAN1_TxBuf[8], CAN2_RxBuf[8], CAN2_TxBuf[8];

// Within Board Communicate
uint8_t RxBufFromZGT[8], TxBufToZGT[21], uart4_rev[8], uart2_rev[9], RxBufFromIR[3];
uint16_t IR_tim = 0;
uint8_t Tx_IR[9] = {0x55, 0xAA, 0x51, 0x00, 0x00, 0x00, 0x00, 0x02, 0x55};
uint8_t collect_kfs_queue[32] = {0}, point1 = 0, point2 = 0;
int32_t communicate_tim = 0;
uint8_t communicate_flag = 0;
uint8_t r1_orient_flag = 1;
uint8_t kfs_param_flag = 1;

// AirOperator
AIR_OPERATOR airOperator;

// DJI variables
ST_DJI_MOTOR stretch_2006 = {.motor_encoder = {.siGearRatio = M2006_uiGearRatio, .siNumber = M2006_siNumber, .state = 0}, .id = 0x202, .getStartPos = 0},
             DJI_3508 = {.motor_encoder = {.siGearRatio = M3508_uiGearRatio, .siNumber = M3508_siNumber, .state = 0}, .id = 0x201, .getStartPos = 0};
uint8_t m3508_ctrl_flag = 0;

// A1 variables
A1_STRUCTRUE Motor_A1 ={
        .Ctrl_Data ={.ID = 1, .mode = ENABLE_MODE, .K_P = 0.f, .K_W = 0.f}};
uint8_t usartBufForA1[78];
uint8_t a1_init_flag = 0;
float a1_init_pos = 0.f;
uint8_t a1_err = 0;

// DM variables
Motor_DM stretch_DM = {.id = 0x01, .mst_id = 0x01, .ctrl.mode = 1, .ctrl.vel_set = 0.0f, .ctrl.pos_set = 0.0f, .ctrl.kd_set = 0.0f, .ctrl.kp_set = 0.0f};
uint8_t dm_init_flag = 0;
float dm_init_pos = 0.f;

// J60 variables
DEEP_MOTOR Gimbal_J60 = {.motor_id_ = 0x01, .MaxPos = MY_J60_MAX_POS, .MinPos = MY_J60_MIN_POS};
MotorCMD Gimbal_J60_CMD = {.motor_id_ = 0x01};
MotorDATA Gimbal_J60_Receive;

MANIPULATION ace = {.act_state = 255, .kfs = {0, 0, 0}, .cplt_state = {0, 0, 0, 0, 0, 0}, .tim = 0, .set_flag = 0, .r1_orient = 0};
uint8_t flag_tic_tac_toe = 0;
uint8_t flag_stand = 0;
uint8_t index_stand = 0;
uint8_t flag_trial_orient = 0;

G_FEEDFORWARD G_ff = {.flag = 0, .DM = 0.f, .A1 = 0.f, .M2006 = 0.f, .M3508 = 0.f};

uint8_t g_uart_rx_buf[512]; /*rx DMA buffer of uart2*/
uint16_t g_uart_rx_cnt = 0; /*reciede data length of uart2*/

uint8_t g_decode_data[512] ={0};     /*buffer for decoding*/
uint16_t g_decode_data_pos = 0; /*bytes left in decode buffer*/

// uint8_t gyro_rx_ready=0;
// uint8_t gyro_rx_shadow_buf[512]={0};
// int16_t gyro_rx_len=0;