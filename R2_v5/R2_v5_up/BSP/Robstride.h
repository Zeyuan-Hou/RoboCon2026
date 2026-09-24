#ifndef __ROBSTRIDE_H__
#define __ROBSTRIDE_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include "fdcan.h"

#define Set_mode 'j'
#define Set_parameter 'p'

#define move_control_mode 0
#define Pos_control_mode 1
#define Speed_control_mode 2
#define Elect_control_mode 3
#define Set_Zero_mode 4
#define CSP_control_mode 5

#define Communication_Type_Get_ID 0x00
#define Communication_Type_MotionControl 0x01
#define Communication_Type_MotorRequest 0x02
#define Communication_Type_MotorEnable 0x03
#define Communication_Type_MotorStop 0x04
#define Communication_Type_SetPosZero 0x06
#define Communication_Type_Can_ID 0x07
#define Communication_Type_Control_Mode 0x12
#define Communication_Type_GetSingleParameter 0x11
#define Communication_Type_SetSingleParameter 0x12
#define Communication_Type_ErrorFeedback 0x15
#define Communication_Type_MotorDataSave 0x16
#define Communication_Type_BaudRateChange 0x17
#define Communication_Type_ProactiveEscalationSet 0x18
#define Communication_Type_MotorModeSet 0x19

    typedef struct
    {
        uint16_t index;
        float data;
    } data_read_write_one;

    extern const uint16_t Index_List[15];

    typedef struct
    {
        data_read_write_one run_mode;
        data_read_write_one iq_ref;
        data_read_write_one spd_ref;
        data_read_write_one imit_torque;
        data_read_write_one cur_kp;
        data_read_write_one cur_ki;
        data_read_write_one cur_filt_gain;
        data_read_write_one loc_ref;
        data_read_write_one limit_spd;
        data_read_write_one limit_cur;
        data_read_write_one mechPos;
        data_read_write_one iqf;
        data_read_write_one mechVel;
        data_read_write_one VBUS;
        data_read_write_one rotation;
    } data_read_write;

    typedef struct
    {
        float Angle;
        float Speed;
        float Torque;
        float Temp;
        int pattern;
    } Motor_Pos_RobStride_Info;

    typedef struct
    {
        int set_motor_mode;
        float set_current;
        float set_speed;
        float set_acceleration;
        float set_Torque;
        float set_angle;
        float set_limit_cur;
        float set_limit_speed;
        float set_Kp;
        float set_Ki;
        float set_Kd;
    } Motor_Set;

    typedef enum
    {
        operationControl = 0,
        positionControl = 1,
        speedControl = 2
    } MIT_TYPE;

    typedef struct
    {
        uint8_t CAN_ID;
        uint64_t Unique_ID;
        uint16_t Master_CAN_ID;
        float (*Motor_Offset_MotoFunc)(float Motor_Tar);
        Motor_Set Motor_Set_All;
        uint8_t error_code;
        uint8_t MIT_Mode;
        MIT_TYPE MIT_Type;

        float output;
        int Can_Motor;
        Motor_Pos_RobStride_Info Pos_Info;
        data_read_write drw;
    } RobStride_Motor;

    void data_read_write_init(data_read_write *drw, const uint16_t *index_list);

    void RobStride_Motor_Init(RobStride_Motor *motor, uint8_t CAN_Id, uint8_t MIT_mode);
    void RobStride_Motor_InitWithOffset(RobStride_Motor *motor, float (*Offset_MotoFunc)(float Motor_Tar), uint8_t CAN_Id, uint8_t MIT_mode);

    /* 选择RobStride使用哪一路FDCAN。默认使用hfdcan3。 */
    void RobStride_Motor_Set_FDCAN(FDCAN_HandleTypeDef *hfdcan);

    /* 在HAL_FDCAN_RxFifo0Callback中调用，用于把FDCAN接收帧交给RobStride解析。 */
    uint8_t RobStride_Motor_RxFDCAN(RobStride_Motor *motor, const FDCAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);

    void RobStride_Motor_Set_MIT_Mode(RobStride_Motor *motor, uint8_t MIT_mode);
    void RobStride_Motor_Set_MIT_Type(RobStride_Motor *motor, MIT_TYPE MIT_type);
    uint8_t RobStride_Motor_Get_MIT_Mode(const RobStride_Motor *motor);
    MIT_TYPE RobStride_Motor_Get_MIT_Type(const RobStride_Motor *motor);

    void RobStride_Get_CAN_ID(RobStride_Motor *motor);
    void Set_RobStride_Motor_parameter(RobStride_Motor *motor, uint16_t Index, float Value, char Value_mode);
    void Get_RobStride_Motor_parameter(RobStride_Motor *motor, uint16_t Index);
    void RobStride_Motor_Analysis(RobStride_Motor *motor, uint8_t *DataFrame, uint32_t ID_ExtId);

    void RobStride_Motor_move_control(RobStride_Motor *motor, float Torque, float Angle, float Speed, float Kp, float Kd);
    void RobStride_Motor_Pos_control(RobStride_Motor *motor, float Speed, float Angle);
    void RobStride_Motor_CSP_control(RobStride_Motor *motor, float Angle, float limit_spd);
    void RobStride_Motor_Speed_control(RobStride_Motor *motor, float Speed, float limit_cur);
    void RobStride_Motor_current_control(RobStride_Motor *motor, float current);
    void RobStride_Motor_Set_Zero_control(RobStride_Motor *motor);

    void RobStride_Motor_MotorModeSet(RobStride_Motor *motor, uint8_t F_CMD);
    void Enable_Motor(RobStride_Motor *motor);
    void Disenable_Motor(RobStride_Motor *motor, uint8_t clear_error);
    void Set_CAN_ID(RobStride_Motor *motor, uint8_t Set_CAN_ID);
    void Set_ZeroPos(RobStride_Motor *motor);

    void RobStride_Motor_MIT_Control(RobStride_Motor *motor, float Angle, float Speed, float Kp, float Kd, float Torque);
    void RobStride_Motor_MIT_PositionControl(RobStride_Motor *motor, float position_rad, float speed_rad_per_s);
    void RobStride_Motor_MIT_SpeedControl(RobStride_Motor *motor, float speed_rad_per_s, float current_limit);
    void RobStride_Motor_MIT_Enable(RobStride_Motor *motor);
    void RobStride_Motor_MIT_Disable(RobStride_Motor *motor);
    void RobStride_Motor_MIT_SetZeroPos(RobStride_Motor *motor);
    void RobStride_Motor_MIT_ClearOrCheckError(RobStride_Motor *motor, uint8_t F_CMD);
    void RobStride_Motor_MIT_SetMotorType(RobStride_Motor *motor, uint8_t F_CMD);
    void RobStride_Motor_MIT_SetMotorId(RobStride_Motor *motor, uint8_t F_CMD);
    void RobStride_Motor_MotorDataSave(RobStride_Motor *motor);
    void RobStride_Motor_BaudRateChange(RobStride_Motor *motor, uint8_t F_CMD);
    void RobStride_Motor_ProactiveEscalationSet(RobStride_Motor *motor, uint8_t F_CMD);
    void RobStride_Motor_MIT_MotorModeSet(RobStride_Motor *motor, uint8_t F_CMD);

    float uint16_to_float(uint16_t x, float x_min, float x_max, int bits);
    int float_to_uint_RS(float x, float x_min, float x_max, int bits);
    float Byte_to_float(uint8_t *bytedata);
    uint8_t mapFaults(uint16_t fault16);

    extern RobStride_Motor RobStride_01;
    extern float RS_kp, RS_kd, RS_des, RS_tor;
#ifdef __cplusplus
}
#endif

#endif
