#include "Debug.h"

extern uint8_t DM, M2006, A1, J60, M3508;
float j60_kp = 300, j60_kd = 8;
float dm_kp = 35.f, dm_kd = 0.8f;
float rs_kp = 30.f, rs_kd = 0.9f;
float a1_kp = 0.2f, a1_kd = 0.6f;
float m2006_kp_out = 7, m2006_kd_out = 0.4, m2006_kp_in = 180, m2006_kd_in = 50;

void J60_param_set(uint8_t mode)
{
    if (mode == 1 || mode == 10)
    {
        Gimbal_J60_CMD.kp_ = j60_kp;
        Gimbal_J60_CMD.kd_ = j60_kd;
    }
    else if (mode == 0)
    {
        Gimbal_J60_CMD.kp_ = 0;
        Gimbal_J60_CMD.kd_ = 0;
    }
    Gimbal_J60_CMD.torque_ = 0;
    Gimbal_J60_CMD.position_ = ace.real_time_target_pos[J60];
}

void DM_param_set(uint8_t mode)
{
    if (mode == 1)
    {
        stretch_DM.ctrl.kp_set = dm_kp;
        stretch_DM.ctrl.kd_set = dm_kd;
    }
    else if (mode == 0 || mode == 10)
    {
        stretch_DM.ctrl.kp_set = 0.f;
        stretch_DM.ctrl.kd_set = 0.f;
    }
    stretch_DM.ctrl.tor_set = G_ff.DM;
    stretch_DM.ctrl.pos_set = ace.real_time_target_pos[DM] + dm_init_pos;
}

void RS_param_set(uint8_t mode)
{
    if (mode == 1)
    {
        RS_kp = rs_kp;
        RS_kd = rs_kd;
    }
    else if (mode == 0 || mode == 10)
    {
        RS_kp = 0.f;
        RS_kd = 0.f;
    }
    RS_tor = G_ff.DM;
    RS_des = ace.real_time_target_pos[DM] + dm_init_pos;
}

void A1_param_set(uint8_t mode)
{
    if (mode == 1)
    {
        Motor_A1.Ctrl_Data.K_P = a1_kp;
        Motor_A1.Ctrl_Data.K_W = a1_kd;
    }
    else if (mode == 0 || mode == 10)
    {
        Motor_A1.Ctrl_Data.K_P = 0.f;
        Motor_A1.Ctrl_Data.K_W = 0.f;
    }
    Motor_A1.Ctrl_Data.T = G_ff.A1;
    // Motor_A1.Ctrl_Data.T = 0;
    Motor_A1.TargetPos = ace.real_time_target_pos[A1] + a1_init_pos;
}

void M3508_param_set(uint8_t mode)
{
    if (mode == 1)
    {
        PID_Init(&DJI_3508.motor_pid.outer, 5.f, 0.01f, 1.f, 0.f, 100.f, 400.f, 400.f, 400.f, 100.f);
        PID_Init(&DJI_3508.motor_pid.inner, 90, 0.01, 10, 0, 800, 500, 15000, 15000, 3000);
    }
    else if (mode == 2)
    {
        PID_Init(&DJI_3508.motor_pid.outer, 9.f, 0.01f, 2.f, 0.f, 100.f, 400.f, 400.f, 400.f, 100.f);
        PID_Init(&DJI_3508.motor_pid.inner, 270, 0.01, 40, 0, 800, 500, 10000, 10000, 3000);
    }
    else if (mode == 0 || mode == 10)
    {
        PID_Init(&DJI_3508.motor_pid.outer, 0.f, 0.f, 0.f, 0.f, 100.f, 400.f, 400.f, 400.f, 100.f);
        PID_Init(&DJI_3508.motor_pid.inner, 0, 0, 0, 0, 800, 500, 15000, 15000, 3000);
    }
    DJI_3508.outerTarget = ace.real_time_target_pos[M3508];
}

void M2006_param_set(uint8_t mode)
{
    if (mode == 1)
    {
        PID_Init(&stretch_2006.motor_pid.outer, m2006_kp_out, 0.05f, m2006_kd_out, 0.f, 100.f, 350.f, 800.f, 800.f, 200.f);
        PID_Init(&stretch_2006.motor_pid.inner, m2006_kp_in, 0, m2006_kd_in, 0, 800, 1000, 8500, 8500, 4000);
    }
    else if (mode == 0 || mode == 10)
    {
        PID_Init(&stretch_2006.motor_pid.outer, 0.f, 0.f, 0.f, 0.f, 100.f, 350.f, 800.f, 800.f, 200.f);
        PID_Init(&stretch_2006.motor_pid.inner, 0, 0, 0, 0, 800, 1000, 9000, 9000, 3000);
    }
    stretch_2006.outerTarget = ace.real_time_target_pos[M2006];
}

void Observation()
{
    obs.dm_pos = RobStride_01.Pos_Info.Angle - dm_init_pos;
    obs.j60_pos = Gimbal_J60.position_;
    obs.a1_pos = Motor_A1.RealPos - a1_init_pos;
    obs.m2006_pos = stretch_2006.angle;
    obs.m3508_pos = DJI_3508.angle;

    obs.dm_tor = RobStride_01.Pos_Info.Torque;
    obs.j60_tor = Gimbal_J60.torque_;
    obs.a1_tor = Motor_A1.Rec_Data.T;
    obs.m2006_tor = stretch_2006.tor_cur;
    obs.m3508_tor = DJI_3508.tor_cur;
}

uint16_t FPS_Monitor()
{
    if (abs((int)system_monitor.fpsMonitor.DM - 1000) > 20)
    {
        system_monitor.errorMonitor.DM = 0;
    }
    else
    {
        system_monitor.errorMonitor.DM = 1;
    }
    if (abs((int)system_monitor.fpsMonitor.stretch_2006 - 1000) > 20)
    {
        system_monitor.errorMonitor.stretch_2006 = 0;
    }
    else
    {
        system_monitor.errorMonitor.stretch_2006 = 1;
    }
    if (abs((int)system_monitor.fpsMonitor.A1 - 2000) > 40)
    {
        system_monitor.errorMonitor.A1 = 0;
    }
    else
    {
        system_monitor.errorMonitor.A1 = 1;
    }
    if (abs((int)system_monitor.fpsMonitor.J60 - 1000) > 20)
    {
        system_monitor.errorMonitor.J60 = 0;
    }
    else
    {
        system_monitor.errorMonitor.J60 = 1;
    }
    if (abs((int)system_monitor.fpsMonitor.DJI_3508 - 1000) > 20)
    {
        system_monitor.errorMonitor.DJI_3508 = 0;
    }
    else
    {
        system_monitor.errorMonitor.DJI_3508 = 1;
    }
    if (abs((int)system_monitor.fpsMonitor.AirOperater - 330) > 50)
    {
        system_monitor.errorMonitor.AirOperater = 0;
    }
    else
    {
        system_monitor.errorMonitor.AirOperater = 1;
    }
    if (abs((int)system_monitor.fpsMonitor.withinBoardCommunicate - 1000) > 20)
    {
        system_monitor.errorMonitor.withinBoardCommunicate = 0;
    }
    else
    {
        system_monitor.errorMonitor.withinBoardCommunicate = 1;
    }
    if (abs((int)system_monitor.fpsMonitor.Task1 - 1000) > 20)
    {
        system_monitor.errorMonitor.Task1 = 0;
    }
    else
    {
        system_monitor.errorMonitor.Task1 = 1;
    }
    if (abs((int)system_monitor.fpsMonitor.Task2 - 1000) > 20)
    {
        system_monitor.errorMonitor.Task2 = 0;
    }
    else
    {
        system_monitor.errorMonitor.Task2 = 1;
    }
    if (abs((int)system_monitor.fpsMonitor.Task3 - 1000) > 20)
    {
        system_monitor.errorMonitor.Task3 = 0;
    }
    else
    {
        system_monitor.errorMonitor.Task3 = 1;
    }
    if (abs((int)system_monitor.fpsMonitor.Task4 - 1000) > 20)
    {
        system_monitor.errorMonitor.Task4 = 0;
    }
    else
    {
        system_monitor.errorMonitor.Task4 = 1;
    }
    if (abs((int)system_monitor.fpsMonitor.Task5 - 1000) > 20)
    {
        system_monitor.errorMonitor.Task5 = 0;
    }
    else
    {
        system_monitor.errorMonitor.Task5 = 1;
    }

    uint8_t motor_state = 0, task_state = 0;
    motor_state = (system_monitor.errorMonitor.DM << 0) | (system_monitor.errorMonitor.J60 << 1) | (system_monitor.errorMonitor.A1 << 2) | (system_monitor.errorMonitor.stretch_2006 << 3) | (system_monitor.errorMonitor.DJI_3508 << 4) | (system_monitor.errorMonitor.AirOperater << 5);
    task_state = (system_monitor.errorMonitor.withinBoardCommunicate << 0) | (system_monitor.errorMonitor.Task1 << 1) | (system_monitor.errorMonitor.Task2 << 2) | (system_monitor.errorMonitor.Task3 << 3) | (system_monitor.errorMonitor.Task4 << 4) | (system_monitor.errorMonitor.Task5 << 5);
    
    return (((uint16_t)motor_state << 8) | task_state);
}

VOFA_DATA vofa_data;
float vofa[50];
const uint8_t FRAME_TAIL[4] = {0x00, 0x00, 0x80, 0x7F}; // 固定终止符
// 使用justfloat协议将数据发送至vofa+上位机
void VOFA_transmit_data(float *data, uint8_t num)
{
    memcpy(vofa_data.ch_data, data, num * sizeof(float));
    // 明确填充剩余通道为0
    for (int i = num; i < CH_COUNT; i++)
    {
        vofa_data.ch_data[i] = 0.0f;
    }
    memcpy(vofa_data.tail, FRAME_TAIL, sizeof(FRAME_TAIL));

    HAL_UART_Transmit_DMA(&huart7, (uint8_t *)&vofa_data, sizeof(VOFA_DATA)); // 发送
}

void VOFA_load_data()
{
    vofa[0] = ace.act_state;

    vofa[1] = RxBufFromZGT[1];
    vofa[2] = RxBufFromZGT[2];
    vofa[3] = RxBufFromZGT[3];
    vofa[4] = RxBufFromZGT[4];
    vofa[5] = RxBufFromZGT[5];
    vofa[6] = RxBufFromZGT[6];

    vofa[12] = TxBufToZGT[2];
    vofa[13] = TxBufToZGT[3];
    vofa[14] = TxBufToZGT[4];
    vofa[15] = TxBufToZGT[5];
    vofa[16] = TxBufToZGT[6];
    vofa[19] = TxBufToZGT[9];
    vofa[20] = TxBufToZGT[10];
    vofa[21] = TxBufToZGT[11];
    vofa[22] = TxBufToZGT[12];
    vofa[7] = g_output_info.yaw;
    vofa[8] = g_output_info.angle_z;
}

uint8_t crc8_calc(uint8_t *data, uint8_t len) // 豆包写的，自己看
{
    uint8_t crc = 0xFF;
    uint8_t i, j;
    for (i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc <<= 1;
        }
    }
    return crc;
}
