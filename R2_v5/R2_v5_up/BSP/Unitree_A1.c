#include "Unitree_A1.h"

uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++)
    {
        xbit = (uint32_t)1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;

            xbit >>= 1;
        }
    }
    return CRC32;
}

void modify(A1_CTRL_DATA *motor_data, uint8_t *raw_data) // A1电机发送数据转换
{
    raw_data[0] = 0xFE;
    raw_data[1] = 0xEE;
    raw_data[2] = motor_data->ID;
    raw_data[3] = 0x00;
    raw_data[4] = motor_data->mode;
    raw_data[5] = 0xff;
    raw_data[6] = 0x00;
    raw_data[7] = 0x00;
    raw_data[8] = 0x00;
    raw_data[9] = 0x00;
    raw_data[10] = 0x00;
    raw_data[11] = 0x00;
    raw_data[12] = ((int)(motor_data->T * 256) & 0x00FF);
    raw_data[13] = ((int)(motor_data->T * 256) >> 8);
    raw_data[14] = ((int)(motor_data->W * 128) & 0x00FF);
    raw_data[15] = ((int)(motor_data->W * 128) >> 8);

    raw_data[16] = ((int)(motor_data->Pos * 16384 / 2 / PI) & 0x00FF);
    raw_data[17] = ((int)(motor_data->Pos * 16384 / 2 / PI) >> 8);
    raw_data[18] = ((int)(motor_data->Pos * 16384 / 2 / PI) >> 16);
    raw_data[19] = ((int)(motor_data->Pos * 16384 / 2 / PI) >> 24);

    raw_data[20] = ((int)(motor_data->K_P * 2048) & 0x00FF);
    raw_data[21] = ((int)(motor_data->K_P * 2048) >> 8);

    raw_data[22] = ((int)(motor_data->K_W * 1024) & 0x00FF);
    raw_data[23] = ((int)(motor_data->K_W * 1024) >> 8);

    raw_data[24] = 0x00;
    raw_data[25] = 0x00;
    raw_data[26] = 0x00;
    raw_data[27] = 0x00;
    raw_data[28] = 0x00;
    raw_data[29] = 0x00;
    uint32_t crc = crc32_core((uint32_t *)raw_data, 7);
    raw_data[30] = (uint8_t)(crc) & 0xFF;
    raw_data[31] = (uint8_t)(crc >> 8);
    raw_data[32] = (uint8_t)(crc >> 16);
    raw_data[33] = (uint8_t)(crc >> 24);
}

uint32_t extract(A1_RECEIVE_DATA *motor_data, uint8_t *raw_data) // A1电机接收处理
{
    uint32_t crc = (raw_data[74] << 0) + (raw_data[75] << 8) + (raw_data[76] << 16) + (raw_data[77] << 24);
    if (crc == crc32_core((uint32_t *)raw_data, 18))
    {
        motor_data->ID = raw_data[2];
        motor_data->mode = raw_data[4];
        motor_data->Temp = raw_data[6];
        motor_data->Error = raw_data[7];
        motor_data->T = (*(short *)(&raw_data[12])) / 256.0;
        motor_data->W = (*(short *)(&raw_data[14])) / 128.0;
        motor_data->Acc = (*(short *)(&raw_data[26]));
        motor_data->Pos = (*(int *)(&raw_data[30])) * 2 * PI / 16384.0f;
        motor_data->gyro[0] = (*(short *)(&raw_data[38])) * 2000 * 2 * PI / 32768 / 360;
        motor_data->gyro[1] = (*(short *)(&raw_data[40])) * 2000 * 2 * PI / 32768 / 360;
        motor_data->gyro[2] = (*(short *)(&raw_data[42])) * 2000 * 2 * PI / 32768 / 360;
        motor_data->acc[0] = (*(short *)(&raw_data[44])) * 8 * 9.80665 / 32768;
        motor_data->acc[1] = (*(short *)(&raw_data[46])) * 8 * 9.80665 / 32768;
        motor_data->acc[2] = (*(short *)(&raw_data[48])) * 8.0 * 9.80665 / 32768;
        return 1;
    }
    return 0;
}

void receive_motor_feedback(uint8_t *raw_data)
{
    A1_RECEIVE_DATA a1_receive_data;
    if (extract(&a1_receive_data, raw_data)) // 反馈数据crc校验通过，同时将接收到的u8数据处理到结构体里
    {
        if (a1_receive_data.Error != 0)
        {
            // error manage
            a1_err = a1_receive_data.Error;
        }
        switch (a1_receive_data.ID)
        {
        case 1:
            Motor_A1.Rec_Data = a1_receive_data;
            Motor_A1.RealPos = Motor_A1.Rec_Data.Pos / 9.1f; // 计算电机减速后的转子真实转动角度

            system_monitor.cntMonitor.A1++;
            break;
        default:
            break;
        }
    }
}

void motor_A1_cmd(A1_STRUCTRUE *ctrl_data)
{
    ctrl_data->Ctrl_Data.Pos = ctrl_data->TargetPos * 9.1f;

    uint8_t data[34];
    modify(&ctrl_data->Ctrl_Data, data);
    HAL_UART_Transmit_DMA(&huart1, data, 34);
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET);
}
