#include "Communication.h"

void PackDataToZGT(void)
{
    TxBufToZGT[0] = 0xBB;
    TxBufToZGT[20] = 0xAA;
    
    TxBufToZGT[1] = ace.cplt_state.weapon_cplt_flag;
    TxBufToZGT[2] = ace.cplt_state.collect_kfs_cplt_flag;
    TxBufToZGT[3] = ace.cplt_state.place_kfs_cplt_flag;
    TxBufToZGT[4] = ace.cplt_state.get_kfs_cplt_flag;
    TxBufToZGT[5] = ace.cplt_state.prepare_kfs_cplt_flag;
    TxBufToZGT[6] = ace.cplt_state.transfer_cplt_flag;

    uint16_t fps = FPS_Monitor();
    TxBufToZGT[7] = (uint8_t)(fps >> 8) & 0xFF;
    TxBufToZGT[8] = (uint8_t)fps & 0xFF;

    float yaw;
    if (g_output_info.yaw >= 720 || g_output_info.yaw <= -720)
    {
        yaw = 0;
    }
    else
    {
        yaw = g_output_info.yaw;
    }
    memcpy(&TxBufToZGT[9], &yaw, sizeof(yaw));
    memcpy(&TxBufToZGT[13], &g_output_info.angle_z, sizeof(g_output_info.angle_z));

    if (system_monitor.fpsMonitor.IR <= 0)
    {
        TxBufToZGT[17] = 0;
        TxBufToZGT[18] = 0;
        TxBufToZGT[19] = 0;
    }
    else
    {
        TxBufToZGT[17] = RxBufFromIR[0];
        TxBufToZGT[18] = RxBufFromIR[1];
        TxBufToZGT[19] = RxBufFromIR[2];
    }
}

uint8_t Rx1_last = 255, Rx2_last = 255, Rx3_last = 255, Rx4_last = 255, Rx5_last = 255;
void UnpackDataFromZGT(void)
{
    // 找帧头帧尾，防错位
    uint8_t found = 0;
    for (uint8_t a = 0; a < 8; a++)
    {
        if (uart4_rev[a] == 0xAA && uart4_rev[(a + 8 - 1) % 8] == 0xBB)
        {
            memcpy(&RxBufFromZGT, &uart4_rev[a], 8 - a);
            memcpy(&RxBufFromZGT[8 - a], &uart4_rev, a);
            found = 1;
            break;
        }
    }
    if (found == 0)
    {
        return;
    }

    // 主状态，检测到发生变化触发
    if (RxBufFromZGT[0] == 0xAA && RxBufFromZGT[7] == 0xBB)
    {
        if (RxBufFromZGT[1] != Rx1_last)
        {
            uint8_t cmd_accepted = 0;
            if (RxBufFromZGT[1] != 100)
            {
                if (ace.act_state == SILENT) // 锁定状态收到0才解锁
                {
                    if (RxBufFromZGT[1] == 200)
                    {
                        ace.cplt_state.weapon_cplt_flag = 0;
                        if (ace.cplt_state.collect_kfs_cplt_flag == 2)
                        {
                            ace.cplt_state.collect_kfs_cplt_flag = 0;
                        }
                        if (ace.cplt_state.place_kfs_cplt_flag == 2)
                        {
                            ace.cplt_state.place_kfs_cplt_flag = 0;
                        }
                        if (ace.cplt_state.get_kfs_cplt_flag == 2)
                        {
                            ace.cplt_state.get_kfs_cplt_flag = 0;
                        }
                        if (ace.cplt_state.prepare_kfs_cplt_flag == 2)
                        {
                            ace.cplt_state.prepare_kfs_cplt_flag = 0;
                        }
                        if (ace.cplt_state.transfer_cplt_flag == 2)
                        {
                            ace.cplt_state.transfer_cplt_flag = 0;
                        }
                        ace.act_state = IDLE;
                        cmd_accepted = 1;
                    }
                }
                else if (ace.act_state == IDLE) // 解锁空闲状态收到非0才切换
                {
                    if (RxBufFromZGT[1] != 200 && RxBufFromZGT[1] != 0)
                    {
                        ace.set_flag = 0;
                        ace.act_state = (UP_ACTION_STATE)RxBufFromZGT[1];
                        cmd_accepted = 1;
                    }
                }
                else // 其它状态收到新状态才切换
                {
                    if (RxBufFromZGT[1] != 200 && RxBufFromZGT[1] != 0)
                    {
                        ace.set_flag = 0;
                        ace.last_act_state = 0;
                        ace.act_state = (UP_ACTION_STATE)RxBufFromZGT[1];
                        cmd_accepted = 1;
                    }
                }
            }
            if (cmd_accepted || RxBufFromZGT[1] == 100)
            {
                Rx1_last = RxBufFromZGT[1];
            }
        }

        if (RxBufFromZGT[5] != Rx5_last)
        {
            if (RxBufFromZGT[5] == 0)
            {
                ace.cplt_state.prepare_kfs_cplt_flag = 0;
            }
            else if (RxBufFromZGT[5] == 1)
            {
                ace.set_flag = 0;
                ace.act_state = COLLECT_PREPARE;
            }
            else if (RxBufFromZGT[5] == 2)
            {
                ace.set_flag = 0;
                ace.act_state = COLLECT_KFS;
            }
            else if (RxBufFromZGT[5] == 3)
            {
                ace.set_flag = 0;
                flag_tic_tac_toe = 1;
                ace.act_state = COLLECT_PREPARE;
            }
            Rx5_last = RxBufFromZGT[5];
        }

        if (kfs_param_flag == 1)
        {
            if (RxBufFromZGT[4] != Rx4_last)
            {
                communicate_tim = 0;
                communicate_flag = 0;
                Rx4_last = RxBufFromZGT[4];
            }
            if (communicate_tim < 5)
            {
                communicate_tim++;
            }
            else
            {
                if (communicate_flag == 0)
                {
                    ace.kfs.kfs_height = RxBufFromZGT[2];
                    ace.kfs.kfs_orientation = RxBufFromZGT[3];
                    ace.kfs.next_action = RxBufFromZGT[4];
                    communicate_flag = 1;
                }
            }
        } 

        if (r1_orient_flag == 1)
        {
            if (RxBufFromZGT[6] == 0 || RxBufFromZGT[6] == 1)
            {
                ace.r1_orient = 0;
            }
            else if (RxBufFromZGT[6] == 2)
            {
                ace.r1_orient = 1;
            }
        }
    }
}

void receive_IR_feedback()
{
    // 0xAA 0x55 0x51 1 2 3 crc 0x55 0xAA
    if (uart2_rev[0] == 0xAA && uart2_rev[1] == 0x55 && uart2_rev[2] == 0x51 && uart2_rev[7] == 0x55 && uart2_rev[8] == 0xAA)
    {
        uint8_t crc[5];
        memcpy(crc, &uart2_rev[2], 5);
        if (crc8_calc(crc, 4) == uart2_rev[6])
        {
            RxBufFromIR[0] = crc[1];
            RxBufFromIR[1] = crc[2];
            RxBufFromIR[2] = crc[3];
            system_monitor.cntMonitor.IR++;
        }      
    }
    else
    {
        RxBufFromIR[0] = 255;
        RxBufFromIR[1] = 254;
        RxBufFromIR[2] = 253;
    }
}
