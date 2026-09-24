#include "monitor.h"

// 帧率检测

void System_Monitor(ST_SYSTEM_MONITOR *monitor)
{
    Monitor(monitor->rate_fps.can1_dt35, &monitor->system_error.can1_dt35, 800, 40);
    Monitor(monitor->rate_fps.can1_travelSwitch, &monitor->system_error.can1_travelSwitch, 800, 40);
    Monitor(monitor->rate_fps.can1_ld, &monitor->system_error.can1_ld, 1000, 50);
    Monitor(monitor->rate_fps.can1_ld_turn, &monitor->system_error.can1_ld_turn, 1000, 50);
    Monitor(monitor->rate_fps.can1_rd, &monitor->system_error.can1_rd, 1000, 50);
    Monitor(monitor->rate_fps.can1_rd_turn, &monitor->system_error.can1_rd_turn, 1000, 50);
    Monitor(monitor->rate_fps.can2_j60, &monitor->system_error.can2_j60, 500, 25);
    Monitor(monitor->rate_fps.can2_lu, &monitor->system_error.can2_lu_turn, 1000, 50);
    Monitor(monitor->rate_fps.can2_ru, &monitor->system_error.can2_ru, 1000, 50);
    Monitor(monitor->rate_fps.can2_ru_turn, &monitor->system_error.can2_ru_turn, 1000, 50);
    Monitor(monitor->rate_fps.go1_left, &monitor->system_error.go1_left, 500, 25);
    Monitor(monitor->rate_fps.go1_right, &monitor->system_error.go1_right, 500, 25);
    Monitor(monitor->rate_fps.board_communication, &monitor->system_error.board_communication, 1000, 50);
    Monitor(monitor->rate_fps.rc, &monitor->system_error.rc, 250, 125);
}

void Monitor(uint16_t rps, uint16_t *error, uint16_t decision, uint16_t range)
{
    int16_t diff = (int16_t)rps - (int16_t)decision;
    if (abs(diff) < range) *error = 1;
    else *error = 0;
}
