#include "monitor.h"

// 帧率检测

void System_Monitor(ST_SYSTEM_MONITOR *monitor)
{
    Monitor(monitor->rate_fps.can1_travelSwitch, &monitor->system_error.can1_travelSwitch, 800, 40);
    Monitor(monitor->rate_fps.can1_d, &monitor->system_error.can1_d, 500, 25);
    Monitor(monitor->rate_fps.can1_d_turn, &monitor->system_error.can1_d_turn, 500, 25);
    Monitor(monitor->rate_fps.can1_foot, &monitor->system_error.can1_foot, 500, 25);
    Monitor(monitor->rate_fps.can1_j60, &monitor->system_error.can1_j60, 500, 25);
    Monitor(monitor->rate_fps.can2_j60, &monitor->system_error.can2_j60, 500, 25);
    Monitor(monitor->rate_fps.can2_lu, &monitor->system_error.can2_lu, 500, 50);
    Monitor(monitor->rate_fps.can2_lu_turn, &monitor->system_error.can2_lu_turn, 500, 50);
    Monitor(monitor->rate_fps.can2_ru, &monitor->system_error.can2_ru, 500, 50);
    Monitor(monitor->rate_fps.can2_ru_turn, &monitor->system_error.can2_ru_turn, 500, 50);
    Monitor(monitor->rate_fps.go1_left, &monitor->system_error.go1_left, 500, 25);
    Monitor(monitor->rate_fps.go1_right, &monitor->system_error.go1_right, 500, 25);
    Monitor(monitor->rate_fps.rc, &monitor->system_error.rc, 250, 125);
    Monitor(monitor->rate_fps.task1, &monitor->system_error.task1, 500, 25);
    Monitor(monitor->rate_fps.task2, &monitor->system_error.task2, 500, 25);
    Monitor(monitor->rate_fps.task3, &monitor->system_error.task3, 1000, 50);
    Monitor(monitor->rate_fps.task4, &monitor->system_error.task4, 500, 25);
    Monitor(monitor->rate_fps.task5, &monitor->system_error.task5, 333, 16);
    Monitor(monitor->rate_fps.task6, &monitor->system_error.task6, 1000, 50);
    Monitor(monitor->rate_fps.VISION_REC, &monitor->system_error.VISION_REC, 100, 5);
    // Monitor(monitor->rate_fps.VISION_TX, &monitor->system_error.VISION_TX, 1000, 50);
}

void Monitor(uint16_t rps, uint16_t *error, uint16_t decision, uint16_t range)
{
    int16_t diff = (int16_t)rps - (int16_t)decision;
    if (abs(diff) < range) *error = 1;
    else *error = 0;
}
