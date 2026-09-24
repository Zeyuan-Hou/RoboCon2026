#include "system_monitor.h"
#include "robot.h"
#include "udp_comm.h"

system_monitor_t system_monitor = {{0}, {0}, SYS_IDLE};

// 100ms
void error_detection(system_monitor_t *system_monitor)
{
//    if (system_monitor->real_rate[UDP_REAL] < 300)
//    {
//        system_monitor->system_state = SYS_ERROR;
//        udp_send_data.state = SYS_ERROR;
//    }
//    for (int i = 0; i < 6; ++i)
//    {
//        if (leg[i].heel_motor.real_rate < 300 || leg[i].hip_motor.real_rate < 300 || leg[i].knee_motor.real_rate < 300)
//        {
//            system_monitor->system_state = SYS_ERROR;
//            udp_send_data.state = SYS_ERROR;
//        }
//    }
}

