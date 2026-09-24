#ifndef DEVICE_CONTROL_H
#define DEVICE_CONTROL_H

#include "fdcan.h"  // 包含FDCAN相关定义


/**
 * @brief 处理CAN数据发送及按键扫描、LED控制
 * @note 包含距离数据打包、CAN发送、按键扫描逻辑和LED闪烁控制
 */
void CAN_DATA(void);

/**
 * @brief 获取距离传感器数据并转换
 * @note 读取8路距离传感器数据（当前启用4路），并放大1000倍存储
 */
void GET_DISTANCE(void);

/**
 * @brief 阀门控制函数
 * @note 调用ValveCtrl()实现阀门的具体控制逻辑
 */
void VALVE_CONTROL(void);

#endif  // DEVICE_CONTROL_H
