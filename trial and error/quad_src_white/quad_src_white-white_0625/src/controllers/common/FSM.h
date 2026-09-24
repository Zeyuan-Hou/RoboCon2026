/**
 * @file FSM.h
 * @brief 状态机定义
 * @author GuoZhexi (3256646795@qq.com)
 * @date 2025-12-11 
 * 
 * @copyright Copyright (C) 2025, HITCRT_VISION, all rights reserved.
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>Date     <th>Author  <th>Description
 * <tr><td>2025-12-11 <td>GuoZhexi  <td>
 * </table>
 */
#ifndef FSM_H
#define FSM_H
/**
 * @brief 高层状态机
 */
typedef enum{
    STOP=0, // 失能
    PASSIVE, // 阻尼模式
    FIXED_DOWN, // 固定趴下
    FIXED_STAND, // 固定站立
    FREE_STAND, // 自由站立
    RL_MOVE, // 移动,RL控制
    JUMP, // 高墙
    STRIDE, // 跳沙坑
    SMALL_JUMP, // 均匀跳桥
}State_e;

/**
 * @brief 控制器模式
 */
typedef enum{
    DISABLE_MODE=0, // 失能
    DAMPING_MODE=1, // 阻尼模式
    ENABLE_MODE=2, // 使能
}Control_Mode_e;

/**
 * @brief 策略切换
 */
typedef enum{
    TROT=0, // 正常步态
    CREEP=1, // 匍匐步态
    UPSTAIR=2, // 上楼梯
    KNEEL_CRAWL_POLICY=3, // 跪姿爬行
}Policy_e;

#endif  // FSM_H
