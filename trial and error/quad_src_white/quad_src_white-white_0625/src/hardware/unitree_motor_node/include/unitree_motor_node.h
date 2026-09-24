#ifndef UNITREE_MOTOR_NODE_H
#define UNITREE_MOTOR_NODE_H

#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
// 注意：请确保你的工程中有这两个自定义消息的头文件
#include "quad/msg/encoder.hpp"
#include "quad/msg/joint_cmd.hpp"

// 引入 Unitree 官方的 SDK 头文件
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

using namespace std::chrono_literals;

// 控制器模式
typedef enum {
    DISABLE_MODE = 0,  // 失能
    DAMPING_MODE = 1,  // 阻尼模式
    ENABLE_MODE = 2,   // 使能
} Control_Mode_e;

// #define CALIBRATION 

// 沿用你原有的标定参数
static constexpr std::array<double, 12> MOTOR_REDUCTION_RATIO = {
    6.33, 6.33, 6.33, 6.33, 6.33, 6.33, 6.33, 6.33, 6.33, 6.33, 6.33, 6.33};
static constexpr std::array<double, 12> EXTERNAL_REDUCTION_RATIO = {
    1, 1, 1.166, 1, 1, 1.166, 1, 1, 1.166, 1, 1, 1.166};
static constexpr std::array<double, 12> LEG_MIRROR_COE = {1,  -1, -1, -1, 1, 1,
                                                          -1, -1, -1, 1,  1, 1};
// 标定方法,将zero_offset设为0,将机器人摆到期望的初始关节位置,读取此时计算的关节角度填入
static constexpr std::array<double, 12> REAL_INIT_POS = {
    0.428,   -0.535,  -0.643, -1.189, -0.40, 0.688,
    -0.98, -1.4, -0.518, 0.282,  -0.111,  0.128};
static constexpr std::array<double, 12> JOINT_INIT_POS = {0, 0, 0, 0, 0, 0,
                                                          0, 0, 0, 0, 0, 0};

// 【新增映射结构体】将高层逻辑的 12 个关节映射到 4 个串口和 3 个电机ID上
struct UnitreeMotorMap {
    int serial_idx;  // 对应 /dev/ttyUSB0 ~ ttyUSB3
    int motor_id;    // 对应电机 ID: 0, 1, 2
};

// 映射表：逻辑下标 (0-11) 对应的物理串口和电机ID
// 0-2: FL(左前), 3-5: FR(右前), 6-8: RR(右后), 9-11: RL(左后)
static constexpr std::array<UnitreeMotorMap, 12> LOGICAL_TO_UNITREE_MAP = {{
    {0, 0},
    {0, 1},
    {0, 2},  // FL: USB0
    {1, 0},
    {1, 1},
    {1, 2},  // FR: USB1
    {2, 0},
    {2, 1},
    {2, 2},  // RR: USB2
    {3, 0},
    {3, 1},
    {3, 2}  // RL: USB3
}};

class UnitreeMotorNode : public rclcpp::Node {
   public:
    UnitreeMotorNode();
    ~UnitreeMotorNode();

   private:
    void param_init();
    void jointcmd_callback(const quad::msg::JointCmd::SharedPtr msg);
    void publishState();

    // 底层并行通讯线程
    void legControlLoop(int leg_id);

    // ROS 2 对象
    rclcpp::Publisher<quad::msg::Encoder>::SharedPtr encoder_pub_;
    rclcpp::Subscription<quad::msg::JointCmd>::SharedPtr joint_cmd_sub_;
    // rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
    // thermal_load_pub_;
    // rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
    // safe_pos_pub_;
    rclcpp::TimerBase::SharedPtr update_data_timer_;

    // 消息缓存
    quad::msg::Encoder encoder_msg_;
    // std::array<float, 12> thermal_load_array_;
    // std::array<float, 12> safe_pos_array_;

    // 标定与映射参数
    std::array<double, 12> motor_to_joint_scale_;
    std::array<double, 12> zero_offset_;

    // ================= 多线程与底层硬件相关 =================
    std::atomic<bool> running_;
    std::array<std::shared_ptr<SerialPort>, 4> serial_;
    std::array<std::thread, 4> leg_threads_;

    // 为 4 个独立的串口通讯各分配一把锁，极大降低读写冲突
    std::array<std::mutex, 4> leg_mutex_;

    // 共享内存（用于主线程与底层线程之间的数据交换）
    std::array<MotorCmd, 12> target_cmds_;
    std::array<MotorData, 12> current_states_;
};

#endif  // UNITREE_MOTOR_NODE_H
