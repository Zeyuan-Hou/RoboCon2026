#ifndef MOTOR_NODE_H
#define MOTOR_NODE_H

#include <chrono>
#include <fstream>  // 必须添加
#include <functional>
#include <iomanip>  // 用于 setprecision
#include <iostream>
#include <memory>
#include <string>

#include "quad/msg/encoder.hpp"
#include "quad/msg/joint_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "udp_comm.h"

using namespace std::chrono_literals;

// const double CALIBRATION_TIME_SEC = 1.0;

// #define CALIBRATION
// #define POSITION_ERROR_LIMIT

const int kLocalPort = 1234;
const std::string kTargetIp = "192.168.10.3";
const int kTargetPort = 5001;

static constexpr std::array<double, 12> MOTOR_REDUCTION_RATIO = {
    9.1, 9.1, 9.1, 9.1, 9.1, 9.1, 9.1, 9.1, 9.1, 9.1, 9.1, 9.1};
static constexpr std::array<double, 12> EXTERNAL_REDUCTION_RATIO = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
static constexpr std::array<double, 12> REAL_INIT_POS = {
    -0.04, 1.029, 0.456,  -0.328, 0.287, -0.548, -0.475,
    0.72,  0.279, -0.349, 0.121,  -0.308};  // 标定方法,将zero_offset设为0,将机器人摆到期望的初始关节位置,读取此时计算的关节角度填入
static constexpr std::array<double, 12> LEG_MIRROR_COE = {1,  1, 1, -1, -1, -1,
                                                          -1, 1, 1, 1,  -1, -1};
static constexpr std::array<double, 12> JOINT_INIT_POS = {0, 0, 0, 0, 0, 0,
                                                          0, 0, 0, 0, 0, 0};

// ====================== 【新增映射表】 ======================
// 数组下标(Index 0-11): 代表上层控制算法的逻辑关节顺序 (FL, FR, RL, RR)
// 数组值(Value): 代表实际 UDP 板卡上的电机端口号 (硬件ID)
// 请根据实际接线修改大括号内的数字！
static constexpr std::array<int, 12> JOINT_TO_MOTOR_MAP = {
    0, 1,  2,  // FL (Front Left)  逻辑对应的物理电机端口
    3, 4,  5,  // FR (Front Right) 逻辑对应的物理电机端口
    6, 7,  8,  // RR (Rear Right)  逻辑对应的物理电机端口
    9, 10, 11  // RL (Rear Left)   逻辑对应的物理电机端口
};

class MotorNode : public rclcpp::Node {
   public:
    MotorNode(uint16_t local_port, const std::string& target_ip,
              uint16_t target_port)
        : Node("MotorNode"), udp_comm_(local_port, target_ip, target_port) {
        encoder_pub_ =
            this->create_publisher<quad::msg::Encoder>("/quad/encoder", 10);

        // 订阅控制层指令
        joint_cmd_sub_ = this->create_subscription<quad::msg::JointCmd>(
            "/quad/joint_cmd", 10,
            std::bind(&MotorNode::jointcmd_callback, this,
                      std::placeholders::_1));

        // 初始化调试信息发布器
        thermal_load_pub_ =
            this->create_publisher<std_msgs::msg::Float32MultiArray>(
                "motor_debug/thermal_load", 10);
        safe_pos_pub_ =
            this->create_publisher<std_msgs::msg::Float32MultiArray>(
                "motor_debug/safe_pos", 10);
        // 初始化调试数据数组
        thermal_load_array_.fill(0.0f);
        safe_pos_array_.fill(0.0f);

        update_data_timer_ = this->create_wall_timer(
            5ms, std::bind(&MotorNode::update_data, this));

        param_init();

        // is_calibrated_ = false;

        if (udp_comm_.init()) {
            // std::cout << "udp init success!" << std::endl;
            RCLCPP_INFO(this->get_logger(), "udp init success!");
        } else {
            // std::cout << "udp init failed!" << std::endl;
            RCLCPP_INFO(this->get_logger(), "udp init failed!");
        }
        start_time_ = this->now();
    }

    ~MotorNode() {
        udp_send_data_.state = 0;
        write();
        if (log_file_.is_open()) {
            log_file_.flush();
            log_file_.close();
            // std::cout << "Log file closed." << std::endl;
        }
    }

   private:
    void update_data();
    void receiveFeedback(void);
    void sendMotorCommand(void);
    void updateMotorFeedback(void);
    void updateMotorCommand(void);
    void write(void);
    void read(void);

    void param_init(void);
    // void calibrate(void);
    double limitJointPosition(int joint_idx);
    void jointcmd_callback(const quad::msg::JointCmd::SharedPtr msg);

    UdpComm udp_comm_;
    udp::ReceiveData udp_receive_data_{};
    udp::SendData udp_send_data_{};
    rclcpp::Time start_time_;

    // bool is_calibrated_;
    // bool system_ready_;

    std::array<double, 12> motor_to_joint_scale_;
    std::array<double, 12> zero_offset_;

    int state_;

    std::array<double, 12> kp_;
    std::array<double, 12> kd_;
    std::array<double, 12> pos_des_;

    std::array<double, 12> safe_pos_des_;
    std::array<double, 12> last_pos_des_;
    std::array<double, 12> thermal_load_;  // 每个关节的热负载累计值

    // 存储调试数据的数组
    std::array<float, 12> thermal_load_array_;
    std::array<float, 12> safe_pos_array_;

    std::ofstream log_file_;
    bool error_detected = false;

    rclcpp::TimerBase::SharedPtr update_data_timer_;
    rclcpp::Publisher<quad::msg::Encoder>::SharedPtr encoder_pub_;
    rclcpp::Subscription<quad::msg::JointCmd>::SharedPtr joint_cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
        thermal_load_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
        safe_pos_pub_;
    quad::msg::Encoder encoder_msg_;
};

#endif  // MOTOR_NODE_H
