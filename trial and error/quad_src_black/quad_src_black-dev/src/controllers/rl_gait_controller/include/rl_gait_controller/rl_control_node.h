#ifndef RL_INTERFACE_NODE_H
#define RL_INTERFACE_NODE_H

#include <deque>
#include <fstream>
#include <iostream>
#include <vector>
#include <array>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "FSM.h"
#include "Mutex.hpp"
#include "quad/msg/high_commands.hpp"
#include "quad/msg/joint_cmd.hpp"
#include "quad/msg/state.hpp"
#include "quad/msg/encoder.hpp"
#include "quad/msg/quadimu.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>


// 基础结构体
struct Vec3 { float x, y, z; };

struct NormalizationConfig {
    struct ObsScales {
        double lin_vel = 2.0;
        double ang_vel = 0.25;
        double dof_pos = 1.0;
        double dof_vel = 0.05;
    } obs_scales;
    double clip_observations = 100.0;
    double clip_actions = 100.0;
};

struct ControlConfig {
    double action_scale = 0.25;
    double hip_reduction = 0.5;
};

class RLControlNode : public rclcpp::Node {
   public:
    RLControlNode();
    virtual ~RLControlNode() = default;

   private:
    void state_callback(const quad::msg::State::SharedPtr msg);
    void encoder_callback(const quad::msg::Encoder::SharedPtr msg);
    void high_command_callback(const quad::msg::HighCommands::SharedPtr msg);
    void rl_action_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void update_rl_obs();

    Vec3 quatRotateInverse(const Vec3& v, float x, float y, float z, float w);
    void save_obs_to_file();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<quad::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<quad::msg::Encoder>::SharedPtr encoder_sub_;
    rclcpp::Subscription<quad::msg::HighCommands>::SharedPtr high_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr action_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr obs_pub_;
    rclcpp::Publisher<quad::msg::JointCmd>::SharedPtr joint_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr policy_select_pub_;

    State_e current_state_;
    Policy_e current_policy_;
    NormalizationConfig normalization_;
    ControlConfig control_;

    struct SensorData_t {
        quad::msg::Quadimu imu;
        quad::msg::Encoder encoder;
    } sensor_;

    struct Joy_t {
        double lin_x = 0.0, lin_y = 0.0, ang_yaw = 0.0;
    } joy_;

    struct Cmd_t {
        double x = 0.0, y = 0.0, dyaw = 0.0;
    } cmd_;

    std::array<double, 12> last_action_;
    std::deque<std::array<double, 45>> obs_queue_;

    // 配置参数
    const int obs_history_length_ = 6;
    const std::string obs_filename_ = "src/inference/rknn_quad_node/logs/obs.txt";
    std::vector<double> default_dof_angles_;
    std::vector<double> default_angles_trot_;
    std::vector<double> default_angles_creep_;
    std::vector<double> default_angles_upstair_;
    std::vector<double> default_angles_kneel_crawl_;
};

#endif
