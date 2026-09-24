#ifndef POS_CONTROL_NODE_H_
#define POS_CONTROL_NODE_H_

#include <rclcpp/rclcpp.hpp>

#include "FSM.h"
#include "Mutex.hpp"
#include "InterPolate.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "quad/msg/high_commands.hpp"
#include "quad/msg/encoder.hpp"
#include "quad/msg/joint_cmd.hpp"

#include "per_joint_phase_controller.hpp"

class PosControlNode : public rclcpp::Node {
   public:
    PosControlNode();

   private:
    // 控制周期回调函数
    void update_pos_cmd();
    // 编码器接收中断回调函数
    void encoder_callback(const quad::msg::Encoder::SharedPtr msg);
    // 高层命令接收中断回调函数
    void high_command_callback(const quad::msg::HighCommands::SharedPtr msg);
    // 速度命令缓存回调函数（用于跪姿爬行触发）
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void update_jointcmd_smoothly(const std::vector<double>& target,
                                  double time_from_start, double duration);
    void set_pd_default(const double pd[6]);
    void set_pd_arrays(const std::array<double,12>& kp, const std::array<double,12>& kd);
    bool try_parse_state(int raw_state, State_e& parsed_state) const;
    bool try_parse_policy(int raw_policy, Policy_e& parsed_policy) const;
    void reset_action_transition();
    // 状态
    int current_mode_ = DISABLE_MODE;  // 关节控制模式 0:失能 1:失能 2:阻尼模式
    int current_policy_ = TROT;        // 当前策略,需要根据策略设置站立位置
    double pos_change_duration_ = 1.0;  // 关节位置变化总时间
    double pos_cur_[12];
    double pos_start_[12];
    double pos_des_cur_[12];
    double pd_param_default_[6];
    std::array<double,12> kp_current_{};
    std::array<double,12> kd_current_{};
    State_e current_state_;
    std::vector<double> pos_stand_;
    std::vector<double> pos_down_;
    std::vector<double> pos_creep_;
    std::vector<double> pos_kneel_prepare_;
    std::vector<double> pos_kneel_crawl_;
    geometry_msgs::msg::Twist target_velocity_;
    double kneel_prepare_duration_ = 1.0;
    double kneel_gait_period_ = 2.0;
    double kneel_thigh_amp_ = 0.2;
    double kneel_trigger_vel_x_ = 0.05;
    double kneel_rear_thigh_amp_scale_ = 1.35;
    double kneel_rear_calf_lift_offset_ = -0.12;
    double kneel_stop_blend_duration_ = 1.0;
    bool kneel_crawl_active_ = false;
    bool kneel_stop_blending_ = false;
    rclcpp::Time kneel_gait_start_time_;
    rclcpp::Time kneel_stop_blend_start_time_;
    std::array<double, 12> kneel_stop_blend_start_pos_{};

    PerJointPhaseController action_controller_;  // 每关节独立PD动作控制器
        std::array<double,12> pos_start_arr{};

    rclcpp::Time action_start_time_;  // 开始时间
    rclcpp::Time last_time_;          // 上次时间
    rclcpp::Publisher<quad::msg::JointCmd>::SharedPtr joint_cmd_pub_;
    rclcpp::Subscription<quad::msg::Encoder>::SharedPtr encoder_sub_;
    rclcpp::Subscription<quad::msg::HighCommands>::SharedPtr high_cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif