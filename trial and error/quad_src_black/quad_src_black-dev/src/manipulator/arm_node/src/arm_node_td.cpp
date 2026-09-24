/**
 * @file arm_node_td.cpp
 * @brief 二自由度机械臂控制节点 - 使用TD(Tracking Differentiator)替代余弦插值
 *
 * 功能说明：
 * - 控制2个电机（arm[0:1]）
 * - 支持状态切换（GRAB/LIFT/PLACE_LOW/PLACE_HIGH/LAY）
 * - 状态切换时使用TD进行平滑过渡
 * - 电机0带有重力补偿
 * - 发布状态反馈，包含当前状态和到位标志
 *
 * 订阅话题：
 * - /quad/manipulator/motor_state : 电机状态反馈 [arm_pos0, arm_pos1]
 * - /quad/manipulator/state_cmd   : 机械臂状态切换命令 (0=GRAB, 1=LIFT, 2=PLACE_LOW, 3=PLACE_HIGH, 4=LAY)
 *
 * 发布话题：
 * - /quad/manipulator/motor_cmd   : 电机电流指令 [arm_cur0, arm_cur1]
 * - /quad/arm/status              : 机械臂状态反馈 (std_msgs/UInt8: 状态值 + arrived标志)
 */

#include "PIDController.hpp"
#include "TD.hpp"
#include "quad/msg/motor_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief 机械臂状态枚举
 */
enum ArmState : uint8_t {
    NONE = 0,        ///< 未初始化/安全状态 - 不输出控制
    GRAB = 1,        ///< 抓取物块状态
    LIFT = 2,        ///< 抬起物块状态
    PLACE_LOW = 3,   ///< 低层放置状态
    PLACE_HIGH = 4,  ///< 高层放置(叠放)状态
    LAY = 5,         ///< 平放状态
};

/**
 * @brief 轨迹点结构体
 */
struct TrajectoryPoint {
    double timestamp;  ///< 时间戳(s)
    double pos[2];     ///< 电机位置 [motor0, motor1]
};

/**
 * @brief 轨迹跟随阶段
 */
enum class TrajectoryPhase {
    NONE,       ///< 未启动
    APPROACH,   ///< 接近轨迹起始点（预插值阶段）
    TRACKING,   ///< 轨迹跟随阶段
    SHAKE       ///< 腕部正弦往复阶段
};

/**
 * @brief 二自由度机械臂控制节点类 - TD版本
 */
class ArmNodeTD : public rclcpp::Node {
public:
    /**
     * @brief 构造函数 - 初始化参数、订阅器和定时器
     */
    ArmNodeTD()
        : Node("arm_node_td"),
          current_state_(NONE) {
        // ========== 声明机械臂参数 ==========
        // 控制参数
        this->declare_parameter("control_freq_hz", 400.0);  ///< 控制频率(Hz)
        this->declare_parameter("output_max", 10000.0);      ///< PID输出上限
        this->declare_parameter("integral_max", 1000.0);    ///< 积分限幅
        this->declare_parameter("deadband", 0.00);          ///< 死区

        // TD参数（替代原来的arm_interpolate_time）
        this->declare_parameter("td_r", 30.0);              ///< TD速度因子r，越大跟踪越快
        this->declare_parameter("td_h", 0.2);              ///< TD滤波因子h，越大滤波越好
        this->declare_parameter("tracking_td_r", 30.0);    ///< TRACKING阶段TD速度因子
        this->declare_parameter("tracking_td_h", 0.2);     ///< TRACKING阶段TD滤波因子

        // 电机0 PID参数 (控制大臂)
        this->declare_parameter("motor0_Kp", 5000.0);
        this->declare_parameter("motor0_Ki", 0.2);
        this->declare_parameter("motor0_Kd", 0.2);

        // 电机1 PID参数 (控制小臂)
        this->declare_parameter("motor1_Kp", 3000.0);
        this->declare_parameter("motor1_Ki", 0.1);
        this->declare_parameter("motor1_Kd", 0.2);

        // 重力补偿系数
        this->declare_parameter("grav_comp_cur", 800.0);

        // 各状态目标位置 (motor0:大臂, motor1:小臂)
        this->declare_parameter("grab_motor0_pos", 0.5);      ///< GRAB状态大臂目标位置
        this->declare_parameter("grab_motor1_pos", 2.5);      ///< GRAB状态小臂目标位置
        this->declare_parameter("lift_motor0_pos", 4.0);      ///< LIFT状态大臂目标位置
        this->declare_parameter("lift_motor1_pos", 0.0);     ///< LIFT状态小臂目标位置
        this->declare_parameter("place_low_motor0_pos", 1.0); ///< PLACE_LOW状态大臂目标位置
        this->declare_parameter("place_low_motor1_pos", 2.0);///< PLACE_LOW状态小臂目标位置
        this->declare_parameter("place_high_motor0_pos", -0.5);///< PLACE_HIGH状态大臂目标位置
        this->declare_parameter("place_high_motor1_pos", 0.0);///< PLACE_HIGH状态小臂目标位置
        this->declare_parameter("lay_motor0_pos", 3.0);       ///< LAY状态大臂目标位置
        this->declare_parameter("lay_motor1_pos", 0.5);       ///< LAY状态小臂目标位置

        // 偏置
        this->declare_parameter("motor0_zero_offset", -4.7);   ///< 电机0零位偏置
        this->declare_parameter("motor1_zero_offset", 0.0);   ///< 电机1零位偏置

        // 软限位参数
        this->declare_parameter("soft_pos_limit_enabled", true);  ///< 是否启用软限位
        this->declare_parameter("soft_pos_limit_threshold", 6.0); ///< 软限位触发阈值(rad)

        // 轨迹跟随参数
        this->declare_parameter("use_trajectory_for_grab", true); ///< GRAB状态是否使用轨迹跟随
        this->declare_parameter("trajectory_file", "/home/cat/hitcrt_quad_ws/src/manipulator/scripts/grab_trajectory.csv"); ///< 轨迹文件路径
        this->declare_parameter("trajectory_speed_scale", 1.0); ///< 轨迹跟踪速度缩放因子(1.0=原速, 2.0=2倍速)

        // SHAKE阶段参数
        this->declare_parameter("shake_amplitude", 1.0);  ///< 腕部正弦振幅(rad)
        this->declare_parameter("shake_frequency", 0.5);  ///< 腕部正弦频率(Hz)
        this->declare_parameter("shake_duration", 1.0);   ///< 腕部正弦持续时间(s)
        this->declare_parameter("shake_td_r", 50.0);      ///< SHAKE阶段TD速度因子
        this->declare_parameter("shake_td_h", 0.05);       ///< SHAKE阶段TD滤波因子

        // ========== 读取参数 ==========
        double control_freq = this->get_parameter("control_freq_hz").as_double();
        double output_max = this->get_parameter("output_max").as_double();
        double integral_max = this->get_parameter("integral_max").as_double();
        double deadband = this->get_parameter("deadband").as_double();
        double td_r = this->get_parameter("td_r").as_double();
        double td_h = this->get_parameter("td_h").as_double();
        td_r_ = td_r;
        td_h_ = td_h;
        tracking_td_r_ = this->get_parameter("tracking_td_r").as_double();
        tracking_td_h_ = this->get_parameter("tracking_td_h").as_double();
        motor0_zero_offset_ = this->get_parameter("motor0_zero_offset").as_double();
        motor1_zero_offset_ = this->get_parameter("motor1_zero_offset").as_double();

        dt_ = 1.0 / control_freq;

        // 机械臂电机PID参数 (motor0:大臂, motor1:小臂)
        double motor0_Kp = this->get_parameter("motor0_Kp").as_double();
        double motor0_Ki = this->get_parameter("motor0_Ki").as_double();
        double motor0_Kd = this->get_parameter("motor0_Kd").as_double();
        double motor1_Kp = this->get_parameter("motor1_Kp").as_double();
        double motor1_Ki = this->get_parameter("motor1_Ki").as_double();
        double motor1_Kd = this->get_parameter("motor1_Kd").as_double();

        // ========== 初始化TD跟踪器 ==========
        td_tracker_[0].init(td_r, td_h, dt_);
        td_tracker_[1].init(td_r, td_h, dt_);

        // ========== 初始化PID控制器 ==========
        arm_pid_controllers_[0] = std::make_unique<manipulator::PIDController>(
            motor0_Kp, motor0_Ki, motor0_Kd, output_max, integral_max, deadband);
        arm_pid_controllers_[1] = std::make_unique<manipulator::PIDController>(
            motor1_Kp, motor1_Ki, motor1_Kd, output_max, integral_max, deadband);

        // ========== 加载状态目标位置 ==========
        load_state_targets();

        // 加载轨迹文件
        use_trajectory_for_grab_ = this->get_parameter("use_trajectory_for_grab").as_bool();
        trajectory_speed_scale_ = this->get_parameter("trajectory_speed_scale").as_double();
        shake_amplitude_ = this->get_parameter("shake_amplitude").as_double();
        shake_frequency_ = this->get_parameter("shake_frequency").as_double();
        shake_duration_ = this->get_parameter("shake_duration").as_double();
        shake_td_r_ = this->get_parameter("shake_td_r").as_double();
        shake_td_h_ = this->get_parameter("shake_td_h").as_double();
        if (use_trajectory_for_grab_) {
            load_trajectory(this->get_parameter("trajectory_file").as_string());
        }

        // 初始目标位置为GRAB状态
        arm_target_positions_[0] = arm_state_targets_[GRAB][0];
        arm_target_positions_[1] = arm_state_targets_[GRAB][1];

        // 初始化TD跟踪器
        td_tracker_[0].reset(arm_target_positions_[0]);
        td_tracker_[0].setTarget(arm_target_positions_[0]);
        td_tracker_[1].reset(arm_target_positions_[1]);
        td_tracker_[1].setTarget(arm_target_positions_[1]);

        // ========== 创建订阅器 ==========
        motor_state_sub_ = this->create_subscription<quad::msg::MotorState>(
            "/quad/manipulator/motor_state", 10,
            std::bind(&ArmNodeTD::motor_state_callback, this, std::placeholders::_1));

        arm_state_cmd_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/quad/arm/state_cmd", 10,
            std::bind(&ArmNodeTD::arm_state_cmd_callback, this, std::placeholders::_1));

        // ========== 创建发布器 ==========
        motor_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/quad/manipulator/motor_cmd", 10);

        arm_status_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
            "/quad/arm/status", 10);

        arm_arrived_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/quad/arm/arrived", 10);

        // 机械臂电机状态发布（电流+速度，用于吸取检测）
        arm_motor_state_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/quad/arm/motor_state", 10);

        // ========== 创建定时器 ==========
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_),
            std::bind(&ArmNodeTD::control_loop, this));

        RCLCPP_INFO(this->get_logger(),
                    "Arm Node TD started | State: NONE (waiting for command) | TD(r=%.2f, h=%.4f)",
                    td_r, td_h);

        soft_pos_limited_ = false;
    }

private:
    // ========== 机械臂控制函数 ==========

    /**
     * @brief 机械臂状态切换函数
     * @param new_state 目标状态
     *
     * 切换状态时重置TD跟踪器，设置新的目标位置
     */
    void switch_arm_state(ArmState new_state) {
        if (new_state == current_state_) return;
        if (new_state <= NONE || new_state > LAY) {
            RCLCPP_WARN(this->get_logger(), "Invalid arm state: %d", static_cast<int>(new_state));
            return;
        }

        update_state_targets();
        td_tracker_[0].setParameters(td_r_, td_h_);
        td_tracker_[1].setParameters(td_r_, td_h_);
        trajectory_phase_ = TrajectoryPhase::NONE;
        trajectory_finished_ = false;
        current_state_ = new_state;

        // 获取当前电机位置作为TD初始值
        double current_pos[2] = {arm_motor_pos_[0], arm_motor_pos_[1]};

        // 设置TD目标位置
        double target_pos[2] = {arm_state_targets_[current_state_][0],
                                arm_state_targets_[current_state_][1]};

        // 重置并启动TD跟踪器
        td_tracker_[0].reset(current_pos[0]);
        td_tracker_[0].setTarget(target_pos[0]);
        td_tracker_[1].reset(current_pos[1]);
        td_tracker_[1].setTarget(target_pos[1]);

        // 如果进入GRAB状态且启用轨迹跟随，启动预插值阶段
        if (new_state == GRAB && use_trajectory_for_grab_) {
            trajectory_phase_ = TrajectoryPhase::APPROACH;
            RCLCPP_INFO(this->get_logger(), "GRAB state: approaching trajectory start point");
        }

        const char* state_names[] = {"NONE", "GRAB", "LIFT", "PLACE_LOW", "PLACE_HIGH", "LAY"};
        RCLCPP_INFO(this->get_logger(), "Arm State -> %s(%d)", state_names[new_state], new_state);
    }

    /**
     * @brief 从参数服务器加载各状态的目标位置
     */
    void load_state_targets() {
        arm_state_targets_[GRAB][0] = this->get_parameter("grab_motor0_pos").as_double();
        arm_state_targets_[GRAB][1] = this->get_parameter("grab_motor1_pos").as_double();
        arm_state_targets_[LIFT][0] = this->get_parameter("lift_motor0_pos").as_double();
        arm_state_targets_[LIFT][1] = this->get_parameter("lift_motor1_pos").as_double();
        arm_state_targets_[PLACE_LOW][0] = this->get_parameter("place_low_motor0_pos").as_double();
        arm_state_targets_[PLACE_LOW][1] = this->get_parameter("place_low_motor1_pos").as_double();
        arm_state_targets_[PLACE_HIGH][0] = this->get_parameter("place_high_motor0_pos").as_double();
        arm_state_targets_[PLACE_HIGH][1] = this->get_parameter("place_high_motor1_pos").as_double();
        arm_state_targets_[LAY][0] = this->get_parameter("lay_motor0_pos").as_double();
        arm_state_targets_[LAY][1] = this->get_parameter("lay_motor1_pos").as_double();
    }

    /**
     * @brief 更新状态目标位置（支持动态参数调整）
     */
    void update_state_targets() { load_state_targets(); }

    /**
     * @brief 从CSV文件加载轨迹数据
     */
    void load_trajectory(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_WARN(this->get_logger(), "Failed to open trajectory file: %s", filename.c_str());
            return;
        }

        trajectory_.clear();
        std::string line;
        std::getline(file, line);  // 跳过标题行

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            TrajectoryPoint point;
            char comma;
            ss >> point.timestamp >> comma >> point.pos[0] >> comma >> point.pos[1];
            point.pos[0] -= motor0_zero_offset_;
            point.pos[1] -= motor1_zero_offset_;
            trajectory_.push_back(point);
        }

        file.close();
        RCLCPP_INFO(this->get_logger(), "Loaded trajectory with %zu points from %s",
                    trajectory_.size(), filename.c_str());
    }

    /**
     * @brief 获取轨迹上的目标位置
     */
    bool get_trajectory_position(double elapsed, double out_pos[2]) {
        if (trajectory_.empty()) return false;

        if (elapsed >= trajectory_.back().timestamp) {
            out_pos[0] = trajectory_.back().pos[0];
            out_pos[1] = trajectory_.back().pos[1];
            return true;
        }

        for (size_t i = 0; i < trajectory_.size() - 1; ++i) {
            if (elapsed >= trajectory_[i].timestamp && elapsed < trajectory_[i+1].timestamp) {
                double t0 = trajectory_[i].timestamp;
                double t1 = trajectory_[i+1].timestamp;
                double alpha = (elapsed - t0) / (t1 - t0);
                out_pos[0] = trajectory_[i].pos[0] + alpha * (trajectory_[i+1].pos[0] - trajectory_[i].pos[0]);
                out_pos[1] = trajectory_[i].pos[1] + alpha * (trajectory_[i+1].pos[1] - trajectory_[i].pos[1]);
                return true;
            }
        }
        return false;
    }

    // ========== 回调函数 ==========

    void arm_state_cmd_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        switch_arm_state(static_cast<ArmState>(msg->data));
    }

    void motor_state_callback(const quad::msg::MotorState::SharedPtr msg) {
        if (msg->pos.size() >= 2) {
            arm_motor_raw_pos_[0] = msg->pos[0];
            arm_motor_raw_pos_[1] = msg->pos[1];
            arm_motor_pos_[0] = arm_motor_raw_pos_[0] - motor0_zero_offset_;
            arm_motor_pos_[1] = arm_motor_raw_pos_[1] - motor1_zero_offset_;
        }
        if (msg->vel.size() >= 2) {
            arm_motor_vel_[0] = msg->vel[0];
            arm_motor_vel_[1] = msg->vel[1];
        }
        if (msg->cur.size() >= 2) {
            arm_motor_cur_[0] = msg->cur[0];
            arm_motor_cur_[1] = msg->cur[1];
        }

        bool limit_enabled = this->get_parameter("soft_pos_limit_enabled").as_bool();
        double limit_threshold = this->get_parameter("soft_pos_limit_threshold").as_double();
        if (limit_enabled && arm_motor_pos_[0] > limit_threshold) {
            soft_pos_limited_ = true;
        } else {
            soft_pos_limited_ = false;
        }
    }

    // ========== 控制主循环 ==========

    void control_loop() {
        // 更新参数
        trajectory_speed_scale_ = this->get_parameter("trajectory_speed_scale").as_double();
        shake_amplitude_ = this->get_parameter("shake_amplitude").as_double();
        shake_frequency_ = this->get_parameter("shake_frequency").as_double();
        shake_duration_ = this->get_parameter("shake_duration").as_double();
        shake_td_r_ = this->get_parameter("shake_td_r").as_double();
        shake_td_h_ = this->get_parameter("shake_td_h").as_double();
        update_state_targets();

        std_msgs::msg::Float64MultiArray motor_cmd_msg;
        motor_cmd_msg.data.resize(2);

        // NONE状态下不输出控制
        if (current_state_ == NONE) {
            motor_cmd_msg.data[0] = 0.0;
            motor_cmd_msg.data[1] = 0.0;
            motor_cmd_pub_->publish(motor_cmd_msg);
            publish_status();
            return;
        }

        // 获取当前状态的目标位置
        double arm_goal_pos[2] = {arm_state_targets_[current_state_][0],
                                  arm_state_targets_[current_state_][1]};

        // GRAB状态且启用轨迹跟随
        if (current_state_ == GRAB && use_trajectory_for_grab_ && !trajectory_finished_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "Trajectory: phase=%d, empty=%d",
                static_cast<int>(trajectory_phase_), trajectory_.empty());

            if (trajectory_phase_ == TrajectoryPhase::APPROACH) {
                if (!trajectory_.empty()) {
                    arm_goal_pos[0] = trajectory_[0].pos[0];
                    arm_goal_pos[1] = trajectory_[0].pos[1];

                    // 更新TD目标为轨迹起始点
                    td_tracker_[0].setTarget(arm_goal_pos[0]);
                    td_tracker_[1].setTarget(arm_goal_pos[1]);

                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                        "APPROACH: target=[%.3f, %.3f]",
                        arm_goal_pos[0], arm_goal_pos[1]);

                    // 检查是否到达起始点
                    if (td_tracker_[0].hasArrived(0.01, 0.05) && td_tracker_[1].hasArrived(0.01, 0.05)) {
                        trajectory_phase_ = TrajectoryPhase::TRACKING;
                        trajectory_start_time_ = this->now();
                        // 用TRACKING专用参数更新TD（不重置状态）
                        td_tracker_[0].setParameters(tracking_td_r_, tracking_td_h_);
                        td_tracker_[1].setParameters(tracking_td_r_, tracking_td_h_);
                        // 重新启动TD跟踪器用于轨迹跟随
                        double current_pos[2] = {arm_motor_pos_[0], arm_motor_pos_[1]};
                        td_tracker_[0].reset(current_pos[0]);
                        td_tracker_[1].reset(current_pos[1]);
                        RCLCPP_INFO(this->get_logger(), "Approach completed, starting trajectory tracking");
                    }
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Trajectory is empty! Check trajectory file.");
                }
            } else if (trajectory_phase_ == TrajectoryPhase::TRACKING) {
                double traj_elapsed_raw = (this->now() - trajectory_start_time_).seconds();
                double traj_elapsed = traj_elapsed_raw * trajectory_speed_scale_;
                double traj_pos[2];

                if (get_trajectory_position(traj_elapsed, traj_pos)) {
                    // 动态更新TD目标
                    td_tracker_[0].setTarget(traj_pos[0]);
                    td_tracker_[1].setTarget(traj_pos[1]);

                    if (traj_elapsed >= trajectory_.back().timestamp) {
                        trajectory_phase_ = TrajectoryPhase::SHAKE;
                        shake_start_time_ = this->now();
                        shake_base_pos_ = traj_pos[1];
                        // 用SHAKE专用参数更新TD
                        td_tracker_[0].setParameters(shake_td_r_, shake_td_h_);
                        td_tracker_[1].setParameters(shake_td_r_, shake_td_h_);
                        RCLCPP_INFO(this->get_logger(), "Trajectory tracking completed, starting SHAKE");
                    }
                }
            } else if (trajectory_phase_ == TrajectoryPhase::SHAKE) {
                double shake_elapsed = (this->now() - shake_start_time_).seconds();
                if (shake_elapsed < shake_duration_) {
                    // 动态更新SHAKE的TD参数
                    td_tracker_[0].setParameters(shake_td_r_, shake_td_h_);
                    td_tracker_[1].setParameters(shake_td_r_, shake_td_h_);
                    // pos0保持轨迹终点位置
                    arm_goal_pos[0] = trajectory_.back().pos[0];
                    // pos1做正弦往复
                    arm_goal_pos[1] = shake_base_pos_ - shake_amplitude_ * std::sin(2.0 * M_PI * shake_frequency_ * shake_elapsed);
                    td_tracker_[0].setTarget(arm_goal_pos[0]);
                    td_tracker_[1].setTarget(arm_goal_pos[1]);
                } else {
                    trajectory_finished_ = true;
                    trajectory_phase_ = TrajectoryPhase::NONE;
                    td_tracker_[0].setParameters(td_r_, td_h_);
                    td_tracker_[1].setParameters(td_r_, td_h_);
                    RCLCPP_INFO(this->get_logger(), "SHAKE completed");
                }
            }
        } else {
            // 非轨迹跟随模式，更新TD目标为状态目标位置
            td_tracker_[0].setTarget(arm_goal_pos[0]);
            td_tracker_[1].setTarget(arm_goal_pos[1]);
        }

        // TD计算并获取目标位置和速度
        td_tracker_[0].calculate();
        arm_target_positions_[0] = td_tracker_[0].getPosition();
        double arm_target_velocities_[2];
        arm_target_velocities_[0] = td_tracker_[0].getVelocity();

        td_tracker_[1].calculate();
        arm_target_positions_[1] = td_tracker_[1].getPosition();
        arm_target_velocities_[1] = td_tracker_[1].getVelocity();

        // ========== PID控制 ==========
        double grav_comp = this->get_parameter("grav_comp_cur").as_double();

        // 电机0: 大臂 - PID(使用TD期望速度与实际速度误差作为微分项) + 重力补偿
        motor_cmd_msg.data[0] = arm_pid_controllers_[0]->computeWithVelocityError(
                                    arm_target_positions_[0], arm_motor_pos_[0],
                                    arm_target_velocities_[0], arm_motor_vel_[0], dt_) +
                                grav_comp * std::cos(arm_motor_pos_[0] / 2.3);

        // 电机1: 小臂 - PID(使用TD期望速度与实际速度误差作为微分项)
        motor_cmd_msg.data[1] = arm_pid_controllers_[1]->computeWithVelocityError(
                                    arm_target_positions_[1], arm_motor_pos_[1],
                                    arm_target_velocities_[1], arm_motor_vel_[1], dt_);

        // 软限位保护
        if (soft_pos_limited_) {
            motor_cmd_msg.data[0] = 0.0;
            motor_cmd_msg.data[1] = 0.0;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Soft position limit triggered! Motor output disabled.");
        }

        // 发布控制指令
        motor_cmd_pub_->publish(motor_cmd_msg);

        // 发布电机状态数据（电流+速度+位置，用于吸取检测）
        std_msgs::msg::Float64MultiArray motor_state_msg;
        motor_state_msg.data.resize(6);
        motor_state_msg.data[0] = arm_motor_cur_[0];
        motor_state_msg.data[1] = arm_motor_cur_[1];
        motor_state_msg.data[2] = arm_motor_vel_[0];
        motor_state_msg.data[3] = arm_motor_vel_[1];
        motor_state_msg.data[4] = arm_motor_pos_[0];  // 大臂位置（减去offset）
        motor_state_msg.data[5] = arm_motor_pos_[1];  // 小臂位置（减去offset）
        arm_motor_state_pub_->publish(motor_state_msg);

        // 发布状态反馈
        publish_status();
    }

    // ========== 状态反馈函数 ==========

    void publish_status() {
        std_msgs::msg::UInt8 status_msg;
        status_msg.data = static_cast<uint8_t>(current_state_);
        arm_status_pub_->publish(status_msg);

        // 判断是否到位（两个电机都到达目标）
        bool arrived = td_tracker_[0].hasArrived(0.01, 0.05) && td_tracker_[1].hasArrived(0.01, 0.05);
        std_msgs::msg::Bool arrived_msg;
        arrived_msg.data = arrived;
        arm_arrived_pub_->publish(arrived_msg);
    }

    // ========== 成员变量 ==========

    ArmState current_state_;
    double dt_;

    // TD跟踪器 [0:大臂, 1:小臂]
    TrackingDifferentiator td_tracker_[2];

    // 轨迹跟随参数
    bool use_trajectory_for_grab_ = false;
    std::vector<TrajectoryPoint> trajectory_;
    rclcpp::Time trajectory_start_time_;
    bool trajectory_finished_ = false;
    TrajectoryPhase trajectory_phase_ = TrajectoryPhase::NONE;

    // SHAKE阶段参数
    double shake_amplitude_ = 0.4;
    double shake_frequency_ = 1.0;
    double shake_duration_ = 1.0;
    rclcpp::Time shake_start_time_;
    double shake_base_pos_ = 0.0;
    double shake_td_r_;
    double shake_td_h_;

    // PID控制器
    std::unique_ptr<manipulator::PIDController> arm_pid_controllers_[2];

    // 电机状态
    double arm_motor_raw_pos_[2] = {0.0, 0.0};
    double arm_motor_pos_[2] = {0.0, 0.0};
    double arm_motor_vel_[2] = {0.0, 0.0};
    double arm_motor_cur_[2] = {0.0, 0.0};  ///< 电机电流（用于吸取检测）

    // 目标位置
    double arm_target_positions_[2] = {0.0, 0.0};

    // 各状态目标位置
    double arm_state_targets_[6][2];

    // 偏置和限位
    double motor0_zero_offset_;
    double motor1_zero_offset_;
    bool soft_pos_limited_;
    double trajectory_speed_scale_ = 1.0;
    double td_r_;
    double td_h_;
    double tracking_td_r_;
    double tracking_td_h_;

    // ROS2通信
    rclcpp::Subscription<quad::msg::MotorState>::SharedPtr motor_state_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr arm_state_cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr arm_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_arrived_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_motor_state_pub_;  ///< 电机状态发布器（电流+速度）
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmNodeTD>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
