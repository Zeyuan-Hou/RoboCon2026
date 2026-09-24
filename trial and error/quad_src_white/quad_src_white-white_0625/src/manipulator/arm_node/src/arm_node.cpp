/**
 * @file arm_node.cpp
 * @brief 二自由度机械臂控制节点
 *
 * 功能说明：
 * - 控制2个电机（arm[0:1]）
 * - 支持3状态切换（GRAB/LIFT/STACK）
 * - 状态切换时自动进行余弦插值平滑过渡
 * - 电机0带有重力补偿
 * - 发布状态反馈，包含当前状态和到位标志
 *
 * 订阅话题：
 * - /quad/manipulator/motor_state : 电机状态反馈 [arm_pos0, arm_pos1]
 * - /quad/manipulator/state_cmd   : 机械臂状态切换命令 (0=GRAB, 1=LIFT, 2=STACK)
 *
 * 发布话题：
 * - /quad/manipulator/motor_cmd   : 电机电流指令 [arm_cur0, arm_cur1]
 * - /quad/arm/status              : 机械臂状态反馈 (std_msgs/UInt8: 状态值 + arrived标志)
 */

#include "controllers/common/InterPolate.hpp"
#include "PIDController.hpp"
#include "quad/msg/motor_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include <fstream>
#include <sstream>
#include <vector>

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
    TRACKING    ///< 轨迹跟随阶段
};

/**
 * @brief 二自由度机械臂控制节点类
 */
class ArmNode : public rclcpp::Node {
   public:
    /** 
     * @brief 构造函数 - 初始化参数、订阅器和定时器
     */
    ArmNode()
        : Node("arm_node"),
          current_state_(NONE) {
        // ========== 声明机械臂参数 ==========
        // 控制参数
        this->declare_parameter("control_freq_hz", 400.0);  ///< 控制频率(Hz)
        this->declare_parameter("output_max", 5000.0);      ///< PID输出上限
        this->declare_parameter("integral_max", 1000.0);    ///< 积分限幅
        this->declare_parameter("deadband", 0.00);          ///< 死区
        this->declare_parameter("arm_interpolate_time", 1.2);   ///< 机械臂插值过渡时间(s)

        // 电机0 PID参数 (控制大臂)
        this->declare_parameter("motor0_Kp", 2000.0);
        this->declare_parameter("motor0_Ki", 0.2);
        this->declare_parameter("motor0_Kd", 0.2);

        // 电机1 PID参数 (控制小臂)
        this->declare_parameter("motor1_Kp", 2000.0);
        this->declare_parameter("motor1_Ki", 0.1);
        this->declare_parameter("motor1_Kd", 0.2);

        // 重力补偿系数
        this->declare_parameter("grav_comp_cur", 800.0);

        // 各状态目标位置 (motor0:大臂, motor1:小臂)
        this->declare_parameter("grab_motor0_pos", 0.5);      ///< GRAB状态大臂目标位置
        this->declare_parameter("grab_motor1_pos", 2.0);      ///< GRAB状态小臂目标位置
        this->declare_parameter("lift_motor0_pos", 2.5);      ///< LIFT状态大臂目标位置
        this->declare_parameter("lift_motor1_pos", -2.0);     ///< LIFT状态小臂目标位置
        this->declare_parameter("place_low_motor0_pos", 3.5); ///< PLACE_LOW状态大臂目标位置
        this->declare_parameter("place_low_motor1_pos", -2.0);///< PLACE_LOW状态小臂目标位置
        this->declare_parameter("place_high_motor0_pos", 5.0);///< PLACE_HIGH状态大臂目标位置
        this->declare_parameter("place_high_motor1_pos", -2.0);///< PLACE_HIGH状态小臂目标位置
        this->declare_parameter("lay_motor0_pos", 0.0);///< LAY状态大臂目标位置
        this->declare_parameter("lay_motor1_pos", 0.0);///< LAY状态小臂目标位置

        // 偏置
        this->declare_parameter("motor0_zero_offset", -4.8);   ///< 电机0零位偏置
        this->declare_parameter("motor1_zero_offset", 0.0);   ///< 电机1零位偏置

        // 软限位参数
        this->declare_parameter("soft_pos_limit_enabled", true);  ///< 是否启用软限位
        this->declare_parameter("soft_pos_limit_threshold", 6.3); ///< 软限位触发阈值(rad)

        // 轨迹跟随参数
        this->declare_parameter("use_trajectory_for_grab", true); ///< GRAB状态是否使用轨迹跟随
        this->declare_parameter("trajectory_file", "/home/cat/hitcrt_quad2026_ws/src/manipulator/scripts/grab_trajectory.csv"); ///< 轨迹文件路径
        // ========== 读取参数 ==========
        double control_freq = this->get_parameter("control_freq_hz").as_double();
        double output_max = this->get_parameter("output_max").as_double();
        double integral_max = this->get_parameter("integral_max").as_double();
        double deadband = this->get_parameter("deadband").as_double();
        arm_interpolate_time_ = this->get_parameter("arm_interpolate_time").as_double();
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

        // ========== 初始化PID控制器 ==========
        // 机械臂电机 (motor0:大臂带重力补偿, motor1:小臂)
        arm_pid_controllers_[0] = std::make_unique<manipulator::PIDController>(
            motor0_Kp, motor0_Ki, motor0_Kd, output_max, integral_max, deadband);
        arm_pid_controllers_[1] = std::make_unique<manipulator::PIDController>(
            motor1_Kp, motor1_Ki, motor1_Kd, output_max, integral_max, deadband);

        // ========== 加载状态目标位置 ==========
        load_state_targets();

        // 加载轨迹文件
        use_trajectory_for_grab_ = this->get_parameter("use_trajectory_for_grab").as_bool();
        if (use_trajectory_for_grab_) {
            load_trajectory(this->get_parameter("trajectory_file").as_string());
        }

        // 初始目标位置为GRAB状态
        arm_target_positions_[0] = arm_state_targets_[GRAB][0];
        arm_target_positions_[1] = arm_state_targets_[GRAB][1];

        // ========== 创建订阅器 ==========
        // 电机状态订阅（机械臂2个电机）
        motor_state_sub_ = this->create_subscription<quad::msg::MotorState>(
            "/quad/manipulator/motor_state", 10,
            std::bind(&ArmNode::motor_state_callback, this, std::placeholders::_1));

        // 机械臂状态命令订阅
        arm_state_cmd_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/quad/arm/state_cmd", 10,
            std::bind(&ArmNode::arm_state_cmd_callback, this, std::placeholders::_1));

        // ========== 创建发布器 ==========
        // 电机指令（机械臂2个电机）
        motor_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/quad/manipulator/motor_cmd", 10);

        // 状态反馈发布器
        arm_status_pub_ = this->create_publisher<std_msgs::msg::UInt8>(
            "/quad/arm/status", 10);

        // 到位标志发布器
        arm_arrived_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/quad/arm/arrived", 10);

        // ========== 创建定时器 ==========
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_),
            std::bind(&ArmNode::control_loop, this));

        // ========== 初始化插值状态 ==========
        arm_start_time_ = this->now();
        arm_interpolate_active_[0] = true;
        arm_interpolate_active_[1] = true;

        RCLCPP_INFO(this->get_logger(),
                    "Arm node started | State: NONE (waiting for command)");

        soft_pos_limited_ = false;            
    }

   private:
    // ========== 机械臂控制函数 ==========

    /**
     * @brief 机械臂状态切换函数
     * @param new_state 目标状态
     *
     * 切换状态时记录当前位置作为插值起点，启动插值过渡
     */
    void switch_arm_state(ArmState new_state) {
        if (new_state == current_state_) return;
        // NONE 是安全状态，不能作为目标状态切换
        if (new_state <= NONE || new_state > LAY) {
            RCLCPP_WARN(this->get_logger(), "Invalid arm state: %d", static_cast<int>(new_state));
            return;
        }

        update_state_targets();

        current_state_ = new_state;
        arm_start_pos_[0] = arm_motor_pos_[0];
        arm_start_pos_[1] = arm_motor_pos_[1];
        arm_start_time_ = this->now();
        arm_interpolate_active_[0] = true;
        arm_interpolate_active_[1] = true;

        // 如果进入GRAB状态且启用轨迹跟随，启动预插值阶段
        if (new_state == GRAB && use_trajectory_for_grab_) {
            trajectory_phase_ = TrajectoryPhase::APPROACH;  // 先进入接近阶段
            trajectory_finished_ = false;
            RCLCPP_INFO(this->get_logger(), "GRAB state: approaching trajectory start point");
        }

        const char* state_names[] = {"NONE", "GRAB", "LIFT", "PLACE_LOW", "PLACE_HIGH", "LAY"};
        RCLCPP_INFO(this->get_logger(), "Arm State -> %s(%d)", state_names[new_state], new_state);
    }

    /**
     * @brief 从参数服务器加载各状态的目标位置
     * motor0: 大臂 (带重力补偿)
     * motor1: 小臂
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
     * @param filename 轨迹文件路径
     *
     * CSV格式: timestamp,pos0,pos1
     * 时间单位: 秒
     * 位置单位: 弧度
     */
    void load_trajectory(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_WARN(this->get_logger(), "Failed to open trajectory file: %s", filename.c_str());
            return;
        }

        trajectory_.clear();
        std::string line;
        // 跳过标题行
        std::getline(file, line);

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            TrajectoryPoint point;
            char comma;
            ss >> point.timestamp >> comma >> point.pos[0] >> comma >> point.pos[1];
            // 减去零位偏置，将轨迹位置转换为相对零位的坐标系
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
     * @param elapsed 从轨迹开始经过的时间(s)
     * @param out_pos 输出位置 [motor0, motor1]
     * @return 是否成功获取位置
     */
    bool get_trajectory_position(double elapsed, double out_pos[2]) {
        if (trajectory_.empty()) return false;

        // 轨迹结束
        if (elapsed >= trajectory_.back().timestamp) {
            out_pos[0] = trajectory_.back().pos[0];
            out_pos[1] = trajectory_.back().pos[1];
            return true;
        }

        // 查找当前时间点所在的区间
        for (size_t i = 0; i < trajectory_.size() - 1; ++i) {
            if (elapsed >= trajectory_[i].timestamp && elapsed < trajectory_[i+1].timestamp) {
                // 线性插值
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

    /**
     * @brief 机械臂状态命令回调函数
     * @param msg 状态命令消息 (0=GRAB, 1=LIFT, 2=STACK)
     */
    void arm_state_cmd_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        switch_arm_state(static_cast<ArmState>(msg->data));
    }

    /**
     * @brief 电机状态回调函数
     * @param msg 电机状态消息（位置、速度、电流）
     *
     * 注意：假设msg中包含2个电机的数据
     * [0]: 机械臂电机0(大臂), [1]: 机械臂电机1(小臂)
     */
    void motor_state_callback(const quad::msg::MotorState::SharedPtr msg) {
        // 机械臂电机状态（前2个）- 原始位置
        if (msg->pos.size() >= 2) {
            arm_motor_raw_pos_[0] = msg->pos[0];
            arm_motor_raw_pos_[1] = msg->pos[1];
            // 计算相对零位的位置
            arm_motor_pos_[0] = arm_motor_raw_pos_[0] - motor0_zero_offset_;
            arm_motor_pos_[1] = arm_motor_raw_pos_[1] - motor1_zero_offset_;
        }
        if (msg->vel.size() >= 2) {
            arm_motor_vel_[0] = msg->vel[0];
            arm_motor_vel_[1] = msg->vel[1];
        }
        // 软限位检测 - 使用相对零位位置
        bool limit_enabled = this->get_parameter("soft_pos_limit_enabled").as_bool();
        double limit_threshold = this->get_parameter("soft_pos_limit_threshold").as_double();
        if (limit_enabled && arm_motor_pos_[0] > limit_threshold) {
            soft_pos_limited_ = true;
        } else {
            soft_pos_limited_ = false;
        }
    }

    // ========== 控制主循环 ==========

    /**
     * @brief 控制主循环
     *
     * 执行流程：
     * 1. 更新参数和状态目标
     * 2. 计算机械臂插值目标位置并执行PID
     * 3. 发布控制指令
     */
    void control_loop() {
        // ========== 更新参数 ==========
        arm_interpolate_time_ = this->get_parameter("arm_interpolate_time").as_double();
        update_state_targets();

        // ========== 创建电机指令消息（2个电机） ==========
        std_msgs::msg::Float64MultiArray motor_cmd_msg;
        motor_cmd_msg.data.resize(2);

        // ========== 安全状态检查 ==========
        // NONE状态下不输出任何控制，保持电机自由状态
        if (current_state_ == NONE) {
            motor_cmd_msg.data[0] = 0.0;
            motor_cmd_msg.data[1] = 0.0;
            motor_cmd_pub_->publish(motor_cmd_msg);
            publish_status();
            return;
        }

        // ========== 机械臂控制 ==========
        // 获取当前状态的目标位置
        double arm_goal_pos[2] = {arm_state_targets_[current_state_][0],
                                  arm_state_targets_[current_state_][1]};

        double arm_elapsed = (this->now() - arm_start_time_).seconds();

        // GRAB状态且启用轨迹跟随
        if (current_state_ == GRAB && use_trajectory_for_grab_ && !trajectory_finished_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                "Trajectory: phase=%d, empty=%d", 
                static_cast<int>(trajectory_phase_), trajectory_.empty());
            
            if (trajectory_phase_ == TrajectoryPhase::APPROACH) {
                // 阶段1: 预插值到轨迹起始点
                if (!trajectory_.empty()) {
                    arm_goal_pos[0] = trajectory_[0].pos[0];
                    arm_goal_pos[1] = trajectory_[0].pos[1];

                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                        "APPROACH: target=[%.3f, %.3f], active=[%d, %d]",
                        arm_goal_pos[0], arm_goal_pos[1],
                        arm_interpolate_active_[0], arm_interpolate_active_[1]);

                    // 检查是否到达起始点（两个电机都完成插值）
                    if (!arm_interpolate_active_[0] && !arm_interpolate_active_[1]) {
                        // 切换到轨迹跟随阶段
                        trajectory_phase_ = TrajectoryPhase::TRACKING;
                        trajectory_start_time_ = this->now();
                        // 重新启动插值，用于轨迹跟随
                        arm_start_pos_[0] = arm_motor_pos_[0];
                        arm_start_pos_[1] = arm_motor_pos_[1];
                        arm_start_time_ = this->now();
                        arm_interpolate_active_[0] = true;
                        arm_interpolate_active_[1] = true;
                        RCLCPP_INFO(this->get_logger(), "Approach completed, starting trajectory tracking");
                    }
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                        "Trajectory is empty! Check trajectory file.");
                }
            } else if (trajectory_phase_ == TrajectoryPhase::TRACKING) {
                // 阶段2: 轨迹跟随
                double traj_elapsed = (this->now() - trajectory_start_time_).seconds();
                double traj_pos[2];

                if (get_trajectory_position(traj_elapsed, traj_pos)) {
                    arm_goal_pos[0] = traj_pos[0];
                    arm_goal_pos[1] = traj_pos[1];

                    // 检查轨迹是否结束
                    if (traj_elapsed >= trajectory_.back().timestamp) {
                        trajectory_finished_ = true;
                        trajectory_phase_ = TrajectoryPhase::NONE;
                        RCLCPP_INFO(this->get_logger(), "Trajectory tracking completed");
                    }
                }
            }
        }

        // 电机0位置插值
        if (arm_interpolate_active_[0]) {
            arm_target_positions_[0] = cosineInterpolate(arm_start_pos_[0], arm_goal_pos[0],
                                                         arm_interpolate_time_, arm_elapsed);
            if (arm_elapsed >= arm_interpolate_time_) {
                arm_target_positions_[0] = arm_goal_pos[0];
                arm_interpolate_active_[0] = false;
            }
        } else {
            arm_target_positions_[0] = arm_goal_pos[0];
        }

        // 电机1位置插值
        if (arm_interpolate_active_[1]) {
            arm_target_positions_[1] = cosineInterpolate(arm_start_pos_[1], arm_goal_pos[1],
                                                         arm_interpolate_time_, arm_elapsed);
            if (arm_elapsed >= arm_interpolate_time_) {
                arm_target_positions_[1] = arm_goal_pos[1];
                arm_interpolate_active_[1] = false;
            }
        } else {
            arm_target_positions_[1] = arm_goal_pos[1];
        }

        // 更新机械臂PID参数（支持动态调参）
        double motor0_Kp = this->get_parameter("motor0_Kp").as_double();
        double motor0_Ki = this->get_parameter("motor0_Ki").as_double();
        double motor0_Kd = this->get_parameter("motor0_Kd").as_double();
        double motor1_Kp = this->get_parameter("motor1_Kp").as_double();
        double motor1_Ki = this->get_parameter("motor1_Ki").as_double();
        double motor1_Kd = this->get_parameter("motor1_Kd").as_double();

        arm_pid_controllers_[0]->setParameters(motor0_Kp, motor0_Ki, motor0_Kd);
        arm_pid_controllers_[1]->setParameters(motor1_Kp, motor1_Ki, motor1_Kd);

        // 计算机械臂控制输出
        double grav_comp = this->get_parameter("grav_comp_cur").as_double();

        // 电机0: 大臂 - PID + 重力补偿 (使用相对零位位置)
        motor_cmd_msg.data[0] = arm_pid_controllers_[0]->computeNoDerivativeKick(
                                    arm_target_positions_[0], arm_motor_pos_[0], dt_) +
                                grav_comp * cos(arm_motor_pos_[0] / 2.3);

        // 电机1: 小臂 - 纯PID控制 (使用相对零位位置)
        motor_cmd_msg.data[1] = arm_pid_controllers_[1]->computeNoDerivativeKick(
            arm_target_positions_[1], arm_motor_pos_[1], dt_);

        // 软限位保护：触发时切断电机输出
        if (soft_pos_limited_) {
            motor_cmd_msg.data[0] = 0.0;
            motor_cmd_msg.data[1] = 0.0;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Soft position limit triggered! Motor output disabled.");
        }
            

        // ========== 发布控制指令 ==========
        motor_cmd_pub_->publish(motor_cmd_msg);

        // ========== 发布状态反馈 ==========
        publish_status();
    }

    // ========== 状态反馈函数 ==========

    /**
     * @brief 发布机械臂状态反馈
     * 发布当前状态和到位标志
     */
    void publish_status() {
        // 发布当前状态
        std_msgs::msg::UInt8 status_msg;
        status_msg.data = static_cast<uint8_t>(current_state_);
        arm_status_pub_->publish(status_msg);

        // 判断是否到位（两个电机都完成插值）
        bool arrived = !arm_interpolate_active_[0] && !arm_interpolate_active_[1];
        std_msgs::msg::Bool arrived_msg;
        arrived_msg.data = arrived;
        arm_arrived_pub_->publish(arrived_msg);
    }

    // ========== 成员变量 ==========

    // 机械臂状态
    ArmState current_state_;

    // 控制周期
    double dt_;

    // 机械臂插值时间
    double arm_interpolate_time_;

    // 轨迹跟随参数
    bool use_trajectory_for_grab_ = false;      ///< 是否启用轨迹跟随
    std::vector<TrajectoryPoint> trajectory_;   ///< 轨迹数据
    rclcpp::Time trajectory_start_time_;        ///< 轨迹开始时间
    bool trajectory_finished_ = false;          ///< 轨迹是否完成
    TrajectoryPhase trajectory_phase_ = TrajectoryPhase::NONE;  ///< 轨迹跟随阶段

    // 机械臂PID控制器 [0:大臂, 1:小臂]
    std::unique_ptr<manipulator::PIDController> arm_pid_controllers_[2];

    // 机械臂电机当前状态 [0:大臂, 1:小臂]
    double arm_motor_raw_pos_[2] = {0.0, 0.0};  ///< 电机原始位置
    double arm_motor_pos_[2] = {0.0, 0.0};      ///< 相对零位的位置
    double arm_motor_vel_[2] = {0.0, 0.0};

    // 机械臂目标位置 [0:大臂, 1:小臂]
    double arm_target_positions_[2] = {0.0, 0.0};

    // 机械臂各状态目标位置 [state][motor_idx]
    // 状态: 0=NONE(未使用), 1=GRAB, 2=LIFT, 3=PLACE_LOW, 4=PLACE_HIGH, 5=LAY 
    double arm_state_targets_[6][2];

    // 机械臂插值状态
    double arm_start_pos_[2] = {0.0, 0.0};
    rclcpp::Time arm_start_time_;
    bool arm_interpolate_active_[2] = {false, false};

    double motor0_zero_offset_;
    double motor1_zero_offset_;
    bool soft_pos_limited_;
    // ROS2通信
    rclcpp::Subscription<quad::msg::MotorState>::SharedPtr motor_state_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr arm_state_cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr arm_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_arrived_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
