#include "task_manager/obstacle_nav_node.hpp"
#include "task_manager/lidar_pose_guard.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <fstream>
#include <sstream>

/**
 * @brief 构造障碍赛自动导航节点
 *
 * 输入：ROS 参数（经 load_parameters / load_point_sequence 加载）
 * 输出：初始化成员与 ROS 订阅/发布/定时器
 * 处理：声明参数、解析目标序列，再 setup_ros_interfaces
 */
ObstacleNavNode::ObstacleNavNode() : Node("obstacle_nav_node") {
    load_parameters();
    load_point_sequence();
    setup_ros_interfaces();

    RCLCPP_INFO(this->get_logger(), "ObstacleNavNode initialized. mode=%s backend=%s",
                navigation_mode_param_.c_str(), navigation_backend_param_.c_str());
}

/**
 * @brief 创建订阅、发布与控制定时器
 *
 * 输入：成员变量 tf_topic_、control_rate_
 * 输出：nav_enable_sub_、tf_sub_、各 publisher、timer_
 * 处理：按原 QoS 与话题名注册接口，定时器周期 1/control_rate_
 */
void ObstacleNavNode::setup_ros_interfaces() {
    nav_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "nav_enable", 10, std::bind(&ObstacleNavNode::nav_enable_callback, this, std::placeholders::_1));
    external_target_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "obstacle_sequence/nav_target", 10,
        std::bind(&ObstacleNavNode::external_target_callback, this, std::placeholders::_1));
    sequence_slot_sub_ = this->create_subscription<std_msgs::msg::String>(
        "obstacle_sequence/current_slot", 10,
        std::bind(&ObstacleNavNode::sequence_slot_callback, this, std::placeholders::_1));
    tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        tf_topic_, 100, std::bind(&ObstacleNavNode::tf_callback, this, std::placeholders::_1));

    if (using_nav2_backend()) {
#ifdef TASK_MANAGER_HAS_NAV2_MSGS
        nav2_action_client_ =
            rclcpp_action::create_client<NavigateToPose>(this, nav2_action_name_);
#else
        RCLCPP_ERROR(this->get_logger(),
                     "navigation_backend=NAV2 but task_manager was built without nav2_msgs.");
#endif
    }

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    nav_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("nav_status", 10);
    nav_finished_pub_ = this->create_publisher<std_msgs::msg::Bool>("nav_finished", 10);
    current_pose_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("obstacle_nav/current_pose", 10);
    current_target_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("obstacle_nav/current_target", 10);
    debug_error_pub_ = this->create_publisher<geometry_msgs::msg::Point>("obstacle_nav/debug_error", 10);

    const double period = 1.0 / std::max(control_rate_, 1.0);
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(period),
        std::bind(&ObstacleNavNode::control_loop, this));
}

/**
 * @brief 从 ROS 参数加载导航与控制配置
 *
 * 输入：节点参数服务器
 * 输出：target_frame_、nav_mode_、PID/容差/限速等成员
 * 处理：declare_parameter 后 get；未知 navigation_mode 回退 POINT_SEQUENCE
 */
void ObstacleNavNode::load_parameters() {
    this->declare_parameter("target_frame", "camera_init");
    this->declare_parameter("child_frame", "aft_mapped");
    this->declare_parameter("tf_topic", "/tf");
    this->declare_parameter("navigation_backend", "LOCAL_PD");
    this->declare_parameter("navigation_mode", "POINT_SEQUENCE");
    this->declare_parameter("nav2_action_name", "/navigate_to_pose");
    this->declare_parameter("goal_frame_id", "map");
    this->declare_parameter("nav2_action_server_wait_s", 0.1);
    this->declare_parameter("nav2_retry_period_s", 0.5);
    this->declare_parameter("target_points", std::vector<double>{});
    this->declare_parameter("trajectory_files", std::vector<std::string>{});

    this->declare_parameter("position_tolerance", 0.10);
    this->declare_parameter("yaw_tolerance", 0.10);
    this->declare_parameter("kp_x", 0.6);
    this->declare_parameter("kp_y", 1.0);
    this->declare_parameter("kp_yaw", 0.8);
    this->declare_parameter("kd_x", 0.0);
    this->declare_parameter("kd_y", 0.0);
    this->declare_parameter("kd_yaw", 0.0);
    this->declare_parameter("max_vx", 0.45);
    this->declare_parameter("max_vy", 0.35);
    this->declare_parameter("max_dyaw", 0.6);
    this->declare_parameter("yaw_first_threshold", 0.25);
    this->declare_parameter("yaw_only_threshold", 1.2);
    this->declare_parameter("yaw_only_max_dyaw", 1.2);
    this->declare_parameter("yaw_only_slowdown_threshold", 0.7);
    this->declare_parameter("yaw_cmd_slew_rate", 1.5);
    this->declare_parameter("long_straight_distance_threshold", 1.0);
    this->declare_parameter("long_straight_yaw_threshold", 0.25);
    this->declare_parameter("long_straight_kp_x", 0.0);
    this->declare_parameter("long_straight_max_vx", 0.9);
    this->declare_parameter("final_yaw_align_distance", 0.25);
    this->declare_parameter("approach_yaw_max_dyaw", 0.35);
    this->declare_parameter("yaw_then_forward.enabled_slots", std::vector<std::string>{});
    this->declare_parameter("yaw_then_forward.align_yaw_tolerance", 0.08);
    this->declare_parameter("yaw_then_forward.finish_yaw_tolerance", 0.10);
    this->declare_parameter("yaw_then_forward.forward_tolerance", 0.10);
    this->declare_parameter("yaw_then_forward.lateral_tolerance", 0.30);
    this->declare_parameter("yaw_then_forward.kp_yaw", 1.20);
    this->declare_parameter("yaw_then_forward.max_wz", 0.75);
    this->declare_parameter("yaw_then_forward.wz_slew_rate", 1.0);
    this->declare_parameter("yaw_then_forward.kp_forward", 0.80);
    this->declare_parameter("yaw_then_forward.max_vx", 0.40);
    this->declare_parameter("yaw_then_forward.kp_lateral", 0.25);
    this->declare_parameter("yaw_then_forward.max_vy", 0.08);
    this->declare_parameter("control_rate", 50.0);

    target_frame_ = this->get_parameter("target_frame").as_string();
    child_frame_ = this->get_parameter("child_frame").as_string();
    tf_topic_ = this->get_parameter("tf_topic").as_string();
    navigation_backend_param_ = this->get_parameter("navigation_backend").as_string();
    navigation_mode_param_ = this->get_parameter("navigation_mode").as_string();
    nav2_action_name_ = this->get_parameter("nav2_action_name").as_string();
    goal_frame_id_ = this->get_parameter("goal_frame_id").as_string();
    nav2_action_server_wait_s_ = this->get_parameter("nav2_action_server_wait_s").as_double();
    nav2_retry_period_s_ = this->get_parameter("nav2_retry_period_s").as_double();
    target_points_flat_ = this->get_parameter("target_points").as_double_array();
    trajectory_files_ = this->get_parameter("trajectory_files").as_string_array();

    if (navigation_backend_param_ == "LOCAL_PD") {
        navigation_backend_ = NavigationBackend::LOCAL_PD;
    } else if (navigation_backend_param_ == "NAV2") {
        navigation_backend_ = NavigationBackend::NAV2;
    } else {
        RCLCPP_WARN(this->get_logger(),
                    "Unknown navigation_backend '%s', fallback to LOCAL_PD.",
                    navigation_backend_param_.c_str());
        navigation_backend_param_ = "LOCAL_PD";
        navigation_backend_ = NavigationBackend::LOCAL_PD;
    }

    if (navigation_mode_param_ == "POINT_SEQUENCE") {
        nav_mode_ = NavMode::POINT_SEQUENCE;
    } else if (navigation_mode_param_ == "TRAJECTORY_SEQUENCE") {
        nav_mode_ = NavMode::TRAJECTORY_SEQUENCE;
    } else if (navigation_mode_param_ == "EXTERNAL_TARGET") {
        nav_mode_ = NavMode::EXTERNAL_TARGET;
    } else {
        RCLCPP_WARN(this->get_logger(),
                    "Unknown navigation_mode '%s', fallback to POINT_SEQUENCE.",
                    navigation_mode_param_.c_str());
        navigation_mode_param_ = "POINT_SEQUENCE";
        nav_mode_ = NavMode::POINT_SEQUENCE;
    }

    position_tolerance_ = this->get_parameter("position_tolerance").as_double();
    yaw_tolerance_ = this->get_parameter("yaw_tolerance").as_double();
    kp_x_ = this->get_parameter("kp_x").as_double();
    kp_y_ = this->get_parameter("kp_y").as_double();
    kp_yaw_ = this->get_parameter("kp_yaw").as_double();
    kd_x_ = this->get_parameter("kd_x").as_double();
    kd_y_ = this->get_parameter("kd_y").as_double();
    kd_yaw_ = this->get_parameter("kd_yaw").as_double();
    max_vx_ = this->get_parameter("max_vx").as_double();
    max_vy_ = this->get_parameter("max_vy").as_double();
    max_dyaw_ = this->get_parameter("max_dyaw").as_double();
    yaw_first_threshold_ = this->get_parameter("yaw_first_threshold").as_double();
    yaw_only_threshold_ = this->get_parameter("yaw_only_threshold").as_double();
    yaw_only_max_dyaw_ = this->get_parameter("yaw_only_max_dyaw").as_double();
    yaw_only_slowdown_threshold_ = this->get_parameter("yaw_only_slowdown_threshold").as_double();
    yaw_cmd_slew_rate_ = this->get_parameter("yaw_cmd_slew_rate").as_double();
    long_straight_distance_threshold_ =
        this->get_parameter("long_straight_distance_threshold").as_double();
    long_straight_yaw_threshold_ = this->get_parameter("long_straight_yaw_threshold").as_double();
    long_straight_kp_x_ = this->get_parameter("long_straight_kp_x").as_double();
    long_straight_max_vx_ = this->get_parameter("long_straight_max_vx").as_double();
    final_yaw_align_distance_ = this->get_parameter("final_yaw_align_distance").as_double();
    approach_yaw_max_dyaw_ = this->get_parameter("approach_yaw_max_dyaw").as_double();
    yaw_then_forward_enabled_slots_ =
        this->get_parameter("yaw_then_forward.enabled_slots").as_string_array();
    yaw_then_forward_align_yaw_tolerance_ =
        this->get_parameter("yaw_then_forward.align_yaw_tolerance").as_double();
    yaw_then_forward_finish_yaw_tolerance_ =
        this->get_parameter("yaw_then_forward.finish_yaw_tolerance").as_double();
    yaw_then_forward_forward_tolerance_ =
        this->get_parameter("yaw_then_forward.forward_tolerance").as_double();
    yaw_then_forward_lateral_tolerance_ =
        this->get_parameter("yaw_then_forward.lateral_tolerance").as_double();
    yaw_then_forward_kp_yaw_ = this->get_parameter("yaw_then_forward.kp_yaw").as_double();
    yaw_then_forward_max_wz_ = this->get_parameter("yaw_then_forward.max_wz").as_double();
    yaw_then_forward_wz_slew_rate_ =
        this->get_parameter("yaw_then_forward.wz_slew_rate").as_double();
    yaw_then_forward_kp_forward_ =
        this->get_parameter("yaw_then_forward.kp_forward").as_double();
    yaw_then_forward_max_vx_ = this->get_parameter("yaw_then_forward.max_vx").as_double();
    yaw_then_forward_kp_lateral_ =
        this->get_parameter("yaw_then_forward.kp_lateral").as_double();
    yaw_then_forward_max_vy_ = this->get_parameter("yaw_then_forward.max_vy").as_double();
    control_rate_ = this->get_parameter("control_rate").as_double();
    nav2_action_server_wait_s_ = std::max(0.0, nav2_action_server_wait_s_);
    nav2_retry_period_s_ = std::max(0.1, nav2_retry_period_s_);
    final_yaw_align_distance_ = std::max(0.0, final_yaw_align_distance_);
    approach_yaw_max_dyaw_ = std::max(0.0, approach_yaw_max_dyaw_);
    yaw_only_threshold_ = std::max(0.0, yaw_only_threshold_);
    yaw_only_max_dyaw_ = std::max(0.0, yaw_only_max_dyaw_);
    yaw_only_slowdown_threshold_ = std::max(yaw_tolerance_, yaw_only_slowdown_threshold_);
    yaw_cmd_slew_rate_ = std::max(0.0, yaw_cmd_slew_rate_);
    long_straight_distance_threshold_ = std::max(0.0, long_straight_distance_threshold_);
    long_straight_yaw_threshold_ = std::max(0.0, long_straight_yaw_threshold_);
    if (long_straight_kp_x_ <= 0.0) {
        long_straight_kp_x_ = kp_x_;
    }
    long_straight_max_vx_ = std::max(max_vx_, long_straight_max_vx_);
    yaw_then_forward_align_yaw_tolerance_ = std::max(0.0, yaw_then_forward_align_yaw_tolerance_);
    yaw_then_forward_finish_yaw_tolerance_ = std::max(0.0, yaw_then_forward_finish_yaw_tolerance_);
    yaw_then_forward_forward_tolerance_ = std::max(0.0, yaw_then_forward_forward_tolerance_);
    yaw_then_forward_lateral_tolerance_ = std::max(0.0, yaw_then_forward_lateral_tolerance_);
    yaw_then_forward_max_wz_ = std::max(0.0, yaw_then_forward_max_wz_);
    yaw_then_forward_wz_slew_rate_ = std::max(0.0, yaw_then_forward_wz_slew_rate_);
    yaw_then_forward_max_vx_ = std::max(0.0, yaw_then_forward_max_vx_);
    yaw_then_forward_max_vy_ = std::max(0.0, yaw_then_forward_max_vy_);
}

/**
 * @brief 解析扁平 target_points 参数为点序列
 *
 * 输入：target_points_flat_
 * 输出：point_targets_
 * 处理：每 3 个 double 组成 (x,y,yaw)；长度非 3 倍数则报错并清空
 */
void ObstacleNavNode::load_point_sequence() {
    point_targets_.clear();
    if (target_points_flat_.empty()) {
        return;
    }
    if (target_points_flat_.size() % 3 != 0) {
        RCLCPP_ERROR(this->get_logger(),
                     "target_points must be flat triples [x1,y1,yaw1,x2,y2,yaw2,...], got %zu values.",
                     target_points_flat_.size());
        return;
    }
    for (std::size_t i = 0; i < target_points_flat_.size(); i += 3) {
        point_targets_.push_back({target_points_flat_[i], target_points_flat_[i + 1], target_points_flat_[i + 2]});
    }
    RCLCPP_INFO(this->get_logger(), "Loaded %zu obstacle point targets.", point_targets_.size());
}

/**
 * @brief 从文本文件加载轨迹点列
 *
 * 输入：file_path
 * 输出：out_points；返回值表示是否成功
 * 处理：逐行解析 x y yaw，忽略空行与 # 注释
 */
bool ObstacleNavNode::load_trajectory_file(const std::string& file_path, std::vector<ObstacleNavPoint>& out_points) {
    out_points.clear();
    std::ifstream file(file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open obstacle trajectory file: %s", file_path.c_str());
        return false;
    }

    std::string line;
    std::size_t line_no = 0;
    while (std::getline(file, line)) {
        line_no++;
        const auto first_non_space = line.find_first_not_of(" \t\r\n");
        if (first_non_space == std::string::npos || line[first_non_space] == '#') {
            continue;
        }

        std::istringstream iss(line);
        ObstacleNavPoint point{};
        if (!(iss >> point.x >> point.y >> point.yaw)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid trajectory format at %s:%zu: %s",
                         file_path.c_str(), line_no, line.c_str());
            out_points.clear();
            return false;
        }
        out_points.push_back(point);
    }

    if (out_points.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Trajectory file has no valid points: %s", file_path.c_str());
        return false;
    }
    return true;
}

/**
 * @brief 返回当前导航模式下的障碍目标总数
 *
 * 输入：nav_mode_、point_targets_、trajectory_files_
 * 输出：障碍目标数量
 * 处理：POINT_SEQUENCE 用点数，TRAJECTORY_SEQUENCE 用轨迹文件数
 */
std::size_t ObstacleNavNode::total_target_count() const {
    if (nav_mode_ == NavMode::EXTERNAL_TARGET) {
        return external_target_received_ ? static_cast<std::size_t>(external_target_index_ + 1) : 0U;
    }
    if (nav_mode_ == NavMode::POINT_SEQUENCE) {
        return point_targets_.size();
    }
    return trajectory_files_.size();
}

/**
 * @brief 按 current_target_index_ 加载当前障碍的路径
 *
 * 输入：current_target_index_、nav_mode_、point_targets_ 或 trajectory_files_
 * 输出：current_path_、current_path_index_=0；重置 PD 与 nav_status_sent_
 * 处理：点模式压入单点；轨迹模式读文件；索引越界或读失败返回 false
 */
bool ObstacleNavNode::load_current_target() {
    current_path_.clear();
    current_path_index_ = 0;
    reset_pd_state();
    reset_yaw_then_forward_state();
    nav_status_sent_ = false;

    if (nav_mode_ == NavMode::EXTERNAL_TARGET) {
        if (!external_target_received_) {
            return false;
        }
        current_target_index_ = static_cast<std::size_t>(std::max(external_target_index_, 0));
        current_path_.push_back(external_target_);
        RCLCPP_INFO(this->get_logger(), "Loaded external obstacle target %d: (%.2f, %.2f, %.2f)",
                    external_target_index_, external_target_.x, external_target_.y, external_target_.yaw);
        return true;
    }

    if (nav_mode_ == NavMode::POINT_SEQUENCE) {
        if (current_target_index_ >= point_targets_.size()) {
            return false;
        }
        current_path_.push_back(point_targets_[current_target_index_]);
        RCLCPP_INFO(this->get_logger(), "Loaded obstacle target %zu/%zu: (%.2f, %.2f, %.2f)",
                    current_target_index_ + 1, point_targets_.size(),
                    current_path_[0].x, current_path_[0].y, current_path_[0].yaw);
        return true;
    }

    if (current_target_index_ >= trajectory_files_.size()) {
        return false;
    }
    if (!load_trajectory_file(trajectory_files_[current_target_index_], current_path_)) {
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Loaded obstacle trajectory %zu/%zu: %s (%zu points)",
                current_target_index_ + 1, trajectory_files_.size(),
                trajectory_files_[current_target_index_].c_str(), current_path_.size());
    return true;
}

void ObstacleNavNode::external_target_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 4) {
        RCLCPP_WARN(this->get_logger(),
                    "Invalid obstacle_sequence/nav_target; expected [slot_index, x, y, yaw].");
        return;
    }

    const int new_index = static_cast<int>(std::lround(msg->data[0]));
    const ObstacleNavPoint new_target{msg->data[1], msg->data[2], msg->data[3]};
    const bool same_target =
        external_target_received_ &&
        new_index == external_target_index_ &&
        std::abs(new_target.x - external_target_.x) < 1e-4 &&
        std::abs(new_target.y - external_target_.y) < 1e-4 &&
        std::abs(normalize_angle(new_target.yaw - external_target_.yaw)) < 1e-4;
    if (same_target) {
        return;
    }

    if (using_nav2_backend()) {
        cancel_nav2_goal();
    }

    external_target_index_ = new_index;
    external_target_ = new_target;
    external_target_received_ = true;
    nav_finished_sent_ = false;
    nav_status_sent_ = false;
    current_target_index_ = static_cast<std::size_t>(std::max(external_target_index_, 0));
    runtime_state_ = nav_enabled_ ? RuntimeState::RUNNING : RuntimeState::IDLE;
    if (load_current_target() && nav_enabled_ && using_nav2_backend()) {
        send_nav2_goal_for_current_point();
    }
}

void ObstacleNavNode::sequence_slot_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == active_sequence_slot_) {
        return;
    }
    active_sequence_slot_ = msg->data;
    reset_yaw_then_forward_state();
}

/**
 * @brief nav_enable 话题回调入口
 *
 * 输入：msg->data 为新的使能状态
 * 输出：更新 nav_enabled_，分发禁用/上升沿处理
 * 处理：记录 previous 后更新，false 走 on_nav_enable_disabled，上升沿走 on_nav_enable_rising_edge
 */
void ObstacleNavNode::nav_enable_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    const bool previous = nav_enabled_;
    nav_enabled_ = msg->data;

    if (!nav_enabled_) {
        on_nav_enable_disabled(previous);
        return;
    }

    if (!previous) {
        on_nav_enable_rising_edge();
    }
}

/**
 * @brief 处理 nav_enable 由 true 变 false
 *
 * 输入：previous_enabled（变更前是否使能）、runtime_state_
 * 输出：可能取消 Nav2 goal、更新 runtime_state_
 * 处理：从 true 变 false 时静默交出 cmd_vel；RUNNING/WAIT_TASK→WAIT_NEXT_ENABLE，其它非 FINISHED→IDLE
 */
void ObstacleNavNode::on_nav_enable_disabled(bool previous_enabled) {
    if (previous_enabled) {
        if (using_nav2_backend()) {
            cancel_nav2_goal();
        }
    }
    if (runtime_state_ == RuntimeState::WAIT_TASK || runtime_state_ == RuntimeState::RUNNING) {
        runtime_state_ = RuntimeState::WAIT_NEXT_ENABLE;
    } else if (runtime_state_ != RuntimeState::FINISHED) {
        runtime_state_ = RuntimeState::IDLE;
    }
}

/**
 * @brief 处理 nav_enable 由 false 变 true（上升沿）
 *
 * 输入：runtime_state_、current_target_index_、total_target_count()
 * 输出：runtime_state_、可能 load current_path_、发布 nav_finished
 * 处理：FINISHED 仅重发 nav_finished；无剩余目标→FINISHED；否则 load 成功→RUNNING
 */
void ObstacleNavNode::on_nav_enable_rising_edge() {
    if (nav_mode_ == NavMode::EXTERNAL_TARGET) {
        if (!external_target_received_) {
            runtime_state_ = RuntimeState::IDLE;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "EXTERNAL_TARGET mode waiting for obstacle_sequence/nav_target.");
            return;
        }
        if (load_current_target()) {
            runtime_state_ = RuntimeState::RUNNING;
            if (using_nav2_backend()) {
                send_nav2_goal_for_current_point();
            }
        }
        return;
    }

    if (runtime_state_ == RuntimeState::FINISHED) {
        publish_nav_finished_once();
        return;
    }
    if (current_target_index_ >= total_target_count()) {
        runtime_state_ = RuntimeState::FINISHED;
        publish_nav_finished_once();
        stop_robot();
        return;
    }
    if (load_current_target()) {
        runtime_state_ = RuntimeState::RUNNING;
        if (using_nav2_backend()) {
            send_nav2_goal_for_current_point();
        }
    } else {
        runtime_state_ = RuntimeState::FINISHED;
        publish_nav_finished_once();
        stop_robot();
    }
}

bool ObstacleNavNode::using_nav2_backend() const {
    return navigation_backend_ == NavigationBackend::NAV2;
}

geometry_msgs::msg::Quaternion ObstacleNavNode::yaw_to_quaternion(double yaw) const {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}

void ObstacleNavNode::send_nav2_goal_for_current_point() {
    if (!using_nav2_backend() || !nav_enabled_ || runtime_state_ != RuntimeState::RUNNING) {
        return;
    }
#ifndef TASK_MANAGER_HAS_NAV2_MSGS
    stop_robot();
    runtime_state_ = RuntimeState::WAIT_NEXT_ENABLE;
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "NAV2 backend requested but nav2_msgs was not found at build time.");
    return;
#else
    if (current_path_index_ >= current_path_.size()) {
        try_advance_past_empty_path();
        return;
    }
    if (!nav2_action_client_) {
        RCLCPP_ERROR(this->get_logger(), "NAV2 backend selected but action client is not initialized.");
        return;
    }
    if (nav2_goal_in_flight_) {
        return;
    }

    const auto now = this->now();
    if (nav2_goal_pending_retry_ &&
        nav2_last_send_attempt_time_.nanoseconds() != 0 &&
        (now - nav2_last_send_attempt_time_).seconds() < nav2_retry_period_s_) {
        return;
    }
    nav2_last_send_attempt_time_ = now;

    const auto wait_duration = std::chrono::duration<double>(nav2_action_server_wait_s_);
    if (!nav2_action_client_->wait_for_action_server(wait_duration)) {
        nav2_goal_pending_retry_ = true;
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Waiting for Nav2 action server '%s'...",
            nav2_action_name_.c_str());
        return;
    }

    const auto& target = current_path_[current_path_index_];
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = goal_frame_id_;
    goal.pose.header.stamp = now;
    goal.pose.pose.position.x = target.x;
    goal.pose.pose.position.y = target.y;
    goal.pose.pose.position.z = 0.0;
    goal.pose.pose.orientation = yaw_to_quaternion(target.yaw);

    const int64_t goal_request_id = ++nav2_goal_request_id_;
    active_nav2_goal_request_id_ = goal_request_id;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
        [this, goal_request_id](const GoalHandleNavigateToPose::SharedPtr& goal_handle) {
            nav2_goal_response_callback(goal_request_id, goal_handle);
        };
    options.result_callback =
        [this, goal_request_id](const GoalHandleNavigateToPose::WrappedResult& result) {
            nav2_result_callback(goal_request_id, result);
        };

    nav2_goal_in_flight_ = true;
    nav2_goal_accepted_ = false;
    nav2_goal_pending_retry_ = false;
    nav2_action_client_->async_send_goal(goal, options);
    publish_debug(target, 0.0, 0.0, 0.0);

    RCLCPP_INFO(this->get_logger(),
                "Sent Nav2 goal for obstacle target=%zu point=%zu/%zu frame=%s pose=(%.3f, %.3f, %.3f)",
                current_target_index_ + 1,
                current_path_index_ + 1,
                current_path_.size(),
                goal_frame_id_.c_str(),
                target.x,
                target.y,
                target.yaw);
#endif
}

void ObstacleNavNode::cancel_nav2_goal() {
    nav2_goal_pending_retry_ = false;
    nav2_goal_in_flight_ = false;
    nav2_goal_accepted_ = false;
    active_nav2_goal_request_id_ = ++nav2_goal_request_id_;

#ifdef TASK_MANAGER_HAS_NAV2_MSGS
    if (nav2_action_client_ && nav2_active_goal_) {
        nav2_action_client_->async_cancel_goal(nav2_active_goal_);
    }
    nav2_active_goal_.reset();
#endif
}

#ifdef TASK_MANAGER_HAS_NAV2_MSGS
void ObstacleNavNode::nav2_goal_response_callback(
    int64_t goal_request_id,
    const GoalHandleNavigateToPose::SharedPtr& goal_handle)
{
    if (goal_request_id != active_nav2_goal_request_id_) {
        return;
    }
    if (!goal_handle) {
        nav2_goal_in_flight_ = false;
        nav2_goal_accepted_ = false;
        nav2_active_goal_.reset();
        stop_robot();
        runtime_state_ = RuntimeState::WAIT_NEXT_ENABLE;
        RCLCPP_ERROR(this->get_logger(), "Nav2 rejected obstacle navigation goal.");
        return;
    }

    nav2_active_goal_ = goal_handle;
    nav2_goal_accepted_ = true;
    RCLCPP_INFO(this->get_logger(), "Nav2 accepted obstacle navigation goal.");
}

void ObstacleNavNode::nav2_result_callback(
    int64_t goal_request_id,
    const GoalHandleNavigateToPose::WrappedResult& result)
{
    if (goal_request_id != active_nav2_goal_request_id_) {
        return;
    }

    nav2_goal_in_flight_ = false;
    nav2_goal_accepted_ = false;
    nav2_active_goal_.reset();

    if (!nav_enabled_) {
        return;
    }

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        if (current_path_index_ < current_path_.size()) {
            const auto target = current_path_[current_path_index_];
            on_waypoint_reached(target);
        }
        if (nav_enabled_ && runtime_state_ == RuntimeState::RUNNING &&
            current_path_index_ < current_path_.size()) {
            send_nav2_goal_for_current_point();
        }
        return;
    }

    stop_robot();
    runtime_state_ = RuntimeState::WAIT_NEXT_ENABLE;
    RCLCPP_WARN(this->get_logger(),
                "Nav2 obstacle goal ended without success. result_code=%d",
                static_cast<int>(result.code));
}
#endif

void ObstacleNavNode::handle_nav2_control_loop() {
    if (runtime_state_ != RuntimeState::RUNNING) {
        return;
    }
    if (current_path_index_ >= current_path_.size()) {
        try_advance_past_empty_path();
        return;
    }

    const auto& target = current_path_[current_path_index_];
    publish_debug(target, 0.0, 0.0, 0.0);

    if (!nav2_goal_in_flight_) {
        send_nav2_goal_for_current_point();
    }
}

/**
 * @brief 从 TF 消息更新机器人位姿
 *
 * 输入：msg 中 target_frame_→child_frame_ 的变换
 * 输出：x_、y_、yaw_、got_tf_
 * 处理：匹配帧名后取平移与四元数 RPY
 */
void ObstacleNavNode::tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    for (const auto& transform : msg->transforms) {
        if (transform.header.frame_id == target_frame_ && transform.child_frame_id == child_frame_) {
            x_ = transform.transform.translation.x;
            y_ = transform.transform.translation.y;
            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch;
            m.getRPY(roll, pitch, yaw_);
            got_tf_ = true;
            break;
        }
    }
}

/**
 * @brief 发布当前位姿调试话题（有 TF 时）
 *
 * 输入：got_tf_、x_、y_、yaw_
 * 输出：obstacle_nav/current_pose
 * 处理：组装 Float32MultiArray 并 publish
 */
void ObstacleNavNode::publish_current_pose_debug() {
    if (!got_tf_) {
        return;
    }
    std_msgs::msg::Float32MultiArray pose_msg;
    pose_msg.data = {
        static_cast<float>(x_),
        static_cast<float>(y_),
        static_cast<float>(yaw_)
    };
    current_pose_pub_->publish(pose_msg);
}

/**
 * @brief 使能状态下处于 FINISHED 时的处理
 *
 * 输入：runtime_state_ == FINISHED
 * 输出：零速度、nav_finished（单次）
 * 处理：stop_robot 并 publish_nav_finished_once
 */
void ObstacleNavNode::handle_finished_while_enabled() {
    stop_robot();
    publish_nav_finished_once();
}

/**
 * @brief 防御：路径索引已越界时完成当前障碍目标
 *
 * 输入：current_path_index_、current_path_.size()
 * 输出：若越界则停车、nav_status、target_index++、WAIT_TASK；返回是否已处理并应 return
 * 处理：与到齐整条路径后的收尾逻辑相同
 */
bool ObstacleNavNode::try_advance_past_empty_path() {
    if (current_path_index_ < current_path_.size()) {
        return false;
    }
    stop_robot();
    publish_nav_status_once();
    current_target_index_++;
    runtime_state_ = RuntimeState::WAIT_TASK;
    return true;
}

/**
 * @brief 计算当前路点跟踪误差（雷达系→机体系）
 *
 * 输入：target、x_/y_/yaw_
 * 输出：WaypointTrackingErrors（位置误差、最终 yaw 误差、朝目标点 yaw 误差）
 * 处理：世界系差分后按当前 yaw 旋转到机体系
 */
WaypointTrackingErrors ObstacleNavNode::compute_waypoint_errors(const ObstacleNavPoint& target) const {
    const double dx_world = target.x - x_;
    const double dy_world = target.y - y_;
    const double err_yaw = normalize_angle(target.yaw - yaw_);
    const double dist = std::hypot(dx_world, dy_world);
    constexpr double kHalfPi = 1.5707963267948966;
    const double approach_yaw = normalize_angle(std::atan2(dy_world, dx_world) - kHalfPi);
    const double err_approach_yaw = normalize_angle(approach_yaw - yaw_);
    const double err_x_body = std::cos(yaw_) * dy_world - std::sin(yaw_) * dx_world;
    const double err_y_body = -std::cos(yaw_) * dx_world - std::sin(yaw_) * dy_world;
    return {dist, err_x_body, err_y_body, err_yaw, err_approach_yaw};
}

/**
 * @brief 判断是否到达当前路点
 *
 * 输入：dist、err_yaw、position_tolerance_、yaw_tolerance_
 * 输出：是否到点
 * 处理：平面距离与航向误差均小于容差
 */
bool ObstacleNavNode::is_waypoint_reached(double dist, double err_yaw) const {
    return dist < position_tolerance_ && std::abs(err_yaw) < yaw_tolerance_;
}

/**
 * @brief 到达当前路点后的索引与状态更新
 *
 * 输入：target（用于日志）、current_path_index_、current_path_
 * 输出：path_index++；若路径走完则 nav_status、target_index++、WAIT_TASK
 * 处理：停车、reset_pd_state；整条 current_path_ 完成时发 nav_status
 */
void ObstacleNavNode::on_waypoint_reached(const ObstacleNavPoint& target) {
    stop_robot();
    RCLCPP_INFO(this->get_logger(), "Reached nav point %zu/%zu of obstacle target %zu. pos=(%.2f, %.2f, %.2f)",
                current_path_index_ + 1, current_path_.size(), current_target_index_ + 1,
                target.x, target.y, target.yaw);
    current_path_index_++;
    reset_pd_state();

    if (current_path_index_ >= current_path_.size()) {
        publish_nav_status_once();
        current_target_index_++;
        runtime_state_ = RuntimeState::WAIT_TASK;
    }
}

/**
 * @brief PD 速度控制并发布 cmd_vel
 *
 * 输入：errors、dist；成员 PD 增益与历史
 * 输出：cmd_vel、更新 pd_initialized_ 与 prev_err_*
 * 处理：远距离边走边朝目标点转向；近点改为最终 yaw；位置到达后只调最终 yaw
 */
void ObstacleNavNode::compute_pd_velocity_cmd(const WaypointTrackingErrors& errors, double dist) {
    const auto now = this->now();
    const bool position_close = dist <= position_tolerance_;
    const bool final_yaw_phase = position_close || dist <= final_yaw_align_distance_;
    const double yaw_error = final_yaw_phase ? errors.err_yaw : errors.err_approach_yaw;
    if (!pd_initialized_) {
        prev_err_x_body_ = errors.err_x_body;
        prev_err_y_body_ = errors.err_y_body;
        prev_err_yaw_ = yaw_error;
        prev_ctrl_time_ = now;
        pd_initialized_ = true;
    }
    double dt = (now - prev_ctrl_time_).seconds();
    dt = std::max(1e-4, std::min(dt, 0.25));

    const double dex_dt = (errors.err_x_body - prev_err_x_body_) / dt;
    const double dey_dt = (errors.err_y_body - prev_err_y_body_) / dt;
    const double deyaw_dt = normalize_angle(yaw_error - prev_err_yaw_) / dt;

    prev_err_x_body_ = errors.err_x_body;
    prev_err_y_body_ = errors.err_y_body;
    prev_err_yaw_ = yaw_error;
    prev_ctrl_time_ = now;

    geometry_msgs::msg::Twist cmd;
    const double wz = kp_yaw_ * yaw_error + kd_yaw_ * deyaw_dt;
    const bool yaw_only = std::abs(yaw_error) > yaw_only_threshold_;
    const bool final_yaw_align_only = final_yaw_phase && std::abs(errors.err_yaw) > yaw_tolerance_;
    const bool turn_in_place = position_close || yaw_only || final_yaw_align_only;
    const bool long_straight =
        !final_yaw_phase &&
        dist >= long_straight_distance_threshold_ &&
        std::abs(errors.err_approach_yaw) <= long_straight_yaw_threshold_ &&
        errors.err_x_body > 0.0;
    double yaw_limit = final_yaw_phase ? max_dyaw_ : std::min(max_dyaw_, approach_yaw_max_dyaw_);
    if (turn_in_place) {
        const double slowdown_scale =
            std::min(1.0, std::abs(yaw_error) / yaw_only_slowdown_threshold_);
        yaw_limit = yaw_only_max_dyaw_ * slowdown_scale;
    }
    double target_wz = std::clamp(wz, -yaw_limit, yaw_limit);
    if (yaw_cmd_slew_rate_ > 0.0) {
        if (target_wz * prev_cmd_wz_ < 0.0) {
            prev_cmd_wz_ = 0.0;
        }
        const double max_delta_wz = yaw_cmd_slew_rate_ * dt;
        target_wz = std::clamp(
            target_wz,
            prev_cmd_wz_ - max_delta_wz,
            prev_cmd_wz_ + max_delta_wz);
    }
    prev_cmd_wz_ = target_wz;

    if (turn_in_place) {
        cmd.angular.z = target_wz;
    } else {
        const double max_forward_vx = long_straight ? long_straight_max_vx_ : max_vx_;
        const double active_kp_x = long_straight ? long_straight_kp_x_ : kp_x_;
        cmd.linear.x =
            std::clamp(active_kp_x * errors.err_x_body + kd_x_ * dex_dt, -max_vx_, max_forward_vx);
        cmd.linear.y = std::clamp(kp_y_ * errors.err_y_body + kd_y_ * dey_dt, -max_vy_, max_vy_);
        cmd.angular.z = target_wz;
    }
    cmd_vel_pub_->publish(cmd);

    const char* phase = yaw_only
        ? "yaw_only"
        : (position_close
            ? "align"
            : (final_yaw_align_only
                ? "final_yaw_align_only"
                : (long_straight ? "long_straight" : (final_yaw_phase ? "final_yaw" : "approach"))));
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "ObstacleNav target=%zu point=%zu/%zu phase=%s dist=%.2f err_body=(%.2f,%.2f) "
        "err_yaw(final=%.2f, approach=%.2f, active=%.2f) cmd=(%.2f,%.2f,%.2f)",
        current_target_index_ + 1, current_path_index_ + 1, current_path_.size(),
        phase, dist, errors.err_x_body, errors.err_y_body,
        errors.err_yaw, errors.err_approach_yaw, yaw_error,
        cmd.linear.x, cmd.linear.y, cmd.angular.z);
}

bool ObstacleNavNode::yaw_then_forward_enabled_for_current_slot() const {
    if (nav_mode_ != NavMode::EXTERNAL_TARGET || using_nav2_backend()) {
        return false;
    }
    return std::find(
        yaw_then_forward_enabled_slots_.begin(),
        yaw_then_forward_enabled_slots_.end(),
        active_sequence_slot_) != yaw_then_forward_enabled_slots_.end();
}

double ObstacleNavNode::slew_yaw_then_forward_wz(double target_wz) {
    const auto now = this->now();
    if (!yaw_then_forward_initialized_) {
        yaw_then_forward_prev_ctrl_time_ = now;
        yaw_then_forward_prev_wz_ = 0.0;
        yaw_then_forward_initialized_ = true;
    }
    double dt = (now - yaw_then_forward_prev_ctrl_time_).seconds();
    dt = std::max(1e-4, std::min(dt, 0.25));

    if (yaw_then_forward_wz_slew_rate_ > 0.0) {
        if (target_wz * yaw_then_forward_prev_wz_ < 0.0) {
            yaw_then_forward_prev_wz_ = 0.0;
        }
        const double max_delta_wz = yaw_then_forward_wz_slew_rate_ * dt;
        target_wz = std::clamp(
            target_wz,
            yaw_then_forward_prev_wz_ - max_delta_wz,
            yaw_then_forward_prev_wz_ + max_delta_wz);
    }

    yaw_then_forward_prev_wz_ = target_wz;
    yaw_then_forward_prev_ctrl_time_ = now;
    return target_wz;
}

void ObstacleNavNode::reset_yaw_then_forward_state() {
    yaw_then_forward_phase_ = YawThenForwardPhase::ALIGN_YAW;
    yaw_then_forward_initialized_ = false;
    yaw_then_forward_prev_wz_ = 0.0;
}

bool ObstacleNavNode::handle_yaw_then_forward_control(
    const ObstacleNavPoint& target,
    const WaypointTrackingErrors& errors)
{
    if (!yaw_then_forward_enabled_for_current_slot()) {
        return false;
    }

    if (yaw_then_forward_phase_ == YawThenForwardPhase::ALIGN_YAW &&
        std::abs(errors.err_yaw) <= yaw_then_forward_align_yaw_tolerance_) {
        yaw_then_forward_phase_ = YawThenForwardPhase::FORWARD;
        yaw_then_forward_initialized_ = false;
        yaw_then_forward_prev_wz_ = 0.0;
        RCLCPP_INFO(this->get_logger(),
                    "YawThenForward slot='%s': yaw aligned, moving forward.",
                    active_sequence_slot_.c_str());
    }

    const bool forward_aligned =
        std::abs(errors.err_x_body) <= yaw_then_forward_forward_tolerance_;
    const bool lateral_aligned =
        std::abs(errors.err_y_body) <= yaw_then_forward_lateral_tolerance_;
    const bool yaw_aligned =
        std::abs(errors.err_yaw) <= yaw_then_forward_finish_yaw_tolerance_;

    if (yaw_then_forward_phase_ == YawThenForwardPhase::FORWARD &&
        forward_aligned && lateral_aligned && yaw_aligned) {
        on_waypoint_reached(target);
        return true;
    }

    geometry_msgs::msg::Twist cmd;
    const double raw_wz = std::clamp(
        yaw_then_forward_kp_yaw_ * errors.err_yaw,
        -yaw_then_forward_max_wz_,
        yaw_then_forward_max_wz_);
    cmd.angular.z = slew_yaw_then_forward_wz(raw_wz);

    if (yaw_then_forward_phase_ == YawThenForwardPhase::FORWARD) {
        cmd.linear.x = std::clamp(
            yaw_then_forward_kp_forward_ * errors.err_x_body,
            -yaw_then_forward_max_vx_,
            yaw_then_forward_max_vx_);
        cmd.linear.y = std::clamp(
            yaw_then_forward_kp_lateral_ * errors.err_y_body,
            -yaw_then_forward_max_vy_,
            yaw_then_forward_max_vy_);
    }

    cmd_vel_pub_->publish(cmd);

    const char* phase =
        yaw_then_forward_phase_ == YawThenForwardPhase::ALIGN_YAW ? "align_yaw" : "forward";
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "YawThenForward slot='%s' phase=%s target=(%.2f,%.2f,%.2f) "
        "err_body=(%.2f,%.2f) err_yaw=%.2f cmd=(%.2f,%.2f,%.2f)",
        active_sequence_slot_.c_str(), phase, target.x, target.y, target.yaw,
        errors.err_x_body, errors.err_y_body, errors.err_yaw,
        cmd.linear.x, cmd.linear.y, cmd.angular.z);
    return true;
}

/**
 * @brief 定时控制循环主入口
 *
 * 输入：nav_enabled_、runtime_state_、TF、current_path_
 * 输出：调试位姿、cmd_vel、nav_status/nav_finished（经子函数）
 * 处理：按原 early-return 顺序调用各子步骤
 */
void ObstacleNavNode::control_loop() {
    publish_current_pose_debug();

    if (!nav_enabled_) {
        return;
    }

    if (runtime_state_ == RuntimeState::FINISHED) {
        handle_finished_while_enabled();
        return;
    }

    if (runtime_state_ != RuntimeState::RUNNING) {
        return;
    }

    if (using_nav2_backend()) {
        handle_nav2_control_loop();
        return;
    }

    if (!got_tf_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "ObstacleNav: waiting for TF %s -> %s",
                             target_frame_.c_str(), child_frame_.c_str());
        return;
    }

    if (stop_on_lidar_drift("ObstacleNav")) {
        return;
    }

    if (try_advance_past_empty_path()) {
        return;
    }

    const auto& target = current_path_[current_path_index_];
    const WaypointTrackingErrors errors = compute_waypoint_errors(target);

    publish_debug(target, errors.err_x_body, errors.err_y_body, errors.err_yaw);

    if (handle_yaw_then_forward_control(target, errors)) {
        return;
    }

    if (is_waypoint_reached(errors.dist, errors.err_yaw)) {
        on_waypoint_reached(target);
        return;
    }

    compute_pd_velocity_cmd(errors, errors.dist);
}

/**
 * @brief 雷达位姿越界时停车并告警
 *
 * 输入：context（日志前缀）、x_/y_
 * 输出：true 表示已越界并已 stop_robot
 * 处理：x∉[-5,5] 或 y∉[0,10] 时 throttle 提示「雷达飘了」
 */
bool ObstacleNavNode::stop_on_lidar_drift(const char* context) {
    if (!task_manager::lidar_pose_guard::is_out_of_bounds(x_, y_)) {
        return false;
    }
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "%s: 雷达飘了! pose(%.2f, %.2f) out of bounds [x:%.0f,%.0f] [y:%.0f,%.0f]",
        context, x_, y_,
        task_manager::lidar_pose_guard::kMinX, task_manager::lidar_pose_guard::kMaxX,
        task_manager::lidar_pose_guard::kMinY, task_manager::lidar_pose_guard::kMaxY);
    stop_robot();
    return true;
}

/**
 * @brief 将角度归一化到 [-pi, pi]
 *
 * 输入：angle（弧度）
 * 输出：归一化后的角度
 * 处理：循环加减 2*pi
 */
double ObstacleNavNode::normalize_angle(double angle) const {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

/**
 * @brief 发布零速度停车
 *
 * 输入：cmd_vel_pub_
 * 输出：cmd_vel 全零
 * 处理：构造默认 Twist 并 publish
 */
void ObstacleNavNode::stop_robot() {
    geometry_msgs::msg::Twist cmd;
    cmd_vel_pub_->publish(cmd);
}

/**
 * @brief 单次发布当前障碍目标到达信号
 *
 * 输入：nav_status_sent_
 * 输出：nav_status=true（至多一次）
 * 处理：未发送过则 publish 并置 nav_status_sent_
 */
void ObstacleNavNode::publish_nav_status_once() {
    if (nav_status_sent_) {
        return;
    }
    std_msgs::msg::Bool msg;
    msg.data = true;
    nav_status_pub_->publish(msg);
    nav_status_sent_ = true;
    RCLCPP_INFO(this->get_logger(), "Published nav_status=true for obstacle target arrival.");
}

/**
 * @brief 单次发布全部障碍导航完成信号
 *
 * 输入：nav_finished_sent_
 * 输出：nav_finished=true（至多一次）
 * 处理：未发送过则 publish 并置 nav_finished_sent_
 */
void ObstacleNavNode::publish_nav_finished_once() {
    if (nav_finished_sent_) {
        return;
    }
    std_msgs::msg::Bool msg;
    msg.data = true;
    nav_finished_pub_->publish(msg);
    nav_finished_sent_ = true;
    RCLCPP_INFO(this->get_logger(), "Published nav_finished=true. No more obstacle nav targets.");
}

/**
 * @brief 发布当前目标与机体系误差调试话题
 *
 * 输入：target、机体系误差三分量
 * 输出：obstacle_nav/current_target、obstacle_nav/debug_error
 * 处理：分别组装 MultiArray 与 Point 发布
 */
void ObstacleNavNode::publish_debug(const ObstacleNavPoint& target, double err_x_body, double err_y_body, double err_yaw) {
    std_msgs::msg::Float32MultiArray target_msg;
    target_msg.data = {
        static_cast<float>(target.x),
        static_cast<float>(target.y),
        static_cast<float>(target.yaw)
    };
    current_target_pub_->publish(target_msg);

    geometry_msgs::msg::Point err_msg;
    err_msg.x = err_x_body;
    err_msg.y = err_y_body;
    err_msg.z = err_yaw;
    debug_error_pub_->publish(err_msg);
}

/**
 * @brief 重置 PD 微分状态
 *
 * 输入：无
 * 输出：pd_initialized_=false，prev_err_* 清零
 * 处理：换路点时调用，避免微分项跳变
 */
void ObstacleNavNode::reset_pd_state() {
    pd_initialized_ = false;
    prev_err_x_body_ = 0.0;
    prev_err_y_body_ = 0.0;
    prev_err_yaw_ = 0.0;
    prev_cmd_wz_ = 0.0;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleNavNode>());
    rclcpp::shutdown();
    return 0;
}
