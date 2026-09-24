#include "task_manager/task_executor_node.hpp"
#include "task_manager/lidar_pose_guard.hpp"
#include <cmath>
#include <algorithm>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>

/**
 * @brief 构造任务执行器节点
 *
 * 输入：ROS 参数（load_parameters）
 * 输出：初始化成员与 ROS 接口
 * 处理：加载参数后 setup_ros_interfaces
 */
TaskExecutorNode::TaskExecutorNode()
    : Node("task_executor_node"),
      current_mode_(ExecutorMode::IDLE),
      x_(0.0), y_(0.0), yaw_(0.0), got_tf_(false)
{
    load_parameters();
    setup_ros_interfaces();
    RCLCPP_INFO(this->get_logger(), "TaskExecutorNode Initialized.");
}

/**
 * @brief 创建订阅、发布与控制定时器
 *
 * 输入：tf_topic_ 等成员
 * 输出：各 sub/pub 与 50Hz timer_
 * 处理：与原构造中接口注册一致
 */
void TaskExecutorNode::setup_ros_interfaces() {
    tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        tf_topic_, 100, std::bind(&TaskExecutorNode::tf_callback, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<std_msgs::msg::String>(
        "current_state", 10, std::bind(&TaskExecutorNode::state_callback, this, std::placeholders::_1));

    qr_sub_ = this->create_subscription<quad::msg::QrResult>(
        "qr_detection_result", 10, std::bind(&TaskExecutorNode::qr_callback, this, std::placeholders::_1));
    sequence_index_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "obstacle_sequence/active_index", 10,
        std::bind(&TaskExecutorNode::sequence_index_callback, this, std::placeholders::_1));
    sequence_segment_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "obstacle_sequence/active_segment", 10,
        std::bind(&TaskExecutorNode::sequence_segment_callback, this, std::placeholders::_1));
    sequence_slot_sub_ = this->create_subscription<std_msgs::msg::String>(
        "obstacle_sequence/current_slot", 10,
        std::bind(&TaskExecutorNode::sequence_slot_callback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    task_done_pub_ = this->create_publisher<std_msgs::msg::Int32>("task_done", 10);
    target_pub_ = this->create_publisher<geometry_msgs::msg::Point>("visual_servoing/target", 10);
    error_debug_pub_ = this->create_publisher<geometry_msgs::msg::Point>("visual_servoing/debug_errors", 10);
    lidar_pose_pub_ = this->create_publisher<geometry_msgs::msg::Point>(lidar_pose_topic_, 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&TaskExecutorNode::control_loop, this));
}

/**
 * @brief 加载各类任务所需的配置参数
 *
 * 输入：节点参数服务器
 * 输出：过桥/楼梯/匍匐/视觉/绕杆相关成员
 * 处理：declare 与 get 顺序与原实现一致
 */
void TaskExecutorNode::load_parameters() {
    this->declare_parameter("target_frame", "camera_init");
    this->declare_parameter("child_frame", "aft_mapped");
    this->declare_parameter("tf_topic", "/tf");
    this->declare_parameter("lidar_pose_topic", "lidar_pose_xyyaw");
    this->declare_parameter("straight_target_yaw", 0.0);
    this->declare_parameter("straight_target_dist", 4.0);
    this->declare_parameter("straight_max_speed", 0.5);
    this->declare_parameter("straight_kp_y", 2.0);
    this->declare_parameter("kp_xy", 0.5);
    this->declare_parameter("kp_yaw", 1.0);
    this->declare_parameter("max_vy", 0.5);
    this->declare_parameter("max_dyaw", 0.5);

    target_frame_ = this->get_parameter("target_frame").as_string();
    child_frame_ = this->get_parameter("child_frame").as_string();
    tf_topic_ = this->get_parameter("tf_topic").as_string();
    lidar_pose_topic_ = this->get_parameter("lidar_pose_topic").as_string();
    straight_target_yaw_ = this->get_parameter("straight_target_yaw").as_double();
    straight_target_dist_ = this->get_parameter("straight_target_dist").as_double();
    straight_max_speed_ = this->get_parameter("straight_max_speed").as_double();
    straight_kp_y_ = this->get_parameter("straight_kp_y").as_double();
    kp_xy_ = this->get_parameter("kp_xy").as_double();
    kp_yaw_ = this->get_parameter("kp_yaw").as_double();
    max_vy_ = this->get_parameter("max_vy").as_double();
    max_dyaw_ = this->get_parameter("max_dyaw").as_double();

    this->declare_parameter("tasks.stair_duration", 4.0);
    this->declare_parameter("tasks.stair_target_yaw", 0.0);
    this->declare_parameter("tasks.stair_use_current_yaw", true);
    this->declare_parameter("tasks.stair_forward_speed", 0.25);
    this->declare_parameter("tasks.stair_kp_y", 2.0);
    this->declare_parameter("tasks.stair_kp_yaw", 1.0);
    this->declare_parameter("tasks.stair_max_vy", 0.25);
    this->declare_parameter("tasks.stair_max_dyaw", 0.4);
    this->declare_parameter("tasks.stair_max_distance", 4.0);

    stair_duration_ = this->get_parameter("tasks.stair_duration").as_double();
    stair_target_yaw_ = this->get_parameter("tasks.stair_target_yaw").as_double();
    stair_use_current_yaw_ = this->get_parameter("tasks.stair_use_current_yaw").as_bool();
    stair_forward_speed_ = this->get_parameter("tasks.stair_forward_speed").as_double();
    stair_kp_y_ = this->get_parameter("tasks.stair_kp_y").as_double();
    stair_kp_yaw_ = this->get_parameter("tasks.stair_kp_yaw").as_double();
    stair_max_vy_ = this->get_parameter("tasks.stair_max_vy").as_double();
    stair_max_dyaw_ = this->get_parameter("tasks.stair_max_dyaw").as_double();
    stair_max_distance_ = this->get_parameter("tasks.stair_max_distance").as_double();

    this->declare_parameter("tasks.crawl_target_yaw", 0.0);
    this->declare_parameter("tasks.crawl_use_current_yaw", true);
    this->declare_parameter("tasks.crawl_forward_speed", 0.25);
    this->declare_parameter("tasks.crawl_kp_y", 1.0);
    this->declare_parameter("tasks.crawl_kp_yaw", 1.0);
    this->declare_parameter("tasks.crawl_max_vy", 0.2);
    this->declare_parameter("tasks.crawl_max_dyaw", 0.3);
    this->declare_parameter("tasks.crawl_target_dist", 1.5);
    this->declare_parameter("stair_combo.up_segment", "stairs_up");
    this->declare_parameter("stair_combo.down_segment", "stairs_down");

    crawl_target_yaw_ = this->get_parameter("tasks.crawl_target_yaw").as_double();
    crawl_use_current_yaw_ = this->get_parameter("tasks.crawl_use_current_yaw").as_bool();
    crawl_forward_speed_ = this->get_parameter("tasks.crawl_forward_speed").as_double();
    crawl_kp_y_ = this->get_parameter("tasks.crawl_kp_y").as_double();
    crawl_kp_yaw_ = this->get_parameter("tasks.crawl_kp_yaw").as_double();
    crawl_max_vy_ = this->get_parameter("tasks.crawl_max_vy").as_double();
    crawl_max_dyaw_ = this->get_parameter("tasks.crawl_max_dyaw").as_double();
    crawl_target_dist_ = this->get_parameter("tasks.crawl_target_dist").as_double();
    stair_combo_up_segment_ = this->get_parameter("stair_combo.up_segment").as_string();
    stair_combo_down_segment_ = this->get_parameter("stair_combo.down_segment").as_string();

    this->declare_parameter("visual_servoing.default_offset", std::vector<double>{0.0, 700.0, 0.0});
    this->declare_parameter("visual_servoing.tolerance_x_m", 0.06);
    this->declare_parameter("visual_servoing.tolerance_y_m", 0.06);
    this->declare_parameter("visual_servoing.tolerance_yaw_rad", 0.08);
    this->declare_parameter("visual_servoing.kp_x", 2.5);
    this->declare_parameter("visual_servoing.kp_y", 2.5);
    this->declare_parameter("visual_servoing.kp_yaw", 0.75);
    this->declare_parameter("visual_servoing.max_vx", 0.4);
    this->declare_parameter("visual_servoing.max_vy", 0.4);
    this->declare_parameter("visual_servoing.max_wz", 0.5);
    this->declare_parameter("visual_servoing.lateral_cmd_sign", 1.0);
    this->declare_parameter("visual_servoing.yaw_error_sign", 1.0);

    auto def_vec = this->get_parameter("visual_servoing.default_offset").as_double_array();
    if (def_vec.size() == 3) {
        default_offset_ = {def_vec[0], def_vec[1], def_vec[2]};
    } else {
        RCLCPP_WARN(this->get_logger(),
                    "visual_servoing.default_offset must contain exactly 3 values; falling back to [0, 700, 0].");
        default_offset_ = {0.0, 700.0, 0.0};
    }
    visual_tolerance_x_m_ = this->get_parameter("visual_servoing.tolerance_x_m").as_double();
    visual_tolerance_y_m_ = this->get_parameter("visual_servoing.tolerance_y_m").as_double();
    visual_tolerance_yaw_rad_ = this->get_parameter("visual_servoing.tolerance_yaw_rad").as_double();
    visual_kp_x_ = this->get_parameter("visual_servoing.kp_x").as_double();
    visual_kp_y_ = this->get_parameter("visual_servoing.kp_y").as_double();
    visual_kp_yaw_ = this->get_parameter("visual_servoing.kp_yaw").as_double();
    visual_max_vx_ = this->get_parameter("visual_servoing.max_vx").as_double();
    visual_max_vy_ = this->get_parameter("visual_servoing.max_vy").as_double();
    visual_max_wz_ = this->get_parameter("visual_servoing.max_wz").as_double();
    visual_lateral_cmd_sign_ = this->get_parameter("visual_servoing.lateral_cmd_sign").as_double();
    visual_yaw_error_sign_ = this->get_parameter("visual_servoing.yaw_error_sign").as_double();

    this->declare_parameter("obstacle_sequence.order", std::vector<std::string>{});
    const auto obstacle_order = this->get_parameter("obstacle_sequence.order").as_string_array();
    auto declare_if_missing = [this](const std::string& name, const auto& default_value) {
        if (!this->has_parameter(name)) {
            this->declare_parameter(name, default_value);
        }
    };
    std::set<std::string> loaded_obstacle_visual_slots;
    for (const auto& name : obstacle_order) {
        const std::string prefix = "obstacle_sequence." + name + ".";
        declare_if_missing(prefix + "action", "skip");
        declare_if_missing(prefix + "expected_task_id", -1);
        declare_if_missing(prefix + "nav_target", std::vector<double>{});
        declare_if_missing(prefix + "visual_offset", std::vector<double>{});

        const std::string action = this->get_parameter(prefix + "action").as_string();
        const bool legacy_is_obstacle = (action != "skip");
        declare_if_missing(prefix + "require_visual_servo", legacy_is_obstacle);
        if (!loaded_obstacle_visual_slots.insert(name).second) {
            continue;
        }
        const bool require_visual_servo = this->get_parameter(prefix + "require_visual_servo").as_bool();
        const int expected_task_id = this->get_parameter(prefix + "expected_task_id").as_int();
        // nav_target is declared to allow shared YAML configs, but we only use
        // expected_task_id/visual_offset here for both visual servo and QR path assist.
        const auto visual_offset = this->get_parameter(prefix + "visual_offset").as_double_array();
        if (expected_task_id <= 0 || visual_offset.size() != 3) {
            if (require_visual_servo) {
                RCLCPP_ERROR(this->get_logger(),
                             "Ignoring obstacle_sequence.%s visual config: expected_task_id > 0 and visual_offset[3] are required.",
                             name.c_str());
            }
            continue;
        }
        if (obstacle_slot_by_task_id_.count(expected_task_id) > 0) {
            RCLCPP_ERROR(this->get_logger(),
                         "Duplicate expected_task_id=%d in obstacle_sequence slots '%s' and '%s'.",
                         expected_task_id,
                         obstacle_slot_by_task_id_[expected_task_id].c_str(),
                         name.c_str());
            continue;
        }

        ObstacleVisualConfig visual_config;
        visual_config.slot_name = name;
        visual_config.expected_task_id = expected_task_id;
        visual_config.visual_offset = {visual_offset[0], visual_offset[1], visual_offset[2]};
        obstacle_visual_configs_by_slot_[name] = visual_config;
        obstacle_slot_by_task_id_[expected_task_id] = name;
    }
    RCLCPP_INFO(this->get_logger(), "Loaded %zu obstacle visual offset configs from obstacle_sequence.",
                obstacle_visual_configs_by_slot_.size());

    this->declare_parameter("tasks.pole_trajectory_file", "");
    this->declare_parameter("tasks.pole_drive_mode", "forward");
    this->declare_parameter("tasks.pole_kp_x", 0.8);
    this->declare_parameter("tasks.pole_kp_y", 0.8);
    this->declare_parameter("tasks.pole_kp_yaw", 1.0);
    this->declare_parameter("tasks.pole_kd_x", 0.0);
    this->declare_parameter("tasks.pole_kd_y", 0.0);
    this->declare_parameter("tasks.pole_kd_yaw", 0.0);
    this->declare_parameter("tasks.pole_lin_vel_creep_min", 0.0);
    this->declare_parameter("tasks.pole_lidar_offset_x", 0.0);
    this->declare_parameter("tasks.pole_lidar_offset_y", 0.0);
    this->declare_parameter("tasks.pole_max_vx", 0.35);
    this->declare_parameter("tasks.pole_max_vy", 0.35);
    this->declare_parameter("tasks.pole_max_dyaw", 0.5);
    this->declare_parameter("tasks.pole_lookahead_distance", 0.35);
    this->declare_parameter("tasks.pole_finish_tolerance", 0.25);

    pole_trajectory_file_ = this->get_parameter("tasks.pole_trajectory_file").as_string();
    pole_drive_mode_ = this->get_parameter("tasks.pole_drive_mode").as_string();
    pole_kp_x_ = this->get_parameter("tasks.pole_kp_x").as_double();
    pole_kp_y_ = this->get_parameter("tasks.pole_kp_y").as_double();
    pole_kp_yaw_ = this->get_parameter("tasks.pole_kp_yaw").as_double();
    pole_kd_x_ = this->get_parameter("tasks.pole_kd_x").as_double();
    pole_kd_y_ = this->get_parameter("tasks.pole_kd_y").as_double();
    pole_kd_yaw_ = this->get_parameter("tasks.pole_kd_yaw").as_double();
    pole_lin_vel_creep_min_ = this->get_parameter("tasks.pole_lin_vel_creep_min").as_double();
    pole_lidar_offset_x_ = this->get_parameter("tasks.pole_lidar_offset_x").as_double();
    pole_lidar_offset_y_ = this->get_parameter("tasks.pole_lidar_offset_y").as_double();
    pole_max_vx_ = this->get_parameter("tasks.pole_max_vx").as_double();
    pole_max_vy_ = this->get_parameter("tasks.pole_max_vy").as_double();
    pole_max_dyaw_ = this->get_parameter("tasks.pole_max_dyaw").as_double();
    pole_lookahead_distance_ = this->get_parameter("tasks.pole_lookahead_distance").as_double();
    pole_finish_tolerance_ = this->get_parameter("tasks.pole_finish_tolerance").as_double();

    const LineProfile default_line{
        straight_target_dist_, straight_max_speed_, straight_kp_y_,
        kp_xy_, kp_yaw_, max_vy_, max_dyaw_
    };
    for (const std::string name : {"bridge_a", "bridge_b", "stairs_up", "stairs_down"}) {
        const std::string prefix = "line_profiles." + name + ".";
        this->declare_parameter(prefix + "target_dist", default_line.target_dist);
        this->declare_parameter(prefix + "max_speed", default_line.max_speed);
        this->declare_parameter(prefix + "kp_y", default_line.kp_y);
        this->declare_parameter(prefix + "kp_xy", default_line.kp_xy);
        this->declare_parameter(prefix + "kp_yaw", default_line.kp_yaw);
        this->declare_parameter(prefix + "max_vy", default_line.max_vy);
        this->declare_parameter(prefix + "max_dyaw", default_line.max_dyaw);
        line_profiles_[name] = {
            this->get_parameter(prefix + "target_dist").as_double(),
            this->get_parameter(prefix + "max_speed").as_double(),
            this->get_parameter(prefix + "kp_y").as_double(),
            this->get_parameter(prefix + "kp_xy").as_double(),
            this->get_parameter(prefix + "kp_yaw").as_double(),
            this->get_parameter(prefix + "max_vy").as_double(),
            this->get_parameter(prefix + "max_dyaw").as_double(),
        };
    }

    const PathProfile default_path{
        pole_trajectory_file_, pole_drive_mode_, false, false, 0.5, 1.0, 0.45, 0.25, 0.35, {},
        pole_kp_x_, pole_kp_y_, pole_kp_yaw_, pole_kd_x_, pole_kd_y_, pole_kd_yaw_,
        pole_lin_vel_creep_min_,
        pole_lidar_offset_x_, pole_lidar_offset_y_, pole_max_vx_,
        pole_max_vy_, pole_max_dyaw_, pole_lookahead_distance_, pole_finish_tolerance_
    };
    std::vector<std::string> path_profile_names;
    std::set<std::string> path_profile_name_set;
    for (const auto& name : obstacle_order) {
        const std::string action_param = "obstacle_sequence." + name + ".action";
        if (!this->has_parameter(action_param)) {
            this->declare_parameter(action_param, "skip");
        }
        if (this->get_parameter(action_param).as_string() == "path" &&
            path_profile_name_set.insert(name).second) {
            path_profile_names.push_back(name);
        }
    }
    if (path_profile_names.empty()) {
        path_profile_names = {"pole"};
    }
    for (const std::string& name : path_profile_names) {
        const std::string prefix = "path_profiles." + name + ".";
        declare_if_missing(prefix + "trajectory_file", default_path.trajectory_file);
        declare_if_missing(prefix + "drive_mode", default_path.drive_mode);
        declare_if_missing(prefix + "qr_assist_enabled", default_path.qr_assist_enabled);
        declare_if_missing(prefix + "qr_assist_required", default_path.qr_assist_required);
        declare_if_missing(prefix + "qr_assist_timeout_s", default_path.qr_assist_timeout_s);
        declare_if_missing(prefix + "qr_assist_kp_y", default_path.qr_assist_kp_y);
        declare_if_missing(prefix + "qr_assist_kp_yaw", default_path.qr_assist_kp_yaw);
        declare_if_missing(prefix + "qr_assist_max_vy", default_path.qr_assist_max_vy);
        declare_if_missing(prefix + "qr_assist_max_dyaw", default_path.qr_assist_max_dyaw);
        declare_if_missing(prefix + "qr_assist_visual_offset", default_path.qr_assist_visual_offset);
        declare_if_missing(prefix + "kp_x", default_path.kp_x);
        declare_if_missing(prefix + "kp_y", default_path.kp_y);
        declare_if_missing(prefix + "kp_yaw", default_path.kp_yaw);
        declare_if_missing(prefix + "kd_x", default_path.kd_x);
        declare_if_missing(prefix + "kd_y", default_path.kd_y);
        declare_if_missing(prefix + "kd_yaw", default_path.kd_yaw);
        declare_if_missing(prefix + "lin_vel_creep_min", default_path.lin_vel_creep_min);
        declare_if_missing(prefix + "lidar_offset_x", default_path.lidar_offset_x);
        declare_if_missing(prefix + "lidar_offset_y", default_path.lidar_offset_y);
        declare_if_missing(prefix + "max_vx", default_path.max_vx);
        declare_if_missing(prefix + "max_vy", default_path.max_vy);
        declare_if_missing(prefix + "max_dyaw", default_path.max_dyaw);
        declare_if_missing(prefix + "lookahead_distance", default_path.lookahead_distance);
        declare_if_missing(prefix + "finish_tolerance", default_path.finish_tolerance);
        path_profiles_[name] = {
            this->get_parameter(prefix + "trajectory_file").as_string(),
            this->get_parameter(prefix + "drive_mode").as_string(),
            this->get_parameter(prefix + "qr_assist_enabled").as_bool(),
            this->get_parameter(prefix + "qr_assist_required").as_bool(),
            this->get_parameter(prefix + "qr_assist_timeout_s").as_double(),
            this->get_parameter(prefix + "qr_assist_kp_y").as_double(),
            this->get_parameter(prefix + "qr_assist_kp_yaw").as_double(),
            this->get_parameter(prefix + "qr_assist_max_vy").as_double(),
            this->get_parameter(prefix + "qr_assist_max_dyaw").as_double(),
            this->get_parameter(prefix + "qr_assist_visual_offset").as_double_array(),
            this->get_parameter(prefix + "kp_x").as_double(),
            this->get_parameter(prefix + "kp_y").as_double(),
            this->get_parameter(prefix + "kp_yaw").as_double(),
            this->get_parameter(prefix + "kd_x").as_double(),
            this->get_parameter(prefix + "kd_y").as_double(),
            this->get_parameter(prefix + "kd_yaw").as_double(),
            this->get_parameter(prefix + "lin_vel_creep_min").as_double(),
            this->get_parameter(prefix + "lidar_offset_x").as_double(),
            this->get_parameter(prefix + "lidar_offset_y").as_double(),
            this->get_parameter(prefix + "max_vx").as_double(),
            this->get_parameter(prefix + "max_vy").as_double(),
            this->get_parameter(prefix + "max_dyaw").as_double(),
            this->get_parameter(prefix + "lookahead_distance").as_double(),
            this->get_parameter(prefix + "finish_tolerance").as_double(),
        };
    }
}

/**
 * @brief 根据上层状态字符串切换执行器模式
 *
 * 输入：state_str（与 task_state_machine current_state 一致）
 * 输出：current_mode_ 及各任务段标志位
 * 处理：与原 if-else 链相同，仅在模式变化时重置标志
 */
void TaskExecutorNode::apply_executor_mode_from_state_string(const std::string& state_str) {
    if (!completed_state_latch_.empty()) {
        if (state_str == completed_state_latch_) {
            return;
        }
        completed_state_latch_.clear();
    }

    if (state_str == "QR_RECOGNITION") {
        if (current_mode_ != ExecutorMode::VISUAL_SERVOING) {
            current_mode_ = ExecutorMode::VISUAL_SERVOING;
            RCLCPP_INFO(this->get_logger(), "Executor Mode: VISUAL_SERVOING");
        }
    } else if (state_str == "TASK_CRAWL_MOVING") {
        if (current_mode_ != ExecutorMode::LIDAR_CRAWL) {
            current_mode_ = ExecutorMode::LIDAR_CRAWL;
            crawl_started_ = false;
            crawl_finished_ = false;
            RCLCPP_INFO(this->get_logger(), "Executor Mode: LIDAR_CRAWL");
        }
    } else if (state_str == "TASK_BRIDGE_CROSS") {
        if (current_mode_ != ExecutorMode::BRIDGE_CROSS) {
            apply_active_line_profile();
            current_mode_ = ExecutorMode::BRIDGE_CROSS;
            straight_started_ = false;
            RCLCPP_INFO(this->get_logger(), "Executor Mode: BRIDGE_CROSS");
        }
    } else if (state_str == "TASK_STAIR_MOVING") {
        if (current_mode_ != ExecutorMode::STAIR_UP_MOVING) {
            apply_active_line_profile();
            current_mode_ = ExecutorMode::STAIR_UP_MOVING;
            stair_started_ = false;
            RCLCPP_INFO(this->get_logger(), "Executor Mode: STAIR_UP_MOVING");
        }
    } else if (state_str == "TASK_POLE_AROUND") {
        if (current_mode_ != ExecutorMode::POLE_AROUND) {
            apply_active_path_profile();
            current_mode_ = ExecutorMode::POLE_AROUND;
            pole_replay_started_ = false;
            pole_replay_done_ = false;
            pole_pd_initialized_ = false;
            pole_path_progress_s_ = 0.0;
            pole_trajectory_loaded_ = load_pole_trajectory();
            RCLCPP_INFO(this->get_logger(), "Executor Mode: POLE_AROUND");
        }
    } else {
        if (current_mode_ != ExecutorMode::IDLE) {
            current_mode_ = ExecutorMode::IDLE;
            RCLCPP_INFO(this->get_logger(), "Executor Mode: IDLE (Released Control)");
        }
    }
}

/**
 * @brief 顶层状态回调入口
 *
 * 输入：current_state 话题字符串
 * 输出：委托 apply_executor_mode_from_state_string
 * 处理：解析 msg->data 后切换模式
 */
void TaskExecutorNode::state_callback(const std_msgs::msg::String::SharedPtr msg) {
    apply_executor_mode_from_state_string(msg->data);
}

/**
 * @brief 二维码检测结果回调
 *
 * 输入：QrResult 消息
 * 输出：latest_qr_data_、last_qr_time_、qr_data_received_
 * 处理：缓存最新视觉观测
 */
void TaskExecutorNode::qr_callback(const quad::msg::QrResult::SharedPtr msg) {
    latest_qr_data_ = *msg;
    last_qr_time_ = this->now();
    qr_data_received_ = true;
}

void TaskExecutorNode::sequence_index_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    active_sequence_index_ = msg->data;
}

void TaskExecutorNode::sequence_segment_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    active_sequence_segment_ = msg->data;
}

void TaskExecutorNode::sequence_slot_callback(const std_msgs::msg::String::SharedPtr msg) {
    active_sequence_slot_ = msg->data;
}

std::string TaskExecutorNode::active_line_profile_name() const {
    const std::string slot_name = active_sequence_slot_.empty() ? last_visual_slot_ : active_sequence_slot_;
    if (slot_name == "bridge_a") {
        return "bridge_a";
    }
    if (slot_name == "bridge_b") {
        return "bridge_b";
    }
    if (slot_name == "stairs") {
        return active_sequence_segment_ == 0 ? stair_combo_up_segment_ : stair_combo_down_segment_;
    }
    return slot_name.empty() ? "bridge_a" : slot_name;
}

std::string TaskExecutorNode::active_path_profile_name() const {
    const std::string slot_name = active_sequence_slot_.empty() ? last_visual_slot_ : active_sequence_slot_;
    if (!slot_name.empty() && path_profiles_.count(slot_name) > 0) {
        return slot_name;
    }
    return "pole";
}

LineProfile TaskExecutorNode::get_line_profile(const std::string& name) const {
    auto it = line_profiles_.find(name);
    if (it != line_profiles_.end()) {
        return it->second;
    }
    return {straight_target_dist_, straight_max_speed_, straight_kp_y_,
            kp_xy_, kp_yaw_, max_vy_, max_dyaw_};
}

PathProfile TaskExecutorNode::get_path_profile(const std::string& name) const {
    auto it = path_profiles_.find(name);
    if (it != path_profiles_.end()) {
        return it->second;
    }
    return {pole_trajectory_file_, pole_drive_mode_, false, false, 0.5, 1.0, 0.45, 0.25, 0.35, {},
            pole_kp_x_, pole_kp_y_, pole_kp_yaw_, pole_kd_x_, pole_kd_y_, pole_kd_yaw_,
            pole_lin_vel_creep_min_,
            pole_lidar_offset_x_, pole_lidar_offset_y_, pole_max_vx_,
            pole_max_vy_, pole_max_dyaw_, pole_lookahead_distance_, pole_finish_tolerance_};
}

void TaskExecutorNode::apply_active_line_profile() {
    const std::string profile_name = active_line_profile_name();
    const LineProfile profile = get_line_profile(profile_name);
    straight_target_dist_ = profile.target_dist;
    straight_max_speed_ = profile.max_speed;
    straight_kp_y_ = profile.kp_y;
    kp_xy_ = profile.kp_xy;
    kp_yaw_ = profile.kp_yaw;
    max_vy_ = profile.max_vy;
    max_dyaw_ = profile.max_dyaw;

    stair_forward_speed_ = profile.max_speed;
    stair_kp_y_ = profile.kp_y;
    stair_kp_yaw_ = profile.kp_yaw;
    stair_max_vy_ = profile.max_vy;
    stair_max_dyaw_ = profile.max_dyaw;
    stair_max_distance_ = profile.target_dist;

    RCLCPP_INFO(this->get_logger(), "Applied line profile '%s': dist=%.2f max_speed=%.2f",
                profile_name.c_str(), profile.target_dist, profile.max_speed);
}

void TaskExecutorNode::apply_active_path_profile() {
    const std::string profile_name = active_path_profile_name();
    const PathProfile profile = get_path_profile(profile_name);
    pole_trajectory_file_ = profile.trajectory_file;
    pole_drive_mode_ = profile.drive_mode;
    pole_qr_assist_enabled_ = profile.qr_assist_enabled;
    pole_qr_assist_required_ = profile.qr_assist_required;
    pole_qr_assist_timeout_s_ = std::max(0.05, profile.qr_assist_timeout_s);
    pole_qr_assist_kp_y_ = profile.qr_assist_kp_y;
    pole_qr_assist_kp_yaw_ = profile.qr_assist_kp_yaw;
    pole_qr_assist_max_vy_ = std::max(0.0, profile.qr_assist_max_vy);
    pole_qr_assist_max_dyaw_ = std::max(0.0, profile.qr_assist_max_dyaw);
    pole_qr_assist_visual_offset_ = profile.qr_assist_visual_offset;
    pole_kp_x_ = profile.kp_x;
    pole_kp_y_ = profile.kp_y;
    pole_kp_yaw_ = profile.kp_yaw;
    pole_kd_x_ = profile.kd_x;
    pole_kd_y_ = profile.kd_y;
    pole_kd_yaw_ = profile.kd_yaw;
    pole_lin_vel_creep_min_ = profile.lin_vel_creep_min;
    pole_lidar_offset_x_ = profile.lidar_offset_x;
    pole_lidar_offset_y_ = profile.lidar_offset_y;
    pole_max_vx_ = profile.max_vx;
    pole_max_vy_ = profile.max_vy;
    pole_max_dyaw_ = profile.max_dyaw;
    pole_lookahead_distance_ = std::max(0.0, profile.lookahead_distance);
    pole_finish_tolerance_ = std::max(0.01, profile.finish_tolerance);

    if (pole_drive_mode_ != "forward" && pole_drive_mode_ != "reverse") {
        RCLCPP_WARN(this->get_logger(),
                    "Path profile '%s' has invalid drive_mode '%s'; using forward.",
                    profile_name.c_str(), pole_drive_mode_.c_str());
        pole_drive_mode_ = "forward";
    }

    RCLCPP_INFO(this->get_logger(),
                "Applied path profile '%s': %s mode=%s lookahead=%.2f finish_tol=%.2f qr_assist=%s required=%s",
                profile_name.c_str(), profile.trajectory_file.c_str(), pole_drive_mode_.c_str(),
                pole_lookahead_distance_, pole_finish_tolerance_,
                pole_qr_assist_enabled_ ? "true" : "false",
                pole_qr_assist_required_ ? "true" : "false");
}

/**
 * @brief 从 TF 消息更新位姿（匹配 target/child 帧）
 *
 * 输入：TFMessage、target_frame_/child_frame_
 * 输出：x_/y_/yaw_、got_tf_、lidar_pose_topic_
 * 处理：遍历 transforms 取第一个匹配项，平移直接使用，四元数只转换 yaw 后发布
 */
void TaskExecutorNode::update_pose_from_tf(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
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

            geometry_msgs::msg::Point pose_msg;
            pose_msg.x = x_;
            pose_msg.y = y_;
            pose_msg.z = yaw_;
            lidar_pose_pub_->publish(pose_msg);
            break;
        }
    }
}

/**
 * @brief TF 回调
 *
 * 输入：TF 消息
 * 输出：更新并发布通用雷达位姿；任务模式可复用 x_/y_/yaw_
 * 处理：只做平移读取和四元数到 yaw 的转换，不做坐标变换
 */
void TaskExecutorNode::tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    update_pose_from_tf(msg);
}

/**
 * @brief 定时控制循环：按 ExecutorMode 分发任务算法
 *
 * 输入：current_mode_
 * 输出：各 execute_* 发布的 cmd_vel / task_done
 * 处理：switch 分发，IDLE 不动作
 */
void TaskExecutorNode::control_loop() {
    switch (current_mode_) {
        case ExecutorMode::VISUAL_SERVOING:
            execute_visual_servoing();
            break;
        case ExecutorMode::LIDAR_CRAWL:
            execute_lidar_crawl();
            break;
        case ExecutorMode::BRIDGE_CROSS:
            execute_bridge_cross();
            break;
        case ExecutorMode::STAIR_UP_MOVING:
            execute_stair_up();
            break;
        case ExecutorMode::POLE_AROUND:
            execute_pole_around();
            break;
        case ExecutorMode::IDLE:
        default:
            break;
    }
}

/**
 * @brief 判断 QR 数据是否超时
 *
 * 输入：last_qr_time_、DATA_TIMEOUT_THRESHOLD
 * 输出：是否应停车并停止伺服
 * 处理：与当前时间差超过阈值则为 stale
 */
bool TaskExecutorNode::is_qr_data_stale() {
    return (this->now() - last_qr_time_).seconds() > DATA_TIMEOUT_THRESHOLD;
}

/**
 * @brief 获取当前视觉伺服目标配置
 *
 * 输入：task_id、active_sequence_slot_、obstacle_sequence 表
 * 输出：当前槽位或 task_id 对应的视觉配置
 * 处理：有当前槽位时先按槽位校验 ID；无槽位时按 task_id 反查槽位
 */
bool TaskExecutorNode::get_active_visual_config(int task_id, ObstacleVisualConfig& config) {
    if (!active_sequence_slot_.empty() && active_sequence_slot_ != "DONE") {
        auto slot_it = obstacle_visual_configs_by_slot_.find(active_sequence_slot_);
        if (slot_it == obstacle_visual_configs_by_slot_.end()) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                  "No visual config for current obstacle slot '%s'. Stop visual servoing.",
                                  active_sequence_slot_.c_str());
            return false;
        }
        if (slot_it->second.expected_task_id != task_id) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                  "QR task mismatch before servo: slot='%s' expected=%d got=%d. Stop.",
                                  active_sequence_slot_.c_str(), slot_it->second.expected_task_id, task_id);
            return false;
        }
        config = slot_it->second;
        return true;
    }

    auto task_it = obstacle_slot_by_task_id_.find(task_id);
    if (task_it == obstacle_slot_by_task_id_.end()) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "No obstacle visual config for QR task_id=%d. Stop visual servoing.",
                              task_id);
        return false;
    }
    auto slot_it = obstacle_visual_configs_by_slot_.find(task_it->second);
    if (slot_it == obstacle_visual_configs_by_slot_.end()) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "Obstacle visual config table is inconsistent for task_id=%d slot='%s'.",
                              task_id, task_it->second.c_str());
        return false;
    }
    config = slot_it->second;
    return true;
}

/**
 * @brief QR 视觉系误差转换到机体系
 *
 * 输入：config、latest_qr_data_、visual_yaw_error_sign_
 * 输出：VisualBodyErrors
 * 处理：mm/deg 差分后按 body.x=qr.y, body.y=-qr.x 变换
 */
VisualBodyErrors TaskExecutorNode::compute_visual_body_errors(const TaskOffset& config) const {
    const double err_qr_x_mm = latest_qr_data_.x - config.target_qr_x;
    const double err_qr_y_mm = latest_qr_data_.y - config.target_qr_y;
    const double err_qr_yaw_deg = latest_qr_data_.yaw - config.target_qr_yaw;
    VisualBodyErrors errors{};
    errors.err_x_m = err_qr_y_mm / 1000.0;
    errors.err_y_m = -err_qr_x_mm / 1000.0;
    errors.err_yaw_rad = visual_yaw_error_sign_ * err_qr_yaw_deg * (M_PI / 180.0);
    errors.err_qr_yaw_deg = err_qr_yaw_deg;
    return errors;
}

/**
 * @brief 发布视觉伺服调试误差
 *
 * 输入：机体系误差
 * 输出：visual_servoing/debug_errors
 * 处理：填入 Point.x/y/z 发布
 */
void TaskExecutorNode::publish_visual_debug_errors(const VisualBodyErrors& errors) const {
    geometry_msgs::msg::Point debug_msg;
    debug_msg.x = errors.err_x_m;
    debug_msg.y = errors.err_y_m;
    debug_msg.z = errors.err_yaw_rad;
    error_debug_pub_->publish(debug_msg);
}

/**
 * @brief 判断三轴视觉对准是否完成
 *
 * 输入：errors、visual_tolerance_* 
 * 输出：是否全部在容差内
 * 处理：分别比较 |err| 与 tolerance
 */
bool TaskExecutorNode::is_visual_aligned(const VisualBodyErrors& errors) const {
    return std::abs(errors.err_x_m) < visual_tolerance_x_m_ &&
           std::abs(errors.err_y_m) < visual_tolerance_y_m_ &&
           std::abs(errors.err_yaw_rad) < visual_tolerance_yaw_rad_;
}

/**
 * @brief 发布视觉 P 控制速度（分轴到位则该轴速度为 0）
 *
 * 输入：errors、config、task_id
 * 输出：cmd_vel
 * 处理：clamp 后 publish，并 throttle 日志
 */
void TaskExecutorNode::publish_visual_velocity_cmd(
    const VisualBodyErrors& errors,
    const TaskOffset& config,
    int task_id) {
    const bool x_aligned = std::abs(errors.err_x_m) < visual_tolerance_x_m_;
    const bool y_aligned = std::abs(errors.err_y_m) < visual_tolerance_y_m_;
    const bool yaw_aligned = std::abs(errors.err_yaw_rad) < visual_tolerance_yaw_rad_;

    geometry_msgs::msg::Twist vel;
    vel.linear.x = x_aligned ? 0.0 : std::clamp(visual_kp_x_ * errors.err_x_m, -visual_max_vx_, visual_max_vx_);
    vel.linear.y = y_aligned ? 0.0 : std::clamp(
        visual_lateral_cmd_sign_ * visual_kp_y_ * errors.err_y_m,
        -visual_max_vy_, visual_max_vy_);
    vel.angular.z = yaw_aligned ? 0.0 : std::clamp(visual_kp_yaw_ * errors.err_yaw_rad, -visual_max_wz_, visual_max_wz_);
    cmd_vel_pub_->publish(vel);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "VISUAL_SERVO: task=%d target_qr=(%.1f, %.1f, %.2f deg) current_qr=(%.1f, %.1f, %.2f deg) "
        "qr_err=(%.1f, %.1f, %.2f deg) body_err=(%.3f, %.3f, %.3f rad yaw_sign=%.1f lateral_cmd_sign=%.1f) cmd=(%.2f, %.2f, %.2f)",
        task_id,
        config.target_qr_x, config.target_qr_y, config.target_qr_yaw,
        latest_qr_data_.x, latest_qr_data_.y, latest_qr_data_.yaw,
        latest_qr_data_.x - config.target_qr_x,
        latest_qr_data_.y - config.target_qr_y,
        latest_qr_data_.yaw - config.target_qr_yaw,
        errors.err_x_m, errors.err_y_m, errors.err_yaw_rad, visual_yaw_error_sign_, visual_lateral_cmd_sign_,
        vel.linear.x, vel.linear.y, vel.angular.z);
}

/**
 * @brief 视觉闭环伺服对准主流程
 *
 * 输入：qr 数据、容差与增益参数
 * 输出：cmd_vel 或 task_done(task_id)
 * 处理：超时停车→算误差→对齐则 finish_task→否则发速度
 */
void TaskExecutorNode::execute_visual_servoing() {
    if (!qr_data_received_) {
        return;
    }

    if (is_qr_data_stale()) {
        stop_robot();
        return;
    }

    const int task_id = latest_qr_data_.task_type;
    ObstacleVisualConfig visual_config;
    if (!get_active_visual_config(task_id, visual_config)) {
        stop_robot();
        return;
    }
    last_visual_slot_ = visual_config.slot_name;
    const TaskOffset& config = visual_config.visual_offset;
    const VisualBodyErrors errors = compute_visual_body_errors(config);

    publish_visual_debug_errors(errors);

    if (is_visual_aligned(errors)) {
        stop_robot();
        RCLCPP_INFO(this->get_logger(),
                    "Visual Servoing Aligned. Task ID: %d target_qr=(%.1f, %.1f, %.2f deg) "
                    "current_qr=(%.1f, %.1f, %.2f deg)",
                    task_id,
                    config.target_qr_x, config.target_qr_y, config.target_qr_yaw,
                    latest_qr_data_.x, latest_qr_data_.y, latest_qr_data_.yaw);
        finish_task(task_id);
        return;
    }

    publish_visual_velocity_cmd(errors, config, task_id);
}

/**
 * @brief 记录匍匐段起始状态
 *
 * 输入：x_/y_/yaw_
 * 输出：crawl_start_*、crawl_active_target_yaw_、crawl_started_
 * 处理：首次进入时记录
 */
void TaskExecutorNode::ensure_crawl_segment_started() {
    if (crawl_started_) {
        return;
    }
    crawl_start_x_ = x_;
    crawl_start_y_ = y_;
    crawl_active_target_yaw_ = crawl_use_current_yaw_ ? yaw_ : crawl_target_yaw_;
    crawl_started_ = true;
    RCLCPP_INFO(this->get_logger(),
                "Lidar Crawl Started at (%.2f, %.2f), yaw current %.2f, target %.2f%s",
                x_, y_, yaw_, crawl_active_target_yaw_,
                crawl_use_current_yaw_ ? " (current yaw)" : "");
}

/**
 * @brief 计算匍匐前进与横向误差
 *
 * 输入：位姿、crawl_active_target_yaw_
 * 输出：CrawlProgress
 * 处理：世界系位移投影到目标直线系
 */
CrawlProgress TaskExecutorNode::compute_crawl_progress() const {
    CrawlProgress progress{};
    const BodyDelta body_delta =
        compute_body_delta_from_start(crawl_start_x_, crawl_start_y_, crawl_active_target_yaw_);
    progress.forward_progress = body_delta.forward;
    progress.lateral_error = body_delta.lateral;
    progress.err_yaw = normalize_angle(crawl_active_target_yaw_ - yaw_);
    return progress;
}

/**
 * @brief 判断匍匐段是否应结束
 *
 * 输入：progress、crawl_target_dist_
 * 输出：是否结束
 * 处理：按前进距离完成，yaw 偏差过大时不完成
 */
bool TaskExecutorNode::is_crawl_finished(const CrawlProgress& progress) const {
    if (std::abs(progress.err_yaw) > 0.25) {
        return false;
    }
    return crawl_target_dist_ > 0.0 && progress.forward_progress >= crawl_target_dist_;
}

/**
 * @brief 计算匍匐雷达闭环 cmd_vel
 *
 * 输入：progress
 * 输出：Twist
 * 处理：|err_yaw|>0.25 只转；否则定速前进+横向纠偏
 */
geometry_msgs::msg::Twist TaskExecutorNode::compute_crawl_twist(const CrawlProgress& progress) const {
    geometry_msgs::msg::Twist cmd;
    if (std::abs(progress.err_yaw) > 0.25) {
        cmd.angular.z = std::clamp(crawl_kp_yaw_ * progress.err_yaw, -crawl_max_dyaw_, crawl_max_dyaw_);
    } else {
        cmd.linear.x = crawl_forward_speed_;
        cmd.linear.y = std::clamp(crawl_kp_y_ * (-progress.lateral_error), -crawl_max_vy_, crawl_max_vy_);
        cmd.angular.z = std::clamp(crawl_kp_yaw_ * progress.err_yaw, -crawl_max_dyaw_, crawl_max_dyaw_);
    }
    return cmd;
}

/**
 * @brief 匍匐雷达闭环直线段
 *
 * 输入：TF、匍匐闭环参数
 * 输出：cmd_vel 或 task_done(-1)
 * 处理：等 TF→记起点→算进度→结束或发速
 */
void TaskExecutorNode::execute_lidar_crawl() {
    if (crawl_finished_) {
        return;
    }
    if (!got_tf_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "LidarCrawl: Waiting for TF...");
        return;
    }
    if (stop_on_lidar_drift("LidarCrawl")) {
        return;
    }

    ensure_crawl_segment_started();
    const CrawlProgress progress = compute_crawl_progress();

    if (is_crawl_finished(progress)) {
        stop_robot();
        crawl_finished_ = true;
        RCLCPP_INFO(this->get_logger(), "Lidar Crawl Finished. progress=%.2f/%.2f",
                    progress.forward_progress, crawl_target_dist_);
        finish_task(-1);
        return;
    }

    const geometry_msgs::msg::Twist cmd = compute_crawl_twist(progress);
    cmd_vel_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "LIDAR_CRAWL: start_yaw=%.2f forward_body=%.2f/%.2f lateral_body=%.2f err_yaw=%.2f "
        "cmd=(%.2f, %.2f, %.2f)",
        crawl_active_target_yaw_, progress.forward_progress, crawl_target_dist_,
        progress.lateral_error, progress.err_yaw,
        cmd.linear.x, cmd.linear.y, cmd.angular.z);
}

/**
 * @brief 记录过桥起始位姿（首次进入时）
 *
 * 输入：x_/y_/yaw_
 * 输出：straight_start_*、straight_started_
 * 处理：仅 straight_started_ 为 false 时记录
 */
void TaskExecutorNode::ensure_bridge_segment_started() {
    if (straight_started_) {
        return;
    }
    straight_start_x_ = x_;
    straight_start_y_ = y_;
    straight_start_yaw_ = yaw_;
    straight_started_ = true;
    RCLCPP_INFO(this->get_logger(), "Bridge Cross Started at (%.2f, %.2f), locked yaw=%.2f", x_, y_, yaw_);
}

/**
 * @brief 计算过桥理想直线系进度与剩余距离
 *
 * 输入：位姿与锁存起始 yaw、straight_target_dist_
 * 输出：BridgeProgressErrors
 * 处理：map 位移投影到理想雷达系
 */
BridgeProgressErrors TaskExecutorNode::compute_bridge_progress_errors() const {
    const BodyDelta body_delta =
        compute_body_delta_from_start(straight_start_x_, straight_start_y_, straight_start_yaw_);

    BridgeProgressErrors progress{};
    progress.dx_ideal_lidar = body_delta.forward;
    progress.dy_ideal_lidar = body_delta.lateral;
    progress.remain_dist = straight_target_dist_ - progress.dx_ideal_lidar;
    progress.err_yaw = normalize_angle(straight_start_yaw_ - yaw_);
    return progress;
}

/**
 * @brief 判断过桥是否完成
 *
 * 输入：remain_dist、err_yaw
 * 输出：是否完成
 * 处理：remain_dist<0.05 且 |err_yaw|<0.1
 */
bool TaskExecutorNode::is_bridge_finished(const BridgeProgressErrors& progress) const {
    return progress.remain_dist < 0.05 && std::abs(progress.err_yaw) < 0.1;
}

/**
 * @brief 根据过桥误差计算 cmd_vel
 *
 * 输入：progress、PID/限速参数
 * 输出：Twist（先对航向再对位置）
 * 处理：|err_yaw|>0.2 只转；否则 vx/vy/wz
 */
geometry_msgs::msg::Twist TaskExecutorNode::compute_bridge_twist(const BridgeProgressErrors& progress) const {
    geometry_msgs::msg::Twist cmd;
    if (std::abs(progress.err_yaw) > 0.2) {
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = std::max(-max_dyaw_, std::min(kp_yaw_ * progress.err_yaw, max_dyaw_));
    } else {
        cmd.linear.x = std::max(-straight_max_speed_, std::min(kp_xy_ * progress.remain_dist, straight_max_speed_));
        cmd.linear.y = std::max(-max_vy_, std::min(straight_kp_y_ * (-progress.dy_ideal_lidar), max_vy_));
        cmd.angular.z = std::max(-max_dyaw_, std::min(kp_yaw_ * progress.err_yaw, max_dyaw_));
    }
    return cmd;
}

/**
 * @brief 雷达辅助直线过桥
 *
 * 输入：TF、直线目标参数
 * 输出：cmd_vel 或 task_done(-1)
 * 处理：等 TF→记起点→算误差→完成或发速
 */
void TaskExecutorNode::execute_bridge_cross() {
    if (!got_tf_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "BridgeCross: Waiting for TF...");
        return;
    }
    if (stop_on_lidar_drift("BridgeCross")) {
        return;
    }

    ensure_bridge_segment_started();
    const BridgeProgressErrors progress = compute_bridge_progress_errors();

    if (is_bridge_finished(progress)) {
        stop_robot();
        RCLCPP_INFO(this->get_logger(), "Bridge Cross Finished.");
        finish_task(-1);
        return;
    }

    const geometry_msgs::msg::Twist cmd = compute_bridge_twist(progress);
    cmd_vel_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "BRIDGE CROSS: start_yaw=%.2f forward_body=%.2f/%.2f remain=%.2f lateral_body=%.2f "
        "err_yaw=%.2f cmd=(%.2f, %.2f, %.2f)",
        straight_start_yaw_, progress.dx_ideal_lidar, straight_target_dist_,
        progress.remain_dist, progress.dy_ideal_lidar, progress.err_yaw,
        cmd.linear.x, cmd.linear.y, cmd.angular.z);
}

/**
 * @brief 记录上楼梯段起始状态
 *
 * 输入：x_/y_
 * 输出：stair_start_*、stair_started_、stair_start_time_
 * 处理：首次进入时记录
 */
void TaskExecutorNode::ensure_stair_segment_started() {
    if (stair_started_) {
        return;
    }
    stair_start_time_ = this->now();
    stair_start_x_ = x_;
    stair_start_y_ = y_;
    stair_active_target_yaw_ = stair_use_current_yaw_ ? yaw_ : stair_target_yaw_;
    stair_started_ = true;
    RCLCPP_INFO(this->get_logger(),
                "Stair Up Started at (%.2f, %.2f), yaw current %.2f, target %.2f%s",
                x_, y_, yaw_, stair_active_target_yaw_,
                stair_use_current_yaw_ ? " (current yaw)" : "");
}

/**
 * @brief 计算上楼梯前进与横向误差
 *
 * 输入：位姿、stair_active_target_yaw_
 * 输出：StairProgress
 * 处理：世界系位移投影到目标直线系
 */
StairProgress TaskExecutorNode::compute_stair_progress() const {
    StairProgress progress{};
    progress.elapsed = (this->now() - stair_start_time_).seconds();
    const BodyDelta body_delta =
        compute_body_delta_from_start(stair_start_x_, stair_start_y_, stair_active_target_yaw_);
    progress.forward_progress = body_delta.forward;
    progress.lateral_error = body_delta.lateral;
    progress.err_yaw = normalize_angle(stair_active_target_yaw_ - yaw_);
    return progress;
}

/**
 * @brief 判断上楼梯段是否应结束
 *
 * 输入：progress、stair_max_distance_
 * 输出：是否结束
 * 处理：只按前进距离完成；yaw 未对齐时不完成，避免原地转误判成功
 */
bool TaskExecutorNode::is_stair_finished(const StairProgress& progress) const {
    if (std::abs(progress.err_yaw) > 0.25) {
        return false;
    }
    return stair_max_distance_ > 0.0 && progress.forward_progress >= stair_max_distance_;
}

/**
 * @brief 计算上楼梯 cmd_vel
 *
 * 输入：progress
 * 输出：Twist
 * 处理：|err_yaw|>0.25 只转；否则定速前进+横向纠偏
 */
geometry_msgs::msg::Twist TaskExecutorNode::compute_stair_twist(const StairProgress& progress) const {
    geometry_msgs::msg::Twist cmd;
    if (std::abs(progress.err_yaw) > 0.25) {
        cmd.angular.z = std::clamp(stair_kp_yaw_ * progress.err_yaw, -stair_max_dyaw_, stair_max_dyaw_);
    } else {
        cmd.linear.x = stair_forward_speed_;
        cmd.linear.y = std::clamp(stair_kp_y_ * (-progress.lateral_error), -stair_max_vy_, stair_max_vy_);
        cmd.angular.z = std::clamp(stair_kp_yaw_ * progress.err_yaw, -stair_max_dyaw_, stair_max_dyaw_);
    }
    return cmd;
}

/**
 * @brief 雷达辅助上楼梯闭环
 *
 * 输入：TF、楼梯参数
 * 输出：cmd_vel 或 task_done(-1)
 * 处理：等 TF→记起点→算进度→结束或发速
 */
void TaskExecutorNode::execute_stair_up() {
    if (!got_tf_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "StairUp: Waiting for TF...");
        return;
    }
    if (stop_on_lidar_drift("StairUp")) {
        return;
    }

    ensure_stair_segment_started();
    const StairProgress progress = compute_stair_progress();

    if (is_stair_finished(progress)) {
        stop_robot();
        RCLCPP_INFO(this->get_logger(), "Stair Up Finished. progress=%.2f/%.2f, elapsed=%.2f",
                    progress.forward_progress, stair_max_distance_, progress.elapsed);
        finish_task(-1);
        return;
    }

    const geometry_msgs::msg::Twist cmd = compute_stair_twist(progress);
    cmd_vel_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "STAIR_UP: t=%.2f start_yaw=%.2f forward_body=%.2f/%.2f lateral_body=%.2f err_yaw=%.2f "
        "cmd=(%.2f, %.2f, %.2f)",
        progress.elapsed, stair_active_target_yaw_, progress.forward_progress, stair_max_distance_,
        progress.lateral_error, progress.err_yaw,
        cmd.linear.x, cmd.linear.y, cmd.angular.z);
}

/**
 * @brief 启动绕杆轨迹前瞻跟踪
 *
 * 输入：pole_trajectory_
 * 输出：pole_replay_started_、pole_path_progress_s_ 等
 * 处理：首次进入时重置 PD/路径进度并打日志
 */
void TaskExecutorNode::ensure_pole_replay_started() {
    if (pole_replay_started_) {
        return;
    }
    pole_replay_t0_ = this->now();
    pole_replay_started_ = true;
    pole_replay_done_ = false;
    pole_pd_initialized_ = false;
    pole_path_progress_s_ = 0.0;
    RCLCPP_INFO(this->get_logger(),
                "PoleAround: lookahead tracking started, length %.3f m, %zu points, mode=%s.",
                pole_trajectory_.back().s, pole_trajectory_.size(), pole_drive_mode_.c_str());
}

/**
 * @brief 计算绕杆参考点与机体的 map→body 误差
 *
 * 输入：参考位姿 ref、当前 x_/y_/yaw_、pole_lidar_offset_*
 * 输出：PoleBodyErrors
 * 处理：与 lidar_nav_auto_parku 一致的坐标变换
 */
PoleBodyErrors TaskExecutorNode::compute_pole_body_errors(double ref_x, double ref_y, double ref_yaw) const {
    const double err_x_map = ref_x - x_;
    const double err_y_map = ref_y - y_;
    const double err_yaw = normalize_angle(ref_yaw - yaw_);
    const double cos_yaw = std::cos(yaw_);
    const double sin_yaw = std::sin(yaw_);
    const double err_x_lidar = cos_yaw * err_y_map - sin_yaw * err_x_map;
    const double err_y_lidar = -sin_yaw * err_y_map - cos_yaw * err_x_map;

    PoleBodyErrors errors{};
    errors.err_x_body = err_x_lidar + pole_lidar_offset_x_;
    errors.err_y_body = err_y_lidar - pole_lidar_offset_y_;
    errors.err_yaw = err_yaw;
    return errors;
}

/**
 * @brief 绕杆 PD 速度控制（含 creep 最小线速度）
 *
 * 输入：PoleBodyErrors
 * 输出：Twist；更新 pole PD 历史
 * 处理：微分限幅 dt，creep 后 clamp 发布
 */
geometry_msgs::msg::Twist TaskExecutorNode::compute_pole_pd_twist(const PoleBodyErrors& errors) {
    const auto ctrl_now = this->now();
    if (!pole_pd_initialized_) {
        pole_prev_err_x_body_ = errors.err_x_body;
        pole_prev_err_y_body_ = errors.err_y_body;
        pole_prev_err_yaw_ = errors.err_yaw;
        pole_prev_ctrl_time_ = ctrl_now;
        pole_pd_initialized_ = true;
    }
    double dt = (ctrl_now - pole_prev_ctrl_time_).seconds();
    dt = std::max(1e-4, std::min(dt, 0.25));

    const double dex_dt = (errors.err_x_body - pole_prev_err_x_body_) / dt;
    const double dey_dt = (errors.err_y_body - pole_prev_err_y_body_) / dt;
    const double deyaw_dt = normalize_angle(errors.err_yaw - pole_prev_err_yaw_) / dt;

    pole_prev_err_x_body_ = errors.err_x_body;
    pole_prev_err_y_body_ = errors.err_y_body;
    pole_prev_err_yaw_ = errors.err_yaw;
    pole_prev_ctrl_time_ = ctrl_now;

    double vx = pole_kp_x_ * errors.err_x_body + pole_kd_x_ * dex_dt;
    double vy = pole_kp_y_ * errors.err_y_body + pole_kd_y_ * dey_dt;
    if (pole_lin_vel_creep_min_ > 0.0) {
        constexpr double eps = 5e-3;
        if (std::abs(errors.err_x_body) > eps && std::abs(vx) < pole_lin_vel_creep_min_) {
            vx = std::copysign(pole_lin_vel_creep_min_, errors.err_x_body);
        }
        if (std::abs(errors.err_y_body) > eps && std::abs(vy) < pole_lin_vel_creep_min_) {
            vy = std::copysign(pole_lin_vel_creep_min_, errors.err_y_body);
        }
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = std::clamp(vx, -pole_max_vx_, pole_max_vx_);
    cmd.linear.y = std::clamp(vy, -pole_max_vy_, pole_max_vy_);
    cmd.angular.z = std::clamp(
        pole_kp_yaw_ * errors.err_yaw + pole_kd_yaw_ * deyaw_dt,
        -pole_max_dyaw_, pole_max_dyaw_);
    return cmd;
}

bool TaskExecutorNode::apply_qr_path_assist(
    geometry_msgs::msg::Twist& cmd,
    VisualBodyErrors& qr_errors) {
    qr_errors = {};
    if (!pole_qr_assist_enabled_) {
        return true;
    }

    const auto stop_or_continue = [this](const char* reason) {
        if (!pole_qr_assist_required_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "QR path assist unavailable (%s); falling back to path tracking.",
                                 reason);
            return true;
        }
        stop_robot();
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "QR path assist required but unavailable (%s); waiting for QR.",
                             reason);
        return false;
    };

    const std::string slot_name = active_sequence_slot_.empty() ? last_visual_slot_ : active_sequence_slot_;
    if (slot_name.empty() || slot_name == "DONE") {
        return stop_or_continue("no active obstacle slot");
    }

    auto config_it = obstacle_visual_configs_by_slot_.find(slot_name);
    if (config_it == obstacle_visual_configs_by_slot_.end()) {
        return stop_or_continue("no visual config for active slot");
    }

    if (!qr_data_received_) {
        return stop_or_continue("no QR data");
    }

    const double qr_age_s = (this->now() - last_qr_time_).seconds();
    if (qr_age_s > pole_qr_assist_timeout_s_) {
        return stop_or_continue("QR timeout");
    }

    const ObstacleVisualConfig& config = config_it->second;
    if (latest_qr_data_.task_type != config.expected_task_id) {
        return stop_or_continue("QR task id mismatch");
    }

    TaskOffset qr_target = config.visual_offset;
    if (pole_qr_assist_visual_offset_.size() == 3) {
        qr_target = {
            pole_qr_assist_visual_offset_[0],
            pole_qr_assist_visual_offset_[1],
            pole_qr_assist_visual_offset_[2],
        };
    } else if (!pole_qr_assist_visual_offset_.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "QR path assist visual offset must have 3 values; using obstacle slot visual_offset.");
    }

    qr_errors = compute_visual_body_errors(qr_target);
    publish_visual_debug_errors(qr_errors);

    cmd.linear.y = std::clamp(
        pole_qr_assist_kp_y_ * qr_errors.err_y_m,
        -pole_qr_assist_max_vy_, pole_qr_assist_max_vy_);
    cmd.angular.z = std::clamp(
        pole_qr_assist_kp_yaw_ * qr_errors.err_yaw_rad,
        -pole_qr_assist_max_dyaw_, pole_qr_assist_max_dyaw_);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "QR_PATH_ASSIST: slot=%s task=%d qr_err_body_m_rad=(%.3f, %.3f, %.3f) cmd_y_yaw=(%.2f, %.2f)",
        slot_name.c_str(), latest_qr_data_.task_type,
        qr_errors.err_x_m, qr_errors.err_y_m, qr_errors.err_yaw_rad,
        cmd.linear.y, cmd.angular.z);
    return true;
}

/**
 * @brief 判断绕杆前瞻跟踪是否结束
 *
 * 输入：closest_s、终点距离
 * 输出：是否应结束；结束时置 pole_replay_done_ 并 finish_task
 * 处理：进度到末段且靠近终点时停车、打日志、finish_task(-1)
 */
bool TaskExecutorNode::check_pole_replay_finished(double closest_s) {
    const double total_s = pole_trajectory_.empty() ? 0.0 : pole_trajectory_.back().s;
    const double finish_gate_s = std::max(0.0, total_s - pole_finish_tolerance_);
    const double end_dist = pole_trajectory_.empty()
        ? std::numeric_limits<double>::infinity()
        : std::hypot(x_ - pole_trajectory_.back().x, y_ - pole_trajectory_.back().y);

    if (closest_s < finish_gate_s || end_dist > pole_finish_tolerance_) {
        return false;
    }
    pole_replay_done_ = true;
    stop_robot();
    RCLCPP_INFO(this->get_logger(),
                "Pole Around Finished. progress %.3f/%.3f m, end_dist=%.3f m.",
                closest_s, total_s, end_dist);
    finish_task(-1);
    return true;
}

/**
 * @brief 绕杆轨迹前瞻点跟踪与闭环纠偏
 *
 * 输入：pole_trajectory_、TF、PD 参数
 * 输出：cmd_vel、target 调试、task_done(-1)
 * 处理：最近投影点→弧长前瞻参考→算 body 误差→PD→检查终点距离
 */
void TaskExecutorNode::execute_pole_around() {
    if (!got_tf_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "PoleAround: Waiting for TF...");
        return;
    }
    if (stop_on_lidar_drift("PoleAround")) {
        return;
    }
    if (!pole_trajectory_loaded_ || pole_trajectory_.empty()) {
        stop_robot();
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "PoleAround: no valid trajectory loaded.");
        return;
    }

    ensure_pole_replay_started();

    if (pole_replay_done_) {
        stop_robot();
        finish_task(-1);
        return;
    }

    double ref_x = 0.0;
    double ref_y = 0.0;
    double ref_yaw = 0.0;
    double closest_s = 0.0;
    double target_s = 0.0;
    if (!sample_pole_reference(ref_x, ref_y, ref_yaw, closest_s, target_s)) {
        return;
    }

    geometry_msgs::msg::Point target_msg;
    target_msg.x = ref_x;
    target_msg.y = ref_y;
    target_msg.z = ref_yaw;
    target_pub_->publish(target_msg);

    const PoleBodyErrors errors = compute_pole_body_errors(ref_x, ref_y, ref_yaw);
    geometry_msgs::msg::Twist cmd = compute_pole_pd_twist(errors);
    VisualBodyErrors qr_errors{};
    if (!apply_qr_path_assist(cmd, qr_errors)) {
        return;
    }
    cmd_vel_pub_->publish(cmd);

    if (check_pole_replay_finished(closest_s)) {
        return;
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "POLE_LOOKAHEAD: s=%.2f target=%.2f/%.2f mode=%s pos(%.2f,%.2f,%.2f) ref(%.2f,%.2f,%.2f) err_body(%.2f,%.2f,%.2f) cmd(%.2f,%.2f,%.2f)",
        closest_s, target_s, pole_trajectory_.back().s, pole_drive_mode_.c_str(),
        x_, y_, yaw_, ref_x, ref_y, ref_yaw,
        errors.err_x_body, errors.err_y_body, errors.err_yaw, cmd.linear.x, cmd.linear.y, cmd.angular.z);
}

/**
 * @brief 雷达位姿越界时停车并告警
 *
 * 输入：context（日志前缀）、x_/y_
 * 输出：true 表示已越界并已 stop_robot
 * 处理：x∉[-5,5] 或 y∉[0,10] 时 throttle 提示「雷达飘了」
 */
bool TaskExecutorNode::stop_on_lidar_drift(const char* context) {
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
 * @brief 将雷达全局位移投影到任务开始时锁定的机体系
 *
 * 输入：任务开始时的雷达位姿 start_x/start_y/start_yaw 与当前 x_/y_
 * 输出：机体系前向和横向位移，forward 对应 cmd_vel.linear.x 正方向
 * 处理：坐标约定与 obstacle_nav_node 的雷达系到机体系误差变换一致
 */
BodyDelta TaskExecutorNode::compute_body_delta_from_start(
    double start_x,
    double start_y,
    double start_yaw) const {
    const double dx_world = x_ - start_x;
    const double dy_world = y_ - start_y;
    const double cos_start = std::cos(start_yaw);
    const double sin_start = std::sin(start_yaw);

    BodyDelta delta{};
    delta.forward = cos_start * dy_world - sin_start * dx_world;
    delta.lateral = -cos_start * dx_world - sin_start * dy_world;
    return delta;
}

std::string TaskExecutorNode::executor_mode_state_name() const {
    switch (current_mode_) {
        case ExecutorMode::VISUAL_SERVOING:
            return "QR_RECOGNITION";
        case ExecutorMode::LIDAR_CRAWL:
            return "TASK_CRAWL_MOVING";
        case ExecutorMode::BRIDGE_CROSS:
            return "TASK_BRIDGE_CROSS";
        case ExecutorMode::STAIR_UP_MOVING:
            return "TASK_STAIR_MOVING";
        case ExecutorMode::POLE_AROUND:
            return "TASK_POLE_AROUND";
        case ExecutorMode::IDLE:
        default:
            return "";
    }
}

/**
 * @brief 规范化角度到 [-pi, pi]
 *
 * 输入：angle（弧度）
 * 输出：归一化角度
 * 处理：循环加减 2*pi
 */
double TaskExecutorNode::normalize_angle(double angle) const {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

/**
 * @brief 发布零速度停车
 *
 * 输入：cmd_vel_pub_
 * 输出：零 Twist
 * 处理：publish 默认构造 Twist
 */
void TaskExecutorNode::stop_robot() {
    geometry_msgs::msg::Twist cmd;
    cmd_vel_pub_->publish(cmd);
}

/**
 * @brief 发布任务完成并进入 IDLE 模式
 *
 * 输入：task_id（QR 为正 ID，常规完成为 -1）
 * 输出：task_done 话题；current_mode_=IDLE
 * 处理：publish Int32 后切换执行器模式
 */
void TaskExecutorNode::finish_task(int task_id) {
    completed_state_latch_ = executor_mode_state_name();
    std_msgs::msg::Int32 done_msg;
    done_msg.data = task_id;
    task_done_pub_->publish(done_msg);
    current_mode_ = ExecutorMode::IDLE;
}

/**
 * @brief 从文本文件加载绕杆录制轨迹
 *
 * 输入：pole_trajectory_file_
 * 输出：pole_trajectory_；返回值是否成功
 * 处理：解析 time x y yaw，校验单调时间，归零 t0 并预计算弧长 s
 */
bool TaskExecutorNode::load_pole_trajectory() {
    pole_trajectory_.clear();

    if (pole_trajectory_file_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Pole trajectory file is empty.");
        return false;
    }

    std::ifstream file(pole_trajectory_file_);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open pole trajectory file: %s",
                     pole_trajectory_file_.c_str());
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
        TrajectoryPoint point{};
        if (!(iss >> point.t >> point.x >> point.y >> point.yaw)) {
            RCLCPP_ERROR(this->get_logger(),
                         "Invalid pole trajectory format at line %zu: %s. Expected: time_sec x y yaw",
                         line_no, line.c_str());
            pole_trajectory_.clear();
            return false;
        }
        point.s = 0.0;
        pole_trajectory_.push_back(point);
    }

    if (pole_trajectory_.size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "Pole trajectory file needs at least 2 valid points: %s",
                     pole_trajectory_file_.c_str());
        return false;
    }
    for (std::size_t i = 1; i < pole_trajectory_.size(); ++i) {
        if (pole_trajectory_[i].t <= pole_trajectory_[i - 1].t) {
            RCLCPP_ERROR(this->get_logger(),
                         "Pole trajectory time must be strictly increasing at index %zu", i);
            pole_trajectory_.clear();
            return false;
        }
    }

    const double t0 = pole_trajectory_.front().t;
    double accumulated_s = 0.0;
    for (auto& point : pole_trajectory_) {
        point.t -= t0;
    }
    pole_trajectory_.front().s = 0.0;
    for (std::size_t i = 1; i < pole_trajectory_.size(); ++i) {
        accumulated_s += std::hypot(
            pole_trajectory_[i].x - pole_trajectory_[i - 1].x,
            pole_trajectory_[i].y - pole_trajectory_[i - 1].y);
        pole_trajectory_[i].s = accumulated_s;
    }
    if (accumulated_s <= 1e-6) {
        RCLCPP_ERROR(this->get_logger(), "Pole trajectory path length is too small: %s",
                     pole_trajectory_file_.c_str());
        pole_trajectory_.clear();
        return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "Loaded %zu pole path points from %s, duration %.3f s, length %.3f m",
                pole_trajectory_.size(), pole_trajectory_file_.c_str(),
                pole_trajectory_.back().t, pole_trajectory_.back().s);
    return true;
}

/**
 * @brief 找当前位置到轨迹线段的最近投影弧长
 *
 * 输入：当前 x_/y_、pole_trajectory_
 * 输出：距离最近的轨迹弧长 s
 * 处理：逐线段投影，返回全局最近点对应弧长
 */
double TaskExecutorNode::find_closest_pole_path_s() const {
    if (!pole_trajectory_loaded_ || pole_trajectory_.empty()) {
        return 0.0;
    }

    double best_s = pole_trajectory_.front().s;
    double best_dist_sq = std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i + 1 < pole_trajectory_.size(); ++i) {
        const auto& a = pole_trajectory_[i];
        const auto& b = pole_trajectory_[i + 1];
        const double vx = b.x - a.x;
        const double vy = b.y - a.y;
        const double len_sq = vx * vx + vy * vy;
        double u = 0.0;
        if (len_sq > 1e-12) {
            u = ((x_ - a.x) * vx + (y_ - a.y) * vy) / len_sq;
            u = std::clamp(u, 0.0, 1.0);
        }
        const double proj_x = a.x + u * vx;
        const double proj_y = a.y + u * vy;
        const double dx = x_ - proj_x;
        const double dy = y_ - proj_y;
        const double dist_sq = dx * dx + dy * dy;
        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best_s = a.s + u * (b.s - a.s);
        }
    }

    return best_s;
}

/**
 * @brief 按路径弧长插值参考位姿
 *
 * 输入：s、pole_trajectory_
 * 输出：x/y/yaw 参考；返回值是否有效
 * 处理：首尾夹持或段内线性插值（yaw 经 normalize）
 */
bool TaskExecutorNode::sample_pole_reference_at_s(double s, double& x, double& y, double& yaw) const {
    if (!pole_trajectory_loaded_ || pole_trajectory_.empty()) {
        return false;
    }

    if (s <= pole_trajectory_.front().s) {
        x = pole_trajectory_.front().x;
        y = pole_trajectory_.front().y;
        yaw = pole_trajectory_.front().yaw;
        return true;
    }

    if (s >= pole_trajectory_.back().s) {
        x = pole_trajectory_.back().x;
        y = pole_trajectory_.back().y;
        yaw = pole_trajectory_.back().yaw;
        return true;
    }

    for (std::size_t i = 0; i + 1 < pole_trajectory_.size(); ++i) {
        const auto& a = pole_trajectory_[i];
        const auto& b = pole_trajectory_[i + 1];
        if (s >= a.s && s <= b.s) {
            const double ds_seg = b.s - a.s;
            const double alpha = (ds_seg > 1e-9) ? (s - a.s) / ds_seg : 0.0;
            x = a.x + alpha * (b.x - a.x);
            y = a.y + alpha * (b.y - a.y);
            const double dyaw = normalize_angle(b.yaw - a.yaw);
            yaw = normalize_angle(a.yaw + alpha * dyaw);
            return true;
        }
    }

    x = pole_trajectory_.back().x;
    y = pole_trajectory_.back().y;
    yaw = pole_trajectory_.back().yaw;
    return true;
}

/**
 * @brief 按最近投影点和固定距离前瞻生成参考位姿
 *
 * 输入：当前 x_/y_、pole_lookahead_distance_、pole_drive_mode_
 * 输出：参考 x/y/yaw、最近进度 closest_s、前瞻进度 target_s
 * 处理：路径进度单调不回退；reverse 模式 yaw 加 pi
 */
bool TaskExecutorNode::sample_pole_reference(
    double& x, double& y, double& yaw, double& closest_s, double& target_s)
{
    if (!pole_trajectory_loaded_ || pole_trajectory_.empty()) {
        return false;
    }

    closest_s = find_closest_pole_path_s();
    pole_path_progress_s_ = std::max(pole_path_progress_s_, closest_s);
    closest_s = pole_path_progress_s_;

    const double total_s = pole_trajectory_.back().s;
    target_s = std::min(total_s, closest_s + pole_lookahead_distance_);
    if (!sample_pole_reference_at_s(target_s, x, y, yaw)) {
        return false;
    }

    if (pole_drive_mode_ == "reverse") {
        yaw = normalize_angle(yaw + M_PI);
    }
    return true;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskExecutorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
