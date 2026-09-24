#include "task_manager/obstacle_nav_node.hpp"
#include "task_manager/lidar_pose_guard.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>

namespace {

std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> fields;
    std::string field;
    bool in_quotes = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char ch = line[i];
        if (ch == '"') {
            if (in_quotes && i + 1 < line.size() && line[i + 1] == '"') {
                field.push_back('"');
                i++;
                continue;
            }
            in_quotes = !in_quotes;
        } else if (ch == ',' && !in_quotes) {
            fields.push_back(field);
            field.clear();
        } else {
            field.push_back(ch);
        }
    }
    fields.push_back(field);
    return fields;
}

std::string trim_copy(const std::string& input) {
    std::size_t begin = 0;
    while (begin < input.size() && std::isspace(static_cast<unsigned char>(input[begin]))) {
        begin++;
    }
    std::size_t end = input.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(input[end - 1]))) {
        end--;
    }
    return input.substr(begin, end - begin);
}

bool parse_bool_field(const std::string& value) {
    const std::string trimmed = trim_copy(value);
    return trimmed == "true" || trimmed == "True" || trimmed == "1" || trimmed == "yes";
}

double interpolate_yaw(double yaw0, double yaw1, double ratio) {
    double diff = yaw1 - yaw0;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    return yaw0 + ratio * diff;
}

void apply_min_speed(double error, double min_speed, double error_deadzone, double& speed) {
    if (std::abs(error) <= error_deadzone || min_speed <= 0.0 || std::abs(speed) >= min_speed) {
        return;
    }
    speed = std::copysign(min_speed, speed == 0.0 ? error : speed);
}

bool parse_json_number_field(const std::string& object, const std::string& name, double& out) {
    const std::string key = "\"" + name + "\":";
    const std::size_t key_pos = object.find(key);
    if (key_pos == std::string::npos) {
        return false;
    }
    std::size_t value_begin = key_pos + key.size();
    std::size_t value_end = value_begin;
    while (value_end < object.size() && object[value_end] != ',' && object[value_end] != '}') {
        value_end++;
    }
    try {
        out = std::stod(trim_copy(object.substr(value_begin, value_end - value_begin)));
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool parse_json_string_field(const std::string& object, const std::string& name, std::string& out) {
    const std::string key = "\"" + name + "\":\"";
    const std::size_t key_pos = object.find(key);
    if (key_pos == std::string::npos) {
        return false;
    }
    const std::size_t value_begin = key_pos + key.size();
    const std::size_t value_end = object.find('"', value_begin);
    if (value_end == std::string::npos) {
        return false;
    }
    out = object.substr(value_begin, value_end - value_begin);
    return true;
}

bool parse_replay_action_steps_json(
    const std::string& json, std::vector<ReplayActionStep>& steps, std::string& error) {
    steps.clear();
    std::size_t pos = 0;
    while (true) {
        const std::size_t begin = json.find('{', pos);
        if (begin == std::string::npos) {
            break;
        }
        const std::size_t end = json.find('}', begin);
        if (end == std::string::npos) {
            error = "unterminated step object";
            return false;
        }
        const std::string object = json.substr(begin, end - begin + 1);
        ReplayActionStep step{};
        double code = 0.0;
        if (!parse_json_number_field(object, "delay_s", step.delay_s) ||
            !parse_json_number_field(object, "event_code", code) ||
            !parse_json_string_field(object, "event_name", step.event_name)) {
            error = "missing delay_s/event_code/event_name";
            return false;
        }
        step.event_code = static_cast<int>(std::llround(code));
        steps.push_back(step);
        pos = end + 1;
    }
    if (steps.empty()) {
        error = "empty steps_json";
        return false;
    }
    std::sort(steps.begin(), steps.end(), [](const ReplayActionStep& a, const ReplayActionStep& b) {
        return a.delay_s < b.delay_s;
    });
    return true;
}

}  // namespace

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

    RCLCPP_INFO(this->get_logger(), "ObstacleNavNode initialized. mode=%s",
                navigation_mode_param_.c_str());
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
    state_array_sub_ = this->create_subscription<std_msgs::msg::Int8MultiArray>(
        "state_array", 10, std::bind(&ObstacleNavNode::state_array_callback, this, std::placeholders::_1));
    tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        tf_topic_, 100, std::bind(&ObstacleNavNode::tf_callback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    high_command_pub_ = this->create_publisher<quad::msg::HighCommands>("high_command", 10);
    cmd_evt_pub_ = this->create_publisher<std_msgs::msg::Int8>("cmd_evt", 10);
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
    this->declare_parameter("navigation_mode", "POINT_SEQUENCE");
    this->declare_parameter("target_points", std::vector<double>{});
    this->declare_parameter("trajectory_files", std::vector<std::string>{});
    this->declare_parameter("record_parku_replay_path", "");
    this->declare_parameter("record_parku_events_path", "");
    this->declare_parameter("record_parku_action_sequences_path", "");

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
    this->declare_parameter("control_rate", 50.0);
    this->declare_parameter("lidar_offset_x", 0.05);
    this->declare_parameter("lidar_offset_y", 0.0);

    this->declare_parameter("replay_lookahead_radius", 0.25);
    this->declare_parameter("replay_intersection_search_dist", 1.0);
    this->declare_parameter("replay_fallback_lookahead", 0.18);
    this->declare_parameter("replay_min_forward_s", 0.03);
    this->declare_parameter("replay_min_target_xy", 0.08);
    this->declare_parameter("replay_yaw_in_place_search_s", 0.35);
    this->declare_parameter("replay_yaw_in_place_xy_thresh", 0.05);
    this->declare_parameter("replay_yaw_in_place_yaw_thresh", 0.35);
    this->declare_parameter("replay_projection_back_dist", 0.20);
    this->declare_parameter("replay_projection_forward_dist", 1.50);
    this->declare_parameter("replay_projection_yaw_weight", 0.25);
    this->declare_parameter("replay_kp_forward", 1.1);
    this->declare_parameter("replay_kp_lateral", 0.75);
    this->declare_parameter("replay_lateral_trim_gain", 0.75);
    this->declare_parameter("replay_kp_yaw", 1.5);
    this->declare_parameter("replay_max_vx", 0.45);
    this->declare_parameter("replay_max_vy", 0.60);
    this->declare_parameter("replay_max_wz", 0.70);
    this->declare_parameter("replay_min_vx", 0.20);
    this->declare_parameter("replay_min_vy", 0.35);
    this->declare_parameter("replay_deadzone_error_x", 0.04);
    this->declare_parameter("replay_deadzone_error_y", 0.04);
    this->declare_parameter("stuck_window_s", 1.5);
    this->declare_parameter("stuck_progress_epsilon", 0.03);
    this->declare_parameter("rewind_distance", 0.3);
    this->declare_parameter("rewind_cooldown_s", 1.0);
    this->declare_parameter("replay_rewind_speed_scale", 2.0);
    this->declare_parameter("replay_rewind_recover_speed_scale", 1.5);
    this->declare_parameter("replay_default_speed_scale", 1.3);
    this->declare_parameter("replay_upstair_speed_scale", 1.3);
    this->declare_parameter("replay_kneel_crawl_speed_scale", 2.2);
    this->declare_parameter("finish_s_tolerance", 0.08);
    this->declare_parameter("finish_xy_tolerance", 0.10);
    this->declare_parameter("finish_yaw_tolerance", 0.20);
    this->declare_parameter("replay_event_min_interval_s", 0.25);
    this->declare_parameter(
        "replay_event_blocklist",
        std::vector<int64_t>{0, 1, 3, 12, 13, 14, 15, 16, 17, 18, 19, 20});
    this->declare_parameter("replay_event_pause_codes", std::vector<int64_t>{4, 5, 6, 7, 8, 9, 10, 11});
    this->declare_parameter("replay_event_pause_durations", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    this->declare_parameter("replay_action_sequence_enable", true);
    this->declare_parameter("replay_action_event_codes", std::vector<int64_t>{2, 4, 5, 6, 7, 8, 21});
    this->declare_parameter("replay_action_group_max_dt", 2.0);
    this->declare_parameter("replay_action_group_max_xy", 0.20);
    this->declare_parameter("replay_action_group_max_ds", 0.30);
    this->declare_parameter("replay_action_finish_enter_rl", false);
    this->declare_parameter("replay_action_finish_wait_s", 0.30);
    this->declare_parameter("replay_action_hold_codes", std::vector<int64_t>{5, 6, 7, 8});
    this->declare_parameter("replay_action_hold_durations", std::vector<double>{1.0, 1.0, 1.5, 1.0});
    this->declare_parameter("replay_action_recovery_interval_s", 0.35);
    this->declare_parameter("replay_skip_stall_enable", true);
    this->declare_parameter("replay_skip_stall_window_s", 1.0);
    this->declare_parameter("replay_skip_stall_progress_epsilon", 0.01);
    this->declare_parameter("replay_skip_stall_xy_error", 0.05);
    this->declare_parameter("replay_skip_stall_cmd_xy", 0.05);
    this->declare_parameter("replay_skip_stall_advance_s", 0.20);
    this->declare_parameter("replay_skip_stall_guard_s", 0.25);
    this->declare_parameter("replay_publish_high_command", true);

    target_frame_ = this->get_parameter("target_frame").as_string();
    child_frame_ = this->get_parameter("child_frame").as_string();
    tf_topic_ = this->get_parameter("tf_topic").as_string();
    navigation_mode_param_ = this->get_parameter("navigation_mode").as_string();
    target_points_flat_ = this->get_parameter("target_points").as_double_array();
    trajectory_files_ = this->get_parameter("trajectory_files").as_string_array();
    record_parku_replay_path_ = this->get_parameter("record_parku_replay_path").as_string();
    record_parku_events_path_ = this->get_parameter("record_parku_events_path").as_string();
    record_parku_action_sequences_path_ = this->get_parameter("record_parku_action_sequences_path").as_string();

    if (navigation_mode_param_ == "POINT_SEQUENCE") {
        nav_mode_ = NavMode::POINT_SEQUENCE;
    } else if (navigation_mode_param_ == "TRAJECTORY_SEQUENCE") {
        nav_mode_ = NavMode::TRAJECTORY_SEQUENCE;
    } else if (navigation_mode_param_ == "RECORD_PARKU_REPLAY") {
        nav_mode_ = NavMode::RECORD_PARKU_REPLAY;
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
    control_rate_ = this->get_parameter("control_rate").as_double();
    lidar_offset_x_ = this->get_parameter("lidar_offset_x").as_double();
    lidar_offset_y_ = this->get_parameter("lidar_offset_y").as_double();
    replay_lookahead_radius_ = this->get_parameter("replay_lookahead_radius").as_double();
    replay_intersection_search_dist_ = this->get_parameter("replay_intersection_search_dist").as_double();
    replay_fallback_lookahead_ = this->get_parameter("replay_fallback_lookahead").as_double();
    replay_min_forward_s_ = this->get_parameter("replay_min_forward_s").as_double();
    replay_min_target_xy_ = this->get_parameter("replay_min_target_xy").as_double();
    replay_yaw_in_place_search_s_ = this->get_parameter("replay_yaw_in_place_search_s").as_double();
    replay_yaw_in_place_xy_thresh_ = this->get_parameter("replay_yaw_in_place_xy_thresh").as_double();
    replay_yaw_in_place_yaw_thresh_ = this->get_parameter("replay_yaw_in_place_yaw_thresh").as_double();
    replay_projection_back_dist_ = this->get_parameter("replay_projection_back_dist").as_double();
    replay_projection_forward_dist_ = this->get_parameter("replay_projection_forward_dist").as_double();
    replay_projection_yaw_weight_ = this->get_parameter("replay_projection_yaw_weight").as_double();
    replay_kp_forward_ = this->get_parameter("replay_kp_forward").as_double();
    replay_kp_lateral_ = this->get_parameter("replay_kp_lateral").as_double();
    replay_lateral_trim_gain_ = this->get_parameter("replay_lateral_trim_gain").as_double();
    replay_kp_yaw_ = this->get_parameter("replay_kp_yaw").as_double();
    replay_max_vx_ = this->get_parameter("replay_max_vx").as_double();
    replay_max_vy_ = this->get_parameter("replay_max_vy").as_double();
    replay_max_wz_ = this->get_parameter("replay_max_wz").as_double();
    replay_min_vx_ = this->get_parameter("replay_min_vx").as_double();
    replay_min_vy_ = this->get_parameter("replay_min_vy").as_double();
    replay_deadzone_error_x_ = this->get_parameter("replay_deadzone_error_x").as_double();
    replay_deadzone_error_y_ = this->get_parameter("replay_deadzone_error_y").as_double();
    stuck_window_s_ = this->get_parameter("stuck_window_s").as_double();
    stuck_progress_epsilon_ = this->get_parameter("stuck_progress_epsilon").as_double();
    rewind_distance_ = this->get_parameter("rewind_distance").as_double();
    rewind_cooldown_s_ = this->get_parameter("rewind_cooldown_s").as_double();
    replay_rewind_speed_scale_ = this->get_parameter("replay_rewind_speed_scale").as_double();
    replay_rewind_recover_speed_scale_ = this->get_parameter("replay_rewind_recover_speed_scale").as_double();
    replay_default_speed_scale_ = this->get_parameter("replay_default_speed_scale").as_double();
    replay_upstair_speed_scale_ = this->get_parameter("replay_upstair_speed_scale").as_double();
    replay_kneel_crawl_speed_scale_ = this->get_parameter("replay_kneel_crawl_speed_scale").as_double();
    finish_s_tolerance_ = this->get_parameter("finish_s_tolerance").as_double();
    finish_xy_tolerance_ = this->get_parameter("finish_xy_tolerance").as_double();
    finish_yaw_tolerance_ = this->get_parameter("finish_yaw_tolerance").as_double();
    replay_event_min_interval_s_ = this->get_parameter("replay_event_min_interval_s").as_double();
    replay_event_blocklist_ = this->get_parameter("replay_event_blocklist").as_integer_array();
    replay_event_pause_codes_ = this->get_parameter("replay_event_pause_codes").as_integer_array();
    replay_event_pause_durations_ = this->get_parameter("replay_event_pause_durations").as_double_array();
    if (replay_event_pause_codes_.size() != replay_event_pause_durations_.size()) {
        RCLCPP_WARN(this->get_logger(),
                    "replay_event_pause_codes and replay_event_pause_durations size mismatch; event pauses disabled.");
        replay_event_pause_codes_.clear();
        replay_event_pause_durations_.clear();
    }
    replay_action_sequence_enable_ = this->get_parameter("replay_action_sequence_enable").as_bool();
    replay_action_event_codes_ = this->get_parameter("replay_action_event_codes").as_integer_array();
    replay_action_group_max_dt_ = this->get_parameter("replay_action_group_max_dt").as_double();
    replay_action_group_max_xy_ = this->get_parameter("replay_action_group_max_xy").as_double();
    replay_action_group_max_ds_ = this->get_parameter("replay_action_group_max_ds").as_double();
    replay_action_finish_enter_rl_ = this->get_parameter("replay_action_finish_enter_rl").as_bool();
    replay_action_finish_wait_s_ = this->get_parameter("replay_action_finish_wait_s").as_double();
    replay_action_hold_codes_ = this->get_parameter("replay_action_hold_codes").as_integer_array();
    replay_action_hold_durations_ = this->get_parameter("replay_action_hold_durations").as_double_array();
    if (replay_action_hold_codes_.size() != replay_action_hold_durations_.size()) {
        RCLCPP_WARN(this->get_logger(),
                    "replay_action_hold_codes and replay_action_hold_durations size mismatch; action holds disabled.");
        replay_action_hold_codes_.clear();
        replay_action_hold_durations_.clear();
    }
    replay_action_recovery_interval_s_ = this->get_parameter("replay_action_recovery_interval_s").as_double();
    replay_skip_stall_enable_ = this->get_parameter("replay_skip_stall_enable").as_bool();
    replay_skip_stall_window_s_ = this->get_parameter("replay_skip_stall_window_s").as_double();
    replay_skip_stall_progress_epsilon_ = this->get_parameter("replay_skip_stall_progress_epsilon").as_double();
    replay_skip_stall_xy_error_ = this->get_parameter("replay_skip_stall_xy_error").as_double();
    replay_skip_stall_cmd_xy_ = this->get_parameter("replay_skip_stall_cmd_xy").as_double();
    replay_skip_stall_advance_s_ = this->get_parameter("replay_skip_stall_advance_s").as_double();
    replay_skip_stall_guard_s_ = this->get_parameter("replay_skip_stall_guard_s").as_double();
    replay_publish_high_command_ = this->get_parameter("replay_publish_high_command").as_bool();
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

bool ObstacleNavNode::load_replay_path_file() {
    replay_path_.clear();
    if (record_parku_replay_path_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "record_parku_replay_path is empty.");
        return false;
    }

    std::ifstream file(record_parku_replay_path_);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open replay path CSV: %s",
                     record_parku_replay_path_.c_str());
        return false;
    }

    std::string line;
    if (!std::getline(file, line)) {
        RCLCPP_ERROR(this->get_logger(), "Replay path CSV is empty: %s",
                     record_parku_replay_path_.c_str());
        return false;
    }
    const auto header = split_csv_line(line);
    std::map<std::string, std::size_t> columns;
    for (std::size_t i = 0; i < header.size(); ++i) {
        columns[trim_copy(header[i])] = i;
    }
    for (const char* required : {"s", "x", "y", "yaw"}) {
        if (columns.count(required) == 0) {
            RCLCPP_ERROR(this->get_logger(), "Replay path CSV missing column '%s': %s",
                         required, record_parku_replay_path_.c_str());
            return false;
        }
    }

    std::size_t line_no = 1;
    while (std::getline(file, line)) {
        line_no++;
        if (trim_copy(line).empty()) {
            continue;
        }
        const auto fields = split_csv_line(line);
        const auto read_double = [&](const char* name) -> double {
            const auto index = columns[name];
            if (index >= fields.size()) {
                throw std::runtime_error(std::string("missing field ") + name);
            }
            return std::stod(trim_copy(fields[index]));
        };
        try {
            ReplayPathPoint point{};
            point.s = read_double("s");
            point.x = read_double("x");
            point.y = read_double("y");
            point.yaw = read_double("yaw");
            if (!std::isfinite(point.s) || !std::isfinite(point.x) ||
                !std::isfinite(point.y) || !std::isfinite(point.yaw)) {
                throw std::runtime_error("non-finite replay path value");
            }
            replay_path_.push_back(point);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Invalid replay path row at %s:%zu: %s",
                         record_parku_replay_path_.c_str(), line_no, e.what());
            replay_path_.clear();
            return false;
        }
    }

    if (replay_path_.size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "Replay path needs at least two points: %s",
                     record_parku_replay_path_.c_str());
        replay_path_.clear();
        return false;
    }
    std::sort(replay_path_.begin(), replay_path_.end(),
              [](const ReplayPathPoint& a, const ReplayPathPoint& b) { return a.s < b.s; });
    RCLCPP_INFO(this->get_logger(), "Loaded replay path: %s (%zu points, length %.3fm)",
                record_parku_replay_path_.c_str(), replay_path_.size(), replay_path_.back().s);
    return true;
}

bool ObstacleNavNode::load_replay_events_file() {
    replay_events_.clear();
    if (record_parku_events_path_.empty()) {
        RCLCPP_WARN(this->get_logger(), "record_parku_events_path is empty; replay will run without events.");
        return true;
    }

    std::ifstream file(record_parku_events_path_);
    if (!file.is_open()) {
        RCLCPP_WARN(this->get_logger(), "Failed to open replay events CSV: %s; replay will run without events.",
                    record_parku_events_path_.c_str());
        return true;
    }

    std::string line;
    if (!std::getline(file, line)) {
        RCLCPP_WARN(this->get_logger(), "Replay events CSV is empty: %s", record_parku_events_path_.c_str());
        return true;
    }
    const auto header = split_csv_line(line);
    std::map<std::string, std::size_t> columns;
    for (std::size_t i = 0; i < header.size(); ++i) {
        columns[trim_copy(header[i])] = i;
    }
    for (const char* required : {"event_id", "s", "x", "y", "yaw", "event_code", "event_name", "repeat_on_rewind"}) {
        if (columns.count(required) == 0) {
            RCLCPP_ERROR(this->get_logger(), "Replay events CSV missing column '%s': %s",
                         required, record_parku_events_path_.c_str());
            return false;
        }
    }

    std::size_t line_no = 1;
    while (std::getline(file, line)) {
        line_no++;
        if (trim_copy(line).empty()) {
            continue;
        }
        const auto fields = split_csv_line(line);
        const auto field = [&](const char* name) -> std::string {
            const auto index = columns[name];
            if (index >= fields.size()) {
                throw std::runtime_error(std::string("missing field ") + name);
            }
            return trim_copy(fields[index]);
        };
        try {
            ReplayEvent event{};
            event.event_id = std::stoi(field("event_id"));
            event.s = std::stod(field("s"));
            event.x = std::stod(field("x"));
            event.y = std::stod(field("y"));
            event.yaw = std::stod(field("yaw"));
            event.event_code = std::stoi(field("event_code"));
            event.event_name = field("event_name");
            event.repeat_on_rewind = parse_bool_field(field("repeat_on_rewind"));
            event.triggered = false;
            replay_events_.push_back(event);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Invalid replay event row at %s:%zu: %s",
                         record_parku_events_path_.c_str(), line_no, e.what());
            replay_events_.clear();
            return false;
        }
    }

    std::sort(replay_events_.begin(), replay_events_.end(),
              [](const ReplayEvent& a, const ReplayEvent& b) {
                  if (a.s == b.s) {
                      return a.event_id < b.event_id;
                  }
                  return a.s < b.s;
              });
    RCLCPP_INFO(this->get_logger(), "Loaded replay events: %s (%zu events)",
                record_parku_events_path_.c_str(), replay_events_.size());
    return true;
}

bool ObstacleNavNode::load_replay_action_sequences_file() {
    replay_action_sequences_.clear();
    if (!replay_action_sequence_enable_) {
        RCLCPP_INFO(this->get_logger(), "Replay action sequences disabled.");
        return true;
    }
    if (record_parku_action_sequences_path_.empty()) {
        RCLCPP_WARN(this->get_logger(),
                    "record_parku_action_sequences_path is empty; replay will run without action sequences.");
        return true;
    }

    std::ifstream file(record_parku_action_sequences_path_);
    if (!file.is_open()) {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to open replay action sequences CSV: %s; replay will run without action sequences.",
                    record_parku_action_sequences_path_.c_str());
        return true;
    }

    std::string line;
    if (!std::getline(file, line)) {
        RCLCPP_WARN(this->get_logger(), "Replay action sequences CSV is empty: %s",
                    record_parku_action_sequences_path_.c_str());
        return true;
    }
    const auto header = split_csv_line(line);
    std::map<std::string, std::size_t> columns;
    for (std::size_t i = 0; i < header.size(); ++i) {
        columns[trim_copy(header[i])] = i;
    }
    for (const char* required : {
             "sequence_id", "trigger_s", "trigger_x", "trigger_y", "trigger_yaw",
             "start_time", "end_time", "duration_s", "steps_json", "repeat_on_rewind"}) {
        if (columns.count(required) == 0) {
            RCLCPP_ERROR(this->get_logger(), "Replay action sequences CSV missing column '%s': %s",
                         required, record_parku_action_sequences_path_.c_str());
            return false;
        }
    }

    std::size_t line_no = 1;
    while (std::getline(file, line)) {
        line_no++;
        if (trim_copy(line).empty()) {
            continue;
        }
        const auto fields = split_csv_line(line);
        const auto field = [&](const char* name) -> std::string {
            const auto index = columns[name];
            if (index >= fields.size()) {
                throw std::runtime_error(std::string("missing field ") + name);
            }
            return trim_copy(fields[index]);
        };
        try {
            ReplayActionSequence sequence{};
            sequence.sequence_id = std::stoi(field("sequence_id"));
            sequence.trigger_s = std::stod(field("trigger_s"));
            sequence.trigger_x = std::stod(field("trigger_x"));
            sequence.trigger_y = std::stod(field("trigger_y"));
            sequence.trigger_yaw = std::stod(field("trigger_yaw"));
            sequence.start_time = std::stod(field("start_time"));
            sequence.end_time = std::stod(field("end_time"));
            sequence.duration_s = std::stod(field("duration_s"));
            sequence.repeat_on_rewind = parse_bool_field(field("repeat_on_rewind"));
            sequence.triggered = false;
            std::string parse_error;
            if (!parse_replay_action_steps_json(field("steps_json"), sequence.steps, parse_error)) {
                throw std::runtime_error("invalid steps_json: " + parse_error);
            }
            replay_action_sequences_.push_back(sequence);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Invalid replay action sequence row at %s:%zu: %s",
                         record_parku_action_sequences_path_.c_str(), line_no, e.what());
            replay_action_sequences_.clear();
            return false;
        }
    }

    std::sort(replay_action_sequences_.begin(), replay_action_sequences_.end(),
              [](const ReplayActionSequence& a, const ReplayActionSequence& b) {
                  if (a.trigger_s == b.trigger_s) {
                      return a.sequence_id < b.sequence_id;
                  }
                  return a.trigger_s < b.trigger_s;
              });
    RCLCPP_INFO(this->get_logger(), "Loaded replay action sequences: %s (%zu sequences)",
                record_parku_action_sequences_path_.c_str(), replay_action_sequences_.size());
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
    if (nav_mode_ == NavMode::RECORD_PARKU_REPLAY) {
        return 1;
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
    nav_status_sent_ = false;

    if (nav_mode_ == NavMode::RECORD_PARKU_REPLAY) {
        reset_replay_state();
        replay_loaded_ = load_replay_path_file() && load_replay_events_file() && load_replay_action_sequences_file();
        return replay_loaded_;
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

void ObstacleNavNode::state_array_callback(const std_msgs::msg::Int8MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 2) {
        replay_lower_state_ = static_cast<int>(msg->data[1]);
    }
}

/**
 * @brief 处理 nav_enable 由 true 变 false
 *
 * 输入：previous_enabled（变更前是否使能）、runtime_state_
 * 输出：可能 stop_robot、更新 runtime_state_
 * 处理：从 true 变 false 时停车；RUNNING/WAIT_TASK→WAIT_NEXT_ENABLE，其它非 FINISHED→IDLE
 */
void ObstacleNavNode::on_nav_enable_disabled(bool previous_enabled) {
    if (previous_enabled) {
        stop_robot();
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
    } else {
        runtime_state_ = RuntimeState::FINISHED;
        publish_nav_finished_once();
        stop_robot();
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
 * 输出：WaypointTrackingErrors（距离与机体系 xyz 误差）
 * 处理：世界系差分后按当前 yaw 旋转到机体系
 */
WaypointTrackingErrors ObstacleNavNode::compute_waypoint_errors(const ObstacleNavPoint& target) const {
    const double dx_world = target.x - x_;
    const double dy_world = target.y - y_;
    const double err_yaw = normalize_angle(target.yaw - yaw_);
    const double dist = std::hypot(dx_world, dy_world);
    double err_x_body = 0.0;
    double err_y_body = 0.0;
    map_error_to_body(dx_world, dy_world, err_x_body, err_y_body);
    return {dist, err_x_body, err_y_body, err_yaw};
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
 * 处理：航向误差大于 yaw_first_threshold_ 时只转；否则 vx/vy/wz PD+限幅
 */
void ObstacleNavNode::compute_pd_velocity_cmd(const WaypointTrackingErrors& errors, double dist) {
    const auto now = this->now();
    if (!pd_initialized_) {
        prev_err_x_body_ = errors.err_x_body;
        prev_err_y_body_ = errors.err_y_body;
        prev_err_yaw_ = errors.err_yaw;
        prev_ctrl_time_ = now;
        pd_initialized_ = true;
    }
    double dt = (now - prev_ctrl_time_).seconds();
    dt = std::max(1e-4, std::min(dt, 0.25));

    const double dex_dt = (errors.err_x_body - prev_err_x_body_) / dt;
    const double dey_dt = (errors.err_y_body - prev_err_y_body_) / dt;
    const double deyaw_dt = normalize_angle(errors.err_yaw - prev_err_yaw_) / dt;

    prev_err_x_body_ = errors.err_x_body;
    prev_err_y_body_ = errors.err_y_body;
    prev_err_yaw_ = errors.err_yaw;
    prev_ctrl_time_ = now;

    geometry_msgs::msg::Twist cmd;
    const double wz = kp_yaw_ * errors.err_yaw + kd_yaw_ * deyaw_dt;
    if (std::abs(errors.err_yaw) > yaw_first_threshold_) {
        cmd.angular.z = std::clamp(wz, -max_dyaw_, max_dyaw_);
    } else {
        cmd.linear.x = std::clamp(kp_x_ * errors.err_x_body + kd_x_ * dex_dt, -max_vx_, max_vx_);
        cmd.linear.y = std::clamp(kp_y_ * errors.err_y_body + kd_y_ * dey_dt, -max_vy_, max_vy_);
        cmd.angular.z = std::clamp(wz, -max_dyaw_, max_dyaw_);
    }
    cmd_vel_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "ObstacleNav target=%zu point=%zu/%zu dist=%.2f err_body=(%.2f,%.2f,%.2f) cmd=(%.2f,%.2f,%.2f)",
        current_target_index_ + 1, current_path_index_ + 1, current_path_.size(), dist,
        errors.err_x_body, errors.err_y_body, errors.err_yaw, cmd.linear.x, cmd.linear.y, cmd.angular.z);
}

void ObstacleNavNode::reset_replay_state() {
    replay_phase_ = ReplayPhase::FORWARD;
    replay_s_initialized_ = false;
    replay_last_forward_yaw_only_ = false;
    replay_last_s_ = 0.0;
    replay_skip_min_s_ = 0.0;
    replay_rewind_from_s_ = 0.0;
    replay_rewind_target_s_ = 0.0;
    replay_rewind_recover_boost_active_ = false;
    replay_progress_history_.clear();
    replay_pending_event_indices_.clear();
    replay_current_policy_ = 0;
    replay_last_rewind_end_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    replay_last_event_pub_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    replay_pause_until_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    replay_action_sequence_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    replay_action_last_step_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    replay_action_next_finish_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    replay_action_next_recovery_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    replay_action_sequence_active_ = false;
    replay_action_finish_sent_ = false;
    replay_action_hold_done_ = false;
    replay_active_action_sequence_index_ = 0;
    replay_active_action_step_index_ = 0;
    replay_action_last_event_code_ = -1;
    for (auto& event : replay_events_) {
        event.triggered = false;
    }
    for (auto& sequence : replay_action_sequences_) {
        sequence.triggered = false;
    }
}

void ObstacleNavNode::map_error_to_body(double err_x_map, double err_y_map,
                                        double& err_x_body, double& err_y_body) const {
    const double cos_yaw = std::cos(yaw_);
    const double sin_yaw = std::sin(yaw_);
    const double err_x_lidar = cos_yaw * err_y_map - sin_yaw * err_x_map;
    const double err_y_lidar = -sin_yaw * err_y_map - cos_yaw * err_x_map;
    err_x_body = err_x_lidar + lidar_offset_x_;
    err_y_body = err_y_lidar + lidar_offset_y_;
}

bool ObstacleNavNode::sample_replay_path(double s, ReplayPathPoint& out_point) const {
    if (replay_path_.empty()) {
        return false;
    }
    s = std::clamp(s, replay_path_.front().s, replay_path_.back().s);
    if (s <= replay_path_.front().s) {
        out_point = replay_path_.front();
        return true;
    }
    if (s >= replay_path_.back().s) {
        out_point = replay_path_.back();
        return true;
    }

    const auto upper = std::lower_bound(
        replay_path_.begin(), replay_path_.end(), s,
        [](const ReplayPathPoint& point, double value) { return point.s < value; });
    if (upper == replay_path_.begin() || upper == replay_path_.end()) {
        return false;
    }
    const auto& p1 = *upper;
    const auto& p0 = *(upper - 1);
    const double ds = std::max(1e-9, p1.s - p0.s);
    const double ratio = std::clamp((s - p0.s) / ds, 0.0, 1.0);
    out_point.s = s;
    out_point.x = p0.x + ratio * (p1.x - p0.x);
    out_point.y = p0.y + ratio * (p1.y - p0.y);
    out_point.yaw = normalize_angle(interpolate_yaw(p0.yaw, p1.yaw, ratio));
    return true;
}

bool ObstacleNavNode::project_current_pose_to_replay_path(double& out_s, double& out_distance) {
    if (replay_path_.size() < 2) {
        return false;
    }

    double search_min_s = replay_path_.front().s;
    double search_max_s = replay_path_.back().s;
    if (replay_s_initialized_) {
        if (replay_phase_ == ReplayPhase::REWIND) {
            search_min_s = std::max(replay_path_.front().s, replay_last_s_ - rewind_distance_ - replay_projection_back_dist_);
            search_max_s = std::min(replay_path_.back().s, replay_last_s_ + replay_projection_back_dist_);
        } else {
            search_min_s = std::max(replay_path_.front().s, replay_last_s_ - replay_projection_back_dist_);
            search_max_s = std::min(replay_path_.back().s, replay_last_s_ + replay_projection_forward_dist_);
        }
    }

    double best_dist2 = std::numeric_limits<double>::infinity();
    double best_s = replay_path_.front().s;
    for (std::size_t i = 0; i + 1 < replay_path_.size(); ++i) {
        const auto& a = replay_path_[i];
        const auto& b = replay_path_[i + 1];
        if (b.s < search_min_s || a.s > search_max_s) {
            continue;
        }
        const double vx = b.x - a.x;
        const double vy = b.y - a.y;
        const double len2 = vx * vx + vy * vy;
        double ratio = 0.0;
        if (len2 < 1e-12) {
            const double yaw_span = normalize_angle(b.yaw - a.yaw);
            const double abs_yaw_span = std::abs(yaw_span);
            if (abs_yaw_span < 1e-6) {
                continue;
            }
            const double yaw_from_a = normalize_angle(yaw_ - a.yaw);
            ratio = std::clamp(yaw_from_a / yaw_span, 0.0, 1.0);
        } else {
            ratio = ((x_ - a.x) * vx + (y_ - a.y) * vy) / len2;
            ratio = std::clamp(ratio, 0.0, 1.0);
        }
        const double px = a.x + ratio * vx;
        const double py = a.y + ratio * vy;
        const double pyaw = normalize_angle(interpolate_yaw(a.yaw, b.yaw, ratio));
        const double dx = x_ - px;
        const double dy = y_ - py;
        const double dyaw = normalize_angle(yaw_ - pyaw);
        const double weighted_yaw = replay_projection_yaw_weight_ * dyaw;
        const double dist2 = dx * dx + dy * dy + weighted_yaw * weighted_yaw;
        const double candidate_s = a.s + ratio * (b.s - a.s);
        if (dist2 < best_dist2 ||
            (std::abs(dist2 - best_dist2) < 1e-9 && candidate_s < best_s)) {
            best_dist2 = dist2;
            best_s = candidate_s;
        }
    }

    if (!std::isfinite(best_dist2)) {
        return false;
    }
    if (replay_phase_ == ReplayPhase::FORWARD && replay_skip_min_s_ > best_s) {
        best_s = std::min(replay_skip_min_s_, replay_path_.back().s);
    }
    out_s = best_s;
    out_distance = std::sqrt(best_dist2);
    return true;
}

bool ObstacleNavNode::find_forward_circle_target(double current_s, ReplayPathPoint& out_target) const {
    if (replay_path_.size() < 2) {
        return false;
    }

    const double min_s = current_s + replay_min_forward_s_;
    const double max_s = std::min(replay_path_.back().s, current_s + replay_intersection_search_dist_);
    const double r2 = replay_lookahead_radius_ * replay_lookahead_radius_;
    double best_s = std::numeric_limits<double>::infinity();
    bool found = false;

    for (std::size_t i = 0; i + 1 < replay_path_.size(); ++i) {
        const auto& a = replay_path_[i];
        const auto& b = replay_path_[i + 1];
        if (b.s < min_s || a.s > max_s) {
            continue;
        }
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double fx = a.x - x_;
        const double fy = a.y - y_;
        const double qa = dx * dx + dy * dy;
        if (qa < 1e-12) {
            continue;
        }
        const double qb = 2.0 * (fx * dx + fy * dy);
        const double qc = fx * fx + fy * fy - r2;
        const double disc = qb * qb - 4.0 * qa * qc;
        if (disc < 0.0) {
            continue;
        }
        const double root = std::sqrt(disc);
        for (const double ratio : {(-qb - root) / (2.0 * qa), (-qb + root) / (2.0 * qa)}) {
            if (ratio < -1e-9 || ratio > 1.0 + 1e-9) {
                continue;
            }
            const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);
            const double candidate_s = a.s + clamped_ratio * (b.s - a.s);
            if (candidate_s >= min_s && candidate_s <= max_s && candidate_s < best_s) {
                best_s = candidate_s;
                found = true;
            }
        }
    }

    if (found) {
        return sample_replay_path(best_s, out_target);
    }
    return find_forward_fallback_target(current_s, out_target);
}

bool ObstacleNavNode::find_forward_fallback_target(double current_s, ReplayPathPoint& out_target) const {
    if (replay_path_.empty()) {
        return false;
    }

    const double min_s = current_s + replay_min_forward_s_;
    const double preferred_s = current_s + replay_fallback_lookahead_;
    const double max_s = std::min(
        replay_path_.back().s,
        current_s + std::max(replay_fallback_lookahead_, replay_intersection_search_dist_));
    const double min_xy = std::max(0.0, replay_min_target_xy_);

    ReplayPathPoint preferred{};
    if (sample_replay_path(preferred_s, preferred)) {
        const double preferred_xy = std::hypot(preferred.x - x_, preferred.y - y_);
        if (preferred_xy >= min_xy || preferred.s >= replay_path_.back().s - 1e-6) {
            out_target = preferred;
            return true;
        }
    }

    for (const auto& point : replay_path_) {
        if (point.s < min_s) {
            continue;
        }
        if (point.s > max_s) {
            break;
        }
        if (std::hypot(point.x - x_, point.y - y_) >= min_xy) {
            out_target = point;
            return true;
        }
    }

    if (find_yaw_in_place_target(current_s, out_target)) {
        return true;
    }

    return sample_replay_path(preferred_s, out_target);
}

bool ObstacleNavNode::find_yaw_in_place_target(double current_s, ReplayPathPoint& out_target) const {
    if (replay_path_.empty()) {
        return false;
    }

    const double min_s = current_s + replay_min_forward_s_;
    const double max_s = std::min(
        replay_path_.back().s,
        current_s + std::max(replay_min_forward_s_, replay_yaw_in_place_search_s_));

    ReplayPathPoint target{};
    bool have_target = false;
    for (const auto& point : replay_path_) {
        if (point.s < min_s) {
            continue;
        }
        if (point.s > max_s) {
            break;
        }
        const double xy = std::hypot(point.x - x_, point.y - y_);
        const double yaw_delta = std::abs(normalize_angle(point.yaw - yaw_));
        if (xy <= replay_yaw_in_place_xy_thresh_ && yaw_delta >= replay_yaw_in_place_yaw_thresh_) {
            target = point;
            have_target = true;
            break;
        }
    }

    if (!have_target) {
        return false;
    }
    out_target = target;
    return true;
}

geometry_msgs::msg::Twist ObstacleNavNode::compute_replay_cmd(
    const ReplayPathPoint& target, double& err_x_body, double& err_y_body, double& err_yaw) const {
    const double err_x_map = target.x - x_;
    const double err_y_map = target.y - y_;
    err_yaw = normalize_angle(target.yaw - yaw_);
    map_error_to_body(err_x_map, err_y_map, err_x_body, err_y_body);

    geometry_msgs::msg::Twist cmd;
    double vx = replay_kp_forward_ * err_x_body;
    double vy = replay_kp_lateral_ * err_y_body;
    double wz = replay_kp_yaw_ * err_yaw;
    apply_min_speed(err_x_body, replay_min_vx_, replay_deadzone_error_x_, vx);
    apply_min_speed(err_y_body, replay_min_vy_, replay_deadzone_error_y_, vy);
    cmd.linear.x = std::clamp(vx, -replay_max_vx_, replay_max_vx_);
    cmd.linear.y = std::clamp(vy, -replay_max_vy_, replay_max_vy_);
    cmd.angular.z = std::clamp(wz, -replay_max_wz_, replay_max_wz_);
    return cmd;
}

geometry_msgs::msg::Twist ObstacleNavNode::compute_replay_yaw_only_cmd(
    const ReplayPathPoint& target, double& err_x_body, double& err_y_body, double& err_yaw) const {
    const double err_x_map = target.x - x_;
    const double err_y_map = target.y - y_;
    map_error_to_body(err_x_map, err_y_map, err_x_body, err_y_body);
    err_yaw = normalize_angle(target.yaw - yaw_);

    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(replay_kp_yaw_ * err_yaw, -replay_max_wz_, replay_max_wz_);
    return cmd;
}

void ObstacleNavNode::publish_replay_target_debug(
    const ReplayPathPoint& target, double err_x_body, double err_y_body, double err_yaw) {
    std_msgs::msg::Float32MultiArray target_msg;
    target_msg.data = {
        static_cast<float>(target.x),
        static_cast<float>(target.y),
        static_cast<float>(target.yaw),
        static_cast<float>(target.s)
    };
    current_target_pub_->publish(target_msg);

    geometry_msgs::msg::Point err_msg;
    err_msg.x = err_x_body;
    err_msg.y = err_y_body;
    err_msg.z = err_yaw;
    debug_error_pub_->publish(err_msg);
}

void ObstacleNavNode::trigger_forward_events(double previous_s, double current_s) {
    if (current_s < previous_s) {
        return;
    }
    for (std::size_t i = 0; i < replay_events_.size(); ++i) {
        auto& event = replay_events_[i];
        if (event.triggered) {
            continue;
        }
        if (event.s > previous_s && event.s <= current_s) {
            event.triggered = true;
            if (replay_action_sequence_enable_ && !replay_action_sequences_.empty() &&
                is_replay_action_event(event.event_code)) {
                RCLCPP_INFO(this->get_logger(), "Replay event covered by action sequence path: id=%d name=%s code=%d s=%.3f",
                            event.event_id, event.event_name.c_str(), event.event_code, event.s);
                continue;
            }
            if (is_replay_event_blocked(event.event_code)) {
                RCLCPP_INFO(this->get_logger(), "Replay event skipped: id=%d name=%s code=%d s=%.3f",
                            event.event_id, event.event_name.c_str(), event.event_code, event.s);
                continue;
            }
            replay_pending_event_indices_.push_back(i);
            RCLCPP_INFO(this->get_logger(), "Replay event queued: id=%d name=%s code=%d s=%.3f",
                        event.event_id, event.event_name.c_str(), event.event_code, event.s);
        }
    }
}

bool ObstacleNavNode::trigger_forward_action_sequences(double previous_s, double current_s) {
    if (!replay_action_sequence_enable_ || replay_action_sequence_active_ || current_s < previous_s) {
        return false;
    }
    for (std::size_t i = 0; i < replay_action_sequences_.size(); ++i) {
        auto& sequence = replay_action_sequences_[i];
        if (sequence.triggered) {
            continue;
        }
        if (sequence.trigger_s > previous_s && sequence.trigger_s <= current_s) {
            sequence.triggered = true;
            start_replay_action_sequence(i, this->now());
            return true;
        }
    }
    return false;
}

bool ObstacleNavNode::is_replay_event_blocked(int event_code) const {
    return std::find(replay_event_blocklist_.begin(), replay_event_blocklist_.end(), event_code) !=
           replay_event_blocklist_.end();
}

bool ObstacleNavNode::is_replay_action_event(int event_code) const {
    return std::find(replay_action_event_codes_.begin(), replay_action_event_codes_.end(), event_code) !=
           replay_action_event_codes_.end();
}

void ObstacleNavNode::process_replay_event_queue() {
    if (replay_pending_event_indices_.empty()) {
        return;
    }
    const auto now = this->now();
    if (replay_last_event_pub_time_.nanoseconds() > 0 &&
        (now - replay_last_event_pub_time_).seconds() < replay_event_min_interval_s_) {
        return;
    }
    const std::size_t event_index = replay_pending_event_indices_.front();
    replay_pending_event_indices_.pop_front();
    if (event_index >= replay_events_.size()) {
        return;
    }
    const auto& event = replay_events_[event_index];
    if (event.event_code == 9) {
        replay_current_policy_ = 0;
    } else if (event.event_code == 10) {
        replay_current_policy_ = 1;
    } else if (event.event_code == 11) {
        replay_current_policy_ = 2;
    } else if (event.event_code == 21) {
        replay_current_policy_ = 3;
    }
    publish_replay_event_code(event.event_code);
    const double pause_s = replay_event_pause_duration(event.event_code);
    if (pause_s > 0.0) {
        replay_pause_until_time_ = now + rclcpp::Duration::from_seconds(pause_s);
        stop_robot();
    }
    RCLCPP_INFO(this->get_logger(), "Replay event fired: id=%d name=%s code=%d s=%.3f",
                event.event_id, event.event_name.c_str(), event.event_code, event.s);
}

void ObstacleNavNode::publish_replay_event_code(int event_code) {
    std_msgs::msg::Int8 msg;
    msg.data = static_cast<int8_t>(std::clamp(event_code, -128, 127));
    cmd_evt_pub_->publish(msg);
    replay_last_event_pub_time_ = this->now();
}

void ObstacleNavNode::start_replay_action_sequence(std::size_t sequence_index, const rclcpp::Time& now) {
    if (sequence_index >= replay_action_sequences_.size()) {
        return;
    }
    replay_action_sequence_active_ = true;
    replay_action_finish_sent_ = false;
    replay_action_hold_done_ = false;
    replay_active_action_sequence_index_ = sequence_index;
    replay_active_action_step_index_ = 0;
    replay_action_last_event_code_ = -1;
    replay_action_sequence_start_time_ = now;
    replay_action_last_step_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    replay_action_next_finish_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    replay_action_next_recovery_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    stop_robot();

    const auto& sequence = replay_action_sequences_[sequence_index];
    RCLCPP_INFO(this->get_logger(),
                "Replay action sequence start: id=%d trigger_s=%.3f steps=%zu duration=%.2fs",
                sequence.sequence_id, sequence.trigger_s, sequence.steps.size(), sequence.duration_s);
}

void ObstacleNavNode::process_replay_action_sequence(const rclcpp::Time& now) {
    if (!replay_action_sequence_active_) {
        return;
    }
    if (replay_active_action_sequence_index_ >= replay_action_sequences_.size()) {
        replay_action_sequence_active_ = false;
        return;
    }

    auto& sequence = replay_action_sequences_[replay_active_action_sequence_index_];
    const double elapsed = (now - replay_action_sequence_start_time_).seconds();
    while (replay_active_action_step_index_ < sequence.steps.size() &&
           elapsed + 1e-6 >= sequence.steps[replay_active_action_step_index_].delay_s) {
        const auto& step = sequence.steps[replay_active_action_step_index_];
        if (step.event_code == 9) {
            replay_current_policy_ = 0;
        } else if (step.event_code == 10) {
            replay_current_policy_ = 1;
        } else if (step.event_code == 11) {
            replay_current_policy_ = 2;
        } else if (step.event_code == 21) {
            replay_current_policy_ = 3;
        }
        publish_replay_event_code(step.event_code);
        RCLCPP_INFO(this->get_logger(),
                    "Replay action sequence step: id=%d step=%zu/%zu code=%d name=%s delay=%.2fs",
                    sequence.sequence_id, replay_active_action_step_index_ + 1, sequence.steps.size(),
                    step.event_code, step.event_name.c_str(), step.delay_s);
        replay_action_last_event_code_ = step.event_code;
        replay_action_last_step_time_ = now;
        replay_action_hold_done_ = false;
        replay_active_action_step_index_++;
    }

    if (replay_active_action_step_index_ < sequence.steps.size()) {
        stop_robot();
        return;
    }

    const double hold_s = replay_action_hold_duration(replay_action_last_event_code_);
    if (!replay_action_hold_done_ && hold_s > 0.0 && replay_action_last_step_time_.nanoseconds() > 0) {
        const auto hold_until = replay_action_last_step_time_ + rclcpp::Duration::from_seconds(hold_s);
        if (now < hold_until) {
            stop_robot();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "Replay holding last action: id=%d code=%d remain=%.2fs lower_state=%d",
                sequence.sequence_id, replay_action_last_event_code_, (hold_until - now).seconds(), replay_lower_state_);
            return;
        }
        replay_action_hold_done_ = true;
    }

    if (replay_action_finish_enter_rl_) {
        process_replay_action_recovery(now, sequence);
        return;
    }

    replay_action_sequence_active_ = false;
    replay_action_finish_sent_ = false;
    replay_action_hold_done_ = false;
    replay_action_next_finish_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    replay_action_next_recovery_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    RCLCPP_INFO(this->get_logger(), "Replay action sequence done: id=%d lower_state=%d.",
                sequence.sequence_id, replay_lower_state_);
}

double ObstacleNavNode::replay_action_hold_duration(int event_code) const {
    for (std::size_t i = 0; i < replay_action_hold_codes_.size(); ++i) {
        if (static_cast<int>(replay_action_hold_codes_[i]) == event_code) {
            return std::max(0.0, replay_action_hold_durations_[i]);
        }
    }
    return 0.0;
}

void ObstacleNavNode::process_replay_action_recovery(const rclcpp::Time& now, const ReplayActionSequence& sequence) {
    constexpr int kFixedDownState = 2;
    constexpr int kFixedStandState = 3;
    constexpr int kRlMoveState = 5;
    constexpr int kJumpState = 6;
    constexpr int kStrideState = 7;
    constexpr int kSmallJumpState = 8;
    constexpr int kKneelCrawlState = 9;
    constexpr int kUpDownEvent = 2;
    constexpr int kEnterRlEvent = 4;

    if (replay_lower_state_ == kRlMoveState) {
        replay_action_sequence_active_ = false;
        replay_action_finish_sent_ = false;
        replay_action_hold_done_ = false;
        replay_action_next_finish_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
        replay_action_next_recovery_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
        RCLCPP_INFO(this->get_logger(), "Replay action sequence done: id=%d lower_state=%d.",
                    sequence.sequence_id, replay_lower_state_);
        return;
    }

    if (replay_action_next_recovery_time_.nanoseconds() > 0 && now < replay_action_next_recovery_time_) {
        stop_robot();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Replay recovering action sequence: id=%d lower_state=%d",
            sequence.sequence_id, replay_lower_state_);
        return;
    }

    int next_event = kEnterRlEvent;
    const char* reason = "enter_rl";
    if (replay_lower_state_ == kJumpState || replay_lower_state_ == kStrideState ||
        replay_lower_state_ == kSmallJumpState || replay_lower_state_ == kKneelCrawlState) {
        next_event = kUpDownEvent;
        reason = "action_to_fixed_down";
    } else if (replay_lower_state_ == kFixedDownState) {
        next_event = kUpDownEvent;
        reason = "fixed_down_to_fixed_stand";
    } else if (replay_lower_state_ == kFixedStandState) {
        next_event = kEnterRlEvent;
        reason = "fixed_stand_to_rl";
    }

    publish_replay_event_code(next_event);
    replay_action_finish_sent_ = true;
    replay_action_next_finish_time_ = now + rclcpp::Duration::from_seconds(std::max(0.0, replay_action_finish_wait_s_));
    replay_action_next_recovery_time_ = now + rclcpp::Duration::from_seconds(std::max(0.05, replay_action_recovery_interval_s_));
    stop_robot();
    RCLCPP_INFO(this->get_logger(),
                "Replay action sequence recovery: id=%d send code=%d reason=%s lower_state=%d",
                sequence.sequence_id, next_event, reason, replay_lower_state_);
}

double ObstacleNavNode::replay_event_pause_duration(int event_code) const {
    for (std::size_t i = 0; i < replay_event_pause_codes_.size(); ++i) {
        if (static_cast<int>(replay_event_pause_codes_[i]) == event_code) {
            return std::max(0.0, replay_event_pause_durations_[i]);
        }
    }
    return 0.0;
}

void ObstacleNavNode::publish_replay_velocity(const geometry_msgs::msg::Twist& cmd) {
    geometry_msgs::msg::Twist output_cmd = cmd;
    double policy_speed_scale = replay_default_speed_scale_;
    if (replay_current_policy_ == 2) {  // POLICY_UPSTAIR
        policy_speed_scale = replay_upstair_speed_scale_;
    } else if (replay_current_policy_ == 3) {  // POLICY_KNEEL_CRAWL
        policy_speed_scale = replay_kneel_crawl_speed_scale_;
    }
    output_cmd.linear.x *= policy_speed_scale;
    output_cmd.linear.y *= policy_speed_scale;
    if (replay_current_policy_ == 1) {  // POLICY_CREEP
        output_cmd.angular.z = 0.0;
    }
    cmd_vel_pub_->publish(output_cmd);
    if (!replay_publish_high_command_ || replay_action_sequence_active_) {
        return;
    }
    quad::msg::HighCommands high_cmd;
    high_cmd.command = 5;  // RL_MOVE
    high_cmd.policy_switch = static_cast<int8_t>(replay_current_policy_);
    high_cmd.lin_x = output_cmd.linear.x * 1.5;
    high_cmd.lin_y = std::clamp(output_cmd.linear.y * 0.8, -0.8, 0.8);
    high_cmd.ang_yaw = output_cmd.angular.z * 2.0;
    high_command_pub_->publish(high_cmd);
}

void ObstacleNavNode::reset_rewind_events(double rewind_target_s, double rewind_from_s) {
    const double begin_s = std::min(rewind_target_s, rewind_from_s);
    const double end_s = std::max(rewind_target_s, rewind_from_s);
    for (auto& event : replay_events_) {
        if (event.repeat_on_rewind && event.s >= begin_s && event.s <= end_s) {
            event.triggered = false;
            RCLCPP_INFO(this->get_logger(), "Replay event reset after rewind: id=%d name=%s s=%.3f",
                        event.event_id, event.event_name.c_str(), event.s);
        }
    }
    for (auto& sequence : replay_action_sequences_) {
        if (sequence.repeat_on_rewind && sequence.trigger_s >= begin_s && sequence.trigger_s <= end_s) {
            sequence.triggered = false;
            RCLCPP_INFO(this->get_logger(), "Replay action sequence reset after rewind: id=%d s=%.3f",
                        sequence.sequence_id, sequence.trigger_s);
        }
    }
}

void ObstacleNavNode::update_stuck_history(double current_s) {
    const auto now = this->now();
    replay_progress_history_.push_back({now, current_s});
    while (!replay_progress_history_.empty() &&
           (now - replay_progress_history_.front().first).seconds() > stuck_window_s_) {
        replay_progress_history_.pop_front();
    }
}

bool ObstacleNavNode::has_replay_action_trigger_between(double begin_s, double end_s) const {
    const double lo = std::min(begin_s, end_s);
    const double hi = std::max(begin_s, end_s);
    for (const auto& sequence : replay_action_sequences_) {
        if (!sequence.triggered && sequence.trigger_s >= lo && sequence.trigger_s <= hi) {
            return true;
        }
    }
    for (const auto& event : replay_events_) {
        if (!event.triggered && is_replay_action_event(event.event_code) && event.s >= lo && event.s <= hi) {
            return true;
        }
    }
    return false;
}

bool ObstacleNavNode::should_skip_stalled_forward(
    const geometry_msgs::msg::Twist& cmd, double current_s, double err_x_body, double err_y_body) const {
    if (!replay_skip_stall_enable_ || replay_phase_ != ReplayPhase::FORWARD || replay_path_.empty()) {
        return false;
    }
    if (replay_skip_stall_advance_s_ <= 0.0 || current_s >= replay_path_.back().s - finish_s_tolerance_) {
        return false;
    }
    if (replay_progress_history_.size() < 2) {
        return false;
    }

    const auto now = this->now();
    if ((now - replay_progress_history_.front().first).seconds() < replay_skip_stall_window_s_ * 0.8) {
        return false;
    }

    const double progress = current_s - replay_progress_history_.front().second;
    if (progress >= replay_skip_stall_progress_epsilon_) {
        return false;
    }
    if (std::hypot(err_x_body, err_y_body) > replay_skip_stall_xy_error_) {
        return false;
    }
    if (std::hypot(cmd.linear.x, cmd.linear.y) > replay_skip_stall_cmd_xy_) {
        return false;
    }

    const double skip_to_s = std::min(current_s + replay_skip_stall_advance_s_, replay_path_.back().s);
    return !has_replay_action_trigger_between(current_s, skip_to_s + replay_skip_stall_guard_s_);
}

bool ObstacleNavNode::should_enter_rewind(const geometry_msgs::msg::Twist& cmd, double current_s) const {
    if (replay_current_policy_ != 2) {  // Only POLICY_UPSTAIR uses rewind recovery.
        return false;
    }
    if (rewind_distance_ <= 0.0 || current_s < rewind_distance_ || replay_path_.empty()) {
        return false;
    }
    if (current_s >= replay_path_.back().s - finish_s_tolerance_) {
        return false;
    }
    if (replay_progress_history_.size() < 2) {
        return false;
    }
    const auto now = this->now();
    if (replay_last_rewind_end_time_.nanoseconds() > 0 &&
        (now - replay_last_rewind_end_time_).seconds() < rewind_cooldown_s_) {
        return false;
    }
    if ((now - replay_progress_history_.front().first).seconds() < stuck_window_s_ * 0.8) {
        return false;
    }
    const double commanded = std::hypot(cmd.linear.x, cmd.linear.y);
    if (commanded < 0.05) {
        return false;
    }
    const double progress = std::abs(current_s - replay_progress_history_.front().second);
    return progress < stuck_progress_epsilon_;
}

void ObstacleNavNode::start_rewind(double current_s) {
    replay_phase_ = ReplayPhase::REWIND;
    replay_last_forward_yaw_only_ = false;
    replay_rewind_from_s_ = current_s;
    replay_rewind_target_s_ = std::max(replay_path_.front().s, current_s - rewind_distance_);
    replay_rewind_recover_boost_active_ = false;
    replay_progress_history_.clear();
    RCLCPP_WARN(this->get_logger(), "Replay stuck detected. Rewind from s=%.3f to s=%.3f",
                replay_rewind_from_s_, replay_rewind_target_s_);
}

void ObstacleNavNode::control_record_parku_replay() {
    if (!replay_loaded_ || replay_path_.size() < 2) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "Record parku replay is not loaded.");
        stop_robot();
        return;
    }

    const bool had_previous_s = replay_s_initialized_;
    const double previous_s = replay_last_s_;
    double current_s = 0.0;
    double projection_error = 0.0;
    if (!project_current_pose_to_replay_path(current_s, projection_error)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Record parku replay: failed to project current pose to path.");
        stop_robot();
        return;
    }
    replay_s_initialized_ = true;
    replay_last_s_ = current_s;

    const auto now = this->now();
    process_replay_action_sequence(now);
    if (replay_action_sequence_active_) {
        stop_robot();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Replay holding action sequence, lower_state=%d s=%.2f",
            replay_lower_state_, current_s);
        return;
    }

    if (replay_pause_until_time_.nanoseconds() > 0 && now < replay_pause_until_time_) {
        stop_robot();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Replay waiting for event action/state switch, remain=%.2fs s=%.2f",
            (replay_pause_until_time_ - now).seconds(), current_s);
        return;
    }

    if (replay_phase_ == ReplayPhase::FORWARD) {
        ReplayPathPoint target{};
        const bool yaw_only = (replay_current_policy_ != 1) && find_yaw_in_place_target(current_s, target);
        if (!yaw_only && !find_forward_circle_target(current_s, target)) {
            stop_robot();
            return;
        }
        double err_x_body = 0.0;
        double err_y_body = 0.0;
        double err_yaw = 0.0;
        auto cmd = yaw_only
            ? compute_replay_yaw_only_cmd(target, err_x_body, err_y_body, err_yaw)
            : compute_replay_cmd(target, err_x_body, err_y_body, err_yaw);
        const bool recover_boost = replay_rewind_recover_boost_active_ &&
            current_s < replay_rewind_from_s_ - replay_min_forward_s_;
        if (recover_boost) {
            const double scale = std::max(1.0, replay_rewind_recover_speed_scale_);
            cmd.linear.x *= scale;
            cmd.linear.y *= scale;
        } else if (replay_rewind_recover_boost_active_) {
            replay_rewind_recover_boost_active_ = false;
            RCLCPP_INFO(this->get_logger(), "Replay rewind recovery boost done at s=%.3f.", current_s);
        }
        publish_replay_target_debug(target, err_x_body, err_y_body, err_yaw);
        trigger_forward_events(had_previous_s ? previous_s : current_s, current_s);
        process_replay_event_queue();
        if (trigger_forward_action_sequences(had_previous_s ? previous_s : current_s, current_s)) {
            return;
        }
        if (replay_last_forward_yaw_only_ && !yaw_only) {
            replay_progress_history_.clear();
            RCLCPP_INFO(this->get_logger(), "Replay left YAW_ONLY; cleared stuck history at s=%.3f.", current_s);
        }
        replay_last_forward_yaw_only_ = yaw_only;
        if (!yaw_only) {
            update_stuck_history(current_s);
        }

        const double finish_xy = std::hypot(replay_path_.back().x - x_, replay_path_.back().y - y_);
        const double finish_yaw = std::abs(normalize_angle(replay_path_.back().yaw - yaw_));
        if (current_s >= replay_path_.back().s - finish_s_tolerance_ &&
            finish_xy <= finish_xy_tolerance_ &&
            finish_yaw <= finish_yaw_tolerance_) {
            stop_robot();
            publish_nav_status_once();
            publish_nav_finished_once();
            runtime_state_ = RuntimeState::FINISHED;
            RCLCPP_INFO(this->get_logger(), "Record parku replay finished at s=%.3f.", current_s);
            return;
        }

        if (!yaw_only && should_skip_stalled_forward(cmd, current_s, err_x_body, err_y_body)) {
            const double skip_to_s = std::min(current_s + replay_skip_stall_advance_s_, replay_path_.back().s);
            replay_skip_min_s_ = std::max(replay_skip_min_s_, skip_to_s);
            replay_last_s_ = replay_skip_min_s_;
            replay_progress_history_.clear();
            RCLCPP_WARN(this->get_logger(),
                        "Replay small-error stall skipped: s=%.3f -> %.3f err_xy=%.3f cmd_xy=%.3f",
                        current_s, replay_skip_min_s_,
                        std::hypot(err_x_body, err_y_body),
                        std::hypot(cmd.linear.x, cmd.linear.y));
            return;
        }

        if (should_enter_rewind(cmd, current_s)) {
            start_rewind(current_s);
            return;
        }

        publish_replay_velocity(cmd);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Replay FORWARD%s%s s=%.2f/%.2f proj=%.2f target_s=%.2f err_body=(%.2f,%.2f,%.2f) cmd=(%.2f,%.2f,%.2f)",
            yaw_only ? " YAW_ONLY" : "",
            recover_boost ? " BOOST" : "",
            current_s, replay_path_.back().s, projection_error, target.s,
            err_x_body, err_y_body, err_yaw, cmd.linear.x, cmd.linear.y, cmd.angular.z);
        return;
    }

    ReplayPathPoint rewind_target{};
    const double target_s = std::max(replay_rewind_target_s_, current_s - replay_fallback_lookahead_);
    if (!sample_replay_path(target_s, rewind_target)) {
        stop_robot();
        return;
    }
    double err_x_body = 0.0;
    double err_y_body = 0.0;
    double err_yaw = 0.0;
    auto cmd = compute_replay_cmd(rewind_target, err_x_body, err_y_body, err_yaw);
    const double rewind_scale = std::max(1.0, replay_rewind_speed_scale_);
    cmd.linear.x *= rewind_scale;
    cmd.linear.y *= rewind_scale;
    publish_replay_target_debug(rewind_target, err_x_body, err_y_body, err_yaw);
    publish_replay_velocity(cmd);

    if (current_s <= replay_rewind_target_s_ + finish_s_tolerance_) {
        reset_rewind_events(replay_rewind_target_s_, replay_rewind_from_s_);
        replay_phase_ = ReplayPhase::FORWARD;
        replay_last_forward_yaw_only_ = false;
        replay_rewind_recover_boost_active_ = replay_rewind_recover_speed_scale_ > 1.0;
        replay_progress_history_.clear();
        replay_last_rewind_end_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Replay rewind done at s=%.3f. Resume forward replay%s.",
                    current_s, replay_rewind_recover_boost_active_ ? " with recovery boost" : "");
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "Replay REWIND s=%.2f target_s=%.2f err_body=(%.2f,%.2f,%.2f) cmd=(%.2f,%.2f,%.2f)",
        current_s, rewind_target.s, err_x_body, err_y_body, err_yaw,
        cmd.linear.x, cmd.linear.y, cmd.angular.z);
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

    if (!got_tf_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "ObstacleNav: waiting for TF %s -> %s",
                             target_frame_.c_str(), child_frame_.c_str());
        return;
    }

    if (stop_on_lidar_drift("ObstacleNav")) {
        return;
    }

    if (nav_mode_ == NavMode::RECORD_PARKU_REPLAY) {
        control_record_parku_replay();
        return;
    }

    if (try_advance_past_empty_path()) {
        return;
    }

    const auto& target = current_path_[current_path_index_];
    const WaypointTrackingErrors errors = compute_waypoint_errors(target);

    publish_debug(target, errors.err_x_body, errors.err_y_body, errors.err_yaw);

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
    if (replay_action_sequence_active_) {
        return;
    }
    if (replay_publish_high_command_ && high_command_pub_) {
        quad::msg::HighCommands high_cmd;
        high_cmd.command = 5;
        high_cmd.policy_switch = static_cast<int8_t>(replay_current_policy_);
        high_cmd.lin_x = 0.0;
        high_cmd.lin_y = 0.0;
        high_cmd.ang_yaw = 0.0;
        high_command_pub_->publish(high_cmd);
    }
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
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleNavNode>());
    rclcpp::shutdown();
    return 0;
}
