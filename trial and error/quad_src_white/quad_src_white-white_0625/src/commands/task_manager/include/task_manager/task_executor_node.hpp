#ifndef TASK_EXECUTOR_NODE_HPP_
#define TASK_EXECUTOR_NODE_HPP_

#include <string>
#include <map>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "quad/msg/qr_result.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

/**
 * @brief 视觉对准的目标偏移量结构体
 */
struct TaskOffset {
    double target_qr_x;    // QR视觉系：x向右为正，单位 mm
    double target_qr_y;    // QR视觉系：y向前为正，单位 mm
    double target_qr_yaw;  // QR视觉系：yaw顺时针为正，单位 deg
};

struct TrajectoryPoint {
    double t;
    double x;
    double y;
    double yaw;
    double s;
};

struct VisualBodyErrors {
    double err_x_m;
    double err_y_m;
    double err_yaw_rad;
    double err_qr_yaw_deg;
};

struct BridgeProgressErrors {
    double dx_ideal_lidar;
    double dy_ideal_lidar;
    double remain_dist;
    double err_yaw;
};

struct BodyDelta {
    double forward;
    double lateral;
};

struct StairProgress {
    double elapsed;
    double forward_progress;
    double lateral_error;
    double err_yaw;
};

struct CrawlProgress {
    double forward_progress;
    double lateral_error;
    double err_yaw;
};

struct PoleBodyErrors {
    double err_x_body;
    double err_y_body;
    double err_yaw;
};

struct LineProfile {
    double target_dist;
    double max_speed;
    double kp_y;
    double kp_xy;
    double kp_yaw;
    double max_vy;
    double max_dyaw;
};

struct PathProfile {
    std::string trajectory_file;
    std::string drive_mode;
    bool qr_assist_enabled;
    bool qr_assist_required;
    double qr_assist_timeout_s;
    double qr_assist_kp_y;
    double qr_assist_kp_yaw;
    double qr_assist_max_vy;
    double qr_assist_max_dyaw;
    std::vector<double> qr_assist_visual_offset;
    double kp_x;
    double kp_y;
    double kp_yaw;
    double kd_x;
    double kd_y;
    double kd_yaw;
    double lin_vel_creep_min;
    double lidar_offset_x;
    double lidar_offset_y;
    double max_vx;
    double max_vy;
    double max_dyaw;
    double lookahead_distance;
    double finish_tolerance;
};

struct ObstacleVisualConfig {
    std::string slot_name;
    int expected_task_id;
    TaskOffset visual_offset;
};

/**
 * @brief 任务执行器节点：负责纯粹的底层闭环速度控制和算法实现
 *
 * 接收来自上层 task_state_machine 的宏观状态指令，
 * 在特定状态下接管 cmd_vel，执行包括：视觉伺服对准、雷达闭环匍匐、雷达高精度过桥等具体任务。
 */
class TaskExecutorNode : public rclcpp::Node {
public:
    TaskExecutorNode();

private:
    enum class ExecutorMode {
        IDLE,
        VISUAL_SERVOING,
        LIDAR_CRAWL,
        BRIDGE_CROSS,
        STAIR_UP_MOVING,
        POLE_AROUND
    };
    ExecutorMode current_mode_ = ExecutorMode::IDLE;

    // --- ROS 接口 ---
    void setup_ros_interfaces();
    void state_callback(const std_msgs::msg::String::SharedPtr msg);
    void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    void qr_callback(const quad::msg::QrResult::SharedPtr msg);
    void sequence_index_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void sequence_segment_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void sequence_slot_callback(const std_msgs::msg::String::SharedPtr msg);

    // --- 模式切换 ---
    void apply_executor_mode_from_state_string(const std::string& state_str);
    void update_pose_from_tf(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    void apply_active_line_profile();
    void apply_active_path_profile();

    // --- 核心控制与任务算法 ---
    void control_loop();
    void execute_visual_servoing();
    void execute_lidar_crawl();
    void execute_bridge_cross();
    void execute_stair_up();
    void execute_pole_around();

    // --- 视觉伺服子步骤 ---
    bool is_qr_data_stale();
    bool get_active_visual_config(int task_id, ObstacleVisualConfig& config);
    VisualBodyErrors compute_visual_body_errors(const TaskOffset& config) const;
    void publish_visual_debug_errors(const VisualBodyErrors& errors) const;
    bool is_visual_aligned(const VisualBodyErrors& errors) const;
    void publish_visual_velocity_cmd(const VisualBodyErrors& errors, const TaskOffset& config, int task_id);

    // --- 匍匐子步骤 ---
    void ensure_crawl_segment_started();
    CrawlProgress compute_crawl_progress() const;
    bool is_crawl_finished(const CrawlProgress& progress) const;
    geometry_msgs::msg::Twist compute_crawl_twist(const CrawlProgress& progress) const;

    // --- 过桥子步骤 ---
    void ensure_bridge_segment_started();
    BridgeProgressErrors compute_bridge_progress_errors() const;
    bool is_bridge_finished(const BridgeProgressErrors& progress) const;
    geometry_msgs::msg::Twist compute_bridge_twist(const BridgeProgressErrors& progress) const;

    // --- 上楼梯子步骤 ---
    void ensure_stair_segment_started();
    StairProgress compute_stair_progress() const;
    bool is_stair_finished(const StairProgress& progress) const;
    geometry_msgs::msg::Twist compute_stair_twist(const StairProgress& progress) const;

    // --- 绕杆子步骤 ---
    void ensure_pole_replay_started();
    PoleBodyErrors compute_pole_body_errors(double ref_x, double ref_y, double ref_yaw) const;
    geometry_msgs::msg::Twist compute_pole_pd_twist(const PoleBodyErrors& errors);
    bool apply_qr_path_assist(geometry_msgs::msg::Twist& cmd, VisualBodyErrors& qr_errors);
    bool check_pole_replay_finished(double closest_s);

    // --- 辅助与工具函数 ---
    bool stop_on_lidar_drift(const char* context);
    BodyDelta compute_body_delta_from_start(double start_x, double start_y, double start_yaw) const;
    void load_parameters();
    double normalize_angle(double angle) const;
    std::string executor_mode_state_name() const;
    void stop_robot();
    void finish_task(int task_id);
    bool load_pole_trajectory();
    bool sample_pole_reference(double& x, double& y, double& yaw, double& closest_s, double& target_s);
    bool sample_pole_reference_at_s(double s, double& x, double& y, double& yaw) const;
    double find_closest_pole_path_s() const;
    std::string active_line_profile_name() const;
    std::string active_path_profile_name() const;
    LineProfile get_line_profile(const std::string& name) const;
    PathProfile get_path_profile(const std::string& name) const;

    // --- ROS 通信接口 ---
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Subscription<quad::msg::QrResult>::SharedPtr qr_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sequence_index_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sequence_segment_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sequence_slot_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr task_done_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr error_debug_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr lidar_pose_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // --- 视觉伺服状态变量 ---
    quad::msg::QrResult latest_qr_data_;
    bool qr_data_received_ = false;
    rclcpp::Time last_qr_time_;
    std::string completed_state_latch_;
    const double DATA_TIMEOUT_THRESHOLD = 0.5;
    TaskOffset default_offset_;
    std::map<std::string, ObstacleVisualConfig> obstacle_visual_configs_by_slot_;
    std::map<int, std::string> obstacle_slot_by_task_id_;
    double visual_tolerance_x_m_;
    double visual_tolerance_y_m_;
    double visual_tolerance_yaw_rad_;
    double visual_kp_x_;
    double visual_kp_y_;
    double visual_kp_yaw_;
    double visual_max_vx_;
    double visual_max_vy_;
    double visual_max_wz_;
    double visual_lateral_cmd_sign_;
    double visual_yaw_error_sign_;

    // --- 匍匐雷达闭环状态变量 ---
    double crawl_target_yaw_;
    bool crawl_use_current_yaw_;
    double crawl_forward_speed_;
    double crawl_kp_y_;
    double crawl_kp_yaw_;
    double crawl_max_vy_;
    double crawl_max_dyaw_;
    double crawl_target_dist_;
    bool crawl_started_ = false;
    bool crawl_finished_ = false;
    double crawl_start_x_ = 0.0;
    double crawl_start_y_ = 0.0;
    double crawl_active_target_yaw_ = 0.0;

    // --- 过木桥(雷达直线)状态变量 ---
    std::string target_frame_;
    std::string child_frame_;
    std::string tf_topic_;
    std::string lidar_pose_topic_;
    double straight_target_yaw_;
    double straight_target_dist_;
    double straight_max_speed_;
    double straight_kp_y_;
    double kp_xy_;
    double kp_yaw_;
    double max_vy_;
    double max_dyaw_;
    std::map<std::string, LineProfile> line_profiles_;
    std::string stair_combo_up_segment_ = "stairs_up";
    std::string stair_combo_down_segment_ = "stairs_down";

    double x_, y_, yaw_;
    bool got_tf_ = false;
    bool straight_started_ = false;
    double straight_start_x_ = 0.0;
    double straight_start_y_ = 0.0;
    double straight_start_yaw_ = 0.0;

    // --- 上楼梯状态变量 ---
    double stair_duration_;
    double stair_target_yaw_;
    bool stair_use_current_yaw_;
    double stair_forward_speed_;
    double stair_kp_y_;
    double stair_kp_yaw_;
    double stair_max_vy_;
    double stair_max_dyaw_;
    double stair_max_distance_;
    bool stair_started_ = false;
    rclcpp::Time stair_start_time_;
    double stair_start_x_ = 0.0;
    double stair_start_y_ = 0.0;
    double stair_active_target_yaw_ = 0.0;

    // --- 绕杆轨迹跟踪状态变量 ---
    std::string pole_trajectory_file_;
    double pole_kp_x_;
    double pole_kp_y_;
    double pole_kp_yaw_;
    double pole_kd_x_;
    double pole_kd_y_;
    double pole_kd_yaw_;
    double pole_lin_vel_creep_min_;
    double pole_lidar_offset_x_;
    double pole_lidar_offset_y_;
    double pole_max_vx_;
    double pole_max_vy_;
    double pole_max_dyaw_;
    std::string pole_drive_mode_;
    bool pole_qr_assist_enabled_ = false;
    bool pole_qr_assist_required_ = false;
    double pole_qr_assist_timeout_s_ = 0.5;
    double pole_qr_assist_kp_y_ = 1.0;
    double pole_qr_assist_kp_yaw_ = 0.45;
    double pole_qr_assist_max_vy_ = 0.25;
    double pole_qr_assist_max_dyaw_ = 0.35;
    std::vector<double> pole_qr_assist_visual_offset_;
    double pole_lookahead_distance_;
    double pole_finish_tolerance_;
    std::map<std::string, PathProfile> path_profiles_;
    std::vector<TrajectoryPoint> pole_trajectory_;
    bool pole_trajectory_loaded_ = false;
    bool pole_replay_started_ = false;
    bool pole_replay_done_ = false;
    rclcpp::Time pole_replay_t0_;
    double pole_path_progress_s_ = 0.0;
    bool pole_pd_initialized_ = false;
    double pole_prev_err_x_body_ = 0.0;
    double pole_prev_err_y_body_ = 0.0;
    double pole_prev_err_yaw_ = 0.0;
    rclcpp::Time pole_prev_ctrl_time_;

    int active_sequence_index_ = -1;
    int active_sequence_segment_ = 0;
    std::string active_sequence_slot_;
    std::string last_visual_slot_;
};

#endif  // TASK_EXECUTOR_NODE_HPP_
