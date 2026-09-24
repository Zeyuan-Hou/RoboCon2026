#ifndef OBSTACLE_NAV_NODE_HPP_
#define OBSTACLE_NAV_NODE_HPP_

#include <cstddef>
#include <deque>
#include <cstdint>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "quad/msg/high_commands.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_msgs/msg/tf_message.hpp"

struct ObstacleNavPoint {
    double x;
    double y;
    double yaw;
};

struct ReplayPathPoint {
    double s;
    double x;
    double y;
    double yaw;
};

struct ReplayEvent {
    int event_id;
    double s;
    double x;
    double y;
    double yaw;
    int event_code;
    std::string event_name;
    bool repeat_on_rewind;
    bool triggered;
};

struct ReplayActionStep {
    double delay_s;
    int event_code;
    std::string event_name;
};

struct ReplayActionSequence {
    int sequence_id;
    double trigger_s;
    double trigger_x;
    double trigger_y;
    double trigger_yaw;
    double start_time;
    double end_time;
    double duration_s;
    std::vector<ReplayActionStep> steps;
    bool repeat_on_rewind;
    bool triggered;
};

struct WaypointTrackingErrors {
    double dist;
    double err_x_body;
    double err_y_body;
    double err_yaw;
};

class ObstacleNavNode : public rclcpp::Node {
public:
    ObstacleNavNode();

private:
    enum class NavMode {
        POINT_SEQUENCE,
        TRAJECTORY_SEQUENCE,
        RECORD_PARKU_REPLAY
    };

    enum class ReplayPhase {
        FORWARD,
        REWIND
    };

    enum class RuntimeState {
        IDLE,
        RUNNING,
        WAIT_TASK,
        WAIT_NEXT_ENABLE,
        FINISHED
    };

    // --- 参数与目标加载 ---
    void load_parameters();
    void load_point_sequence();
    bool load_trajectory_file(const std::string& file_path, std::vector<ObstacleNavPoint>& out_points);
    bool load_replay_path_file();
    bool load_replay_events_file();
    bool load_replay_action_sequences_file();
    bool load_current_target();
    std::size_t total_target_count() const;

    // --- ROS 接口 ---
    void setup_ros_interfaces();
    void nav_enable_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void state_array_callback(const std_msgs::msg::Int8MultiArray::SharedPtr msg);
    void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    void control_loop();

    // --- nav_enable 处理 ---
    void on_nav_enable_disabled(bool previous_enabled);
    void on_nav_enable_rising_edge();

    // --- 控制循环子步骤 ---
    void publish_current_pose_debug();
    void handle_finished_while_enabled();
    bool try_advance_past_empty_path();
    WaypointTrackingErrors compute_waypoint_errors(const ObstacleNavPoint& target) const;
    bool is_waypoint_reached(double dist, double err_yaw) const;
    void on_waypoint_reached(const ObstacleNavPoint& target);
    void compute_pd_velocity_cmd(const WaypointTrackingErrors& errors, double dist);
    void control_record_parku_replay();
    void reset_replay_state();

    // --- 工具 ---
    bool stop_on_lidar_drift(const char* context);
    double normalize_angle(double angle) const;
    void map_error_to_body(double err_x_map, double err_y_map, double& err_x_body, double& err_y_body) const;
    bool project_current_pose_to_replay_path(double& out_s, double& out_distance);
    bool sample_replay_path(double s, ReplayPathPoint& out_point) const;
    bool find_forward_circle_target(double current_s, ReplayPathPoint& out_target) const;
    bool find_forward_fallback_target(double current_s, ReplayPathPoint& out_target) const;
    bool find_yaw_in_place_target(double current_s, ReplayPathPoint& out_target) const;
    void publish_replay_target_debug(const ReplayPathPoint& target, double err_x_body, double err_y_body, double err_yaw);
    void trigger_forward_events(double previous_s, double current_s);
    bool trigger_forward_action_sequences(double previous_s, double current_s);
    bool is_replay_event_blocked(int event_code) const;
    bool is_replay_action_event(int event_code) const;
    void process_replay_event_queue();
    double replay_event_pause_duration(int event_code) const;
    void publish_replay_event_code(int event_code);
    void start_replay_action_sequence(std::size_t sequence_index, const rclcpp::Time& now);
    void process_replay_action_sequence(const rclcpp::Time& now);
    double replay_action_hold_duration(int event_code) const;
    void process_replay_action_recovery(const rclcpp::Time& now, const ReplayActionSequence& sequence);
    void reset_rewind_events(double rewind_target_s, double rewind_from_s);
    void update_stuck_history(double current_s);
    bool has_replay_action_trigger_between(double begin_s, double end_s) const;
    bool should_skip_stalled_forward(
        const geometry_msgs::msg::Twist& cmd, double current_s, double err_x_body, double err_y_body) const;
    bool should_enter_rewind(const geometry_msgs::msg::Twist& cmd, double current_s) const;
    geometry_msgs::msg::Twist compute_replay_cmd(const ReplayPathPoint& target, double& err_x_body, double& err_y_body, double& err_yaw) const;
    geometry_msgs::msg::Twist compute_replay_yaw_only_cmd(const ReplayPathPoint& target, double& err_x_body, double& err_y_body, double& err_yaw) const;
    void publish_replay_velocity(const geometry_msgs::msg::Twist& cmd);
    void start_rewind(double current_s);
    void stop_robot();
    void publish_nav_status_once();
    void publish_nav_finished_once();
    void publish_debug(const ObstacleNavPoint& target, double err_x_body, double err_y_body, double err_yaw);
    void reset_pd_state();

    // Parameters
    std::string target_frame_;
    std::string child_frame_;
    std::string tf_topic_;
    NavMode nav_mode_;
    std::string navigation_mode_param_;

    std::vector<double> target_points_flat_;
    std::vector<std::string> trajectory_files_;

    double position_tolerance_;
    double yaw_tolerance_;
    double kp_x_;
    double kp_y_;
    double kp_yaw_;
    double kd_x_;
    double kd_y_;
    double kd_yaw_;
    double max_vx_;
    double max_vy_;
    double max_dyaw_;
    double yaw_first_threshold_;
    double control_rate_;

    std::string record_parku_replay_path_;
    std::string record_parku_events_path_;
    std::string record_parku_action_sequences_path_;
    double lidar_offset_x_;
    double lidar_offset_y_;
    double replay_lookahead_radius_;
    double replay_intersection_search_dist_;
    double replay_fallback_lookahead_;
    double replay_min_forward_s_;
    double replay_min_target_xy_;
    double replay_yaw_in_place_search_s_;
    double replay_yaw_in_place_xy_thresh_;
    double replay_yaw_in_place_yaw_thresh_;
    double replay_projection_back_dist_;
    double replay_projection_forward_dist_;
    double replay_projection_yaw_weight_;
    double replay_kp_forward_;
    double replay_kp_lateral_;
    double replay_lateral_trim_gain_;
    double replay_kp_yaw_;
    double replay_max_vx_;
    double replay_max_vy_;
    double replay_max_wz_;
    double replay_min_vx_;
    double replay_min_vy_;
    double replay_deadzone_error_x_;
    double replay_deadzone_error_y_;
    double stuck_window_s_;
    double stuck_progress_epsilon_;
    double rewind_distance_;
    double rewind_cooldown_s_;
    double replay_rewind_speed_scale_;
    double replay_rewind_recover_speed_scale_;
    double replay_default_speed_scale_;
    double replay_upstair_speed_scale_;
    double replay_kneel_crawl_speed_scale_;
    double finish_s_tolerance_;
    double finish_xy_tolerance_;
    double finish_yaw_tolerance_;
    double replay_event_min_interval_s_;
    std::vector<int64_t> replay_event_blocklist_;
    std::vector<int64_t> replay_event_pause_codes_;
    std::vector<double> replay_event_pause_durations_;
    bool replay_action_sequence_enable_;
    std::vector<int64_t> replay_action_event_codes_;
    double replay_action_group_max_dt_;
    double replay_action_group_max_xy_;
    double replay_action_group_max_ds_;
    bool replay_action_finish_enter_rl_;
    double replay_action_finish_wait_s_;
    std::vector<int64_t> replay_action_hold_codes_;
    std::vector<double> replay_action_hold_durations_;
    double replay_action_recovery_interval_s_;
    bool replay_skip_stall_enable_;
    double replay_skip_stall_window_s_;
    double replay_skip_stall_progress_epsilon_;
    double replay_skip_stall_xy_error_;
    double replay_skip_stall_cmd_xy_;
    double replay_skip_stall_advance_s_;
    double replay_skip_stall_guard_s_;
    bool replay_publish_high_command_;

    // Runtime state
    bool nav_enabled_ = false;
    bool got_tf_ = false;
    bool nav_status_sent_ = false;
    bool nav_finished_sent_ = false;
    RuntimeState runtime_state_ = RuntimeState::IDLE;

    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;

    std::vector<ObstacleNavPoint> point_targets_;
    std::vector<ObstacleNavPoint> current_path_;
    std::size_t current_target_index_ = 0;
    std::size_t current_path_index_ = 0;

    std::vector<ReplayPathPoint> replay_path_;
    std::vector<ReplayEvent> replay_events_;
    std::vector<ReplayActionSequence> replay_action_sequences_;
    ReplayPhase replay_phase_ = ReplayPhase::FORWARD;
    bool replay_loaded_ = false;
    bool replay_s_initialized_ = false;
    bool replay_last_forward_yaw_only_ = false;
    double replay_last_s_ = 0.0;
    double replay_skip_min_s_ = 0.0;
    double replay_rewind_from_s_ = 0.0;
    double replay_rewind_target_s_ = 0.0;
    bool replay_rewind_recover_boost_active_ = false;
    rclcpp::Time replay_last_rewind_end_time_;
    rclcpp::Time replay_last_event_pub_time_;
    rclcpp::Time replay_pause_until_time_;
    rclcpp::Time replay_action_sequence_start_time_;
    rclcpp::Time replay_action_last_step_time_;
    rclcpp::Time replay_action_next_finish_time_;
    rclcpp::Time replay_action_next_recovery_time_;
    std::deque<std::pair<rclcpp::Time, double>> replay_progress_history_;
    std::deque<std::size_t> replay_pending_event_indices_;
    int replay_current_policy_ = 0;
    int replay_lower_state_ = -1;
    bool replay_action_sequence_active_ = false;
    bool replay_action_finish_sent_ = false;
    bool replay_action_hold_done_ = false;
    std::size_t replay_active_action_sequence_index_ = 0;
    std::size_t replay_active_action_step_index_ = 0;
    int replay_action_last_event_code_ = -1;

    bool pd_initialized_ = false;
    double prev_err_x_body_ = 0.0;
    double prev_err_y_body_ = 0.0;
    double prev_err_yaw_ = 0.0;
    rclcpp::Time prev_ctrl_time_;

    // ROS interfaces
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nav_enable_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr state_array_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<quad::msg::HighCommands>::SharedPtr high_command_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr cmd_evt_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr nav_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr nav_finished_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr current_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr current_target_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr debug_error_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // OBSTACLE_NAV_NODE_HPP_
