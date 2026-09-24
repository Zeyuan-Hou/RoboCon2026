#ifndef OBSTACLE_NAV_NODE_HPP_
#define OBSTACLE_NAV_NODE_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_msgs/msg/tf_message.hpp"

#ifdef TASK_MANAGER_HAS_NAV2_MSGS
#include "nav2_msgs/action/navigate_to_pose.hpp"
#endif

struct ObstacleNavPoint {
    double x;
    double y;
    double yaw;
};

struct WaypointTrackingErrors {
    double dist;
    double err_x_body;
    double err_y_body;
    double err_yaw;
    double err_approach_yaw;
};

class ObstacleNavNode : public rclcpp::Node {
public:
    ObstacleNavNode();

private:
#ifdef TASK_MANAGER_HAS_NAV2_MSGS
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
#endif

    enum class NavigationBackend {
        LOCAL_PD,
        NAV2
    };

    enum class NavMode {
        POINT_SEQUENCE,
        TRAJECTORY_SEQUENCE,
        EXTERNAL_TARGET
    };

    enum class RuntimeState {
        IDLE,
        RUNNING,
        WAIT_TASK,
        WAIT_NEXT_ENABLE,
        FINISHED
    };

    enum class YawThenForwardPhase {
        ALIGN_YAW,
        FORWARD
    };

    // --- 参数与目标加载 ---
    void load_parameters();
    void load_point_sequence();
    bool load_trajectory_file(const std::string& file_path, std::vector<ObstacleNavPoint>& out_points);
    bool load_current_target();
    std::size_t total_target_count() const;

    // --- ROS 接口 ---
    void setup_ros_interfaces();
    void nav_enable_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void external_target_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void sequence_slot_callback(const std_msgs::msg::String::SharedPtr msg);
    void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    void control_loop();

    // --- nav_enable 处理 ---
    void on_nav_enable_disabled(bool previous_enabled);
    void on_nav_enable_rising_edge();

    // --- Nav2 后端 ---
    bool using_nav2_backend() const;
    void send_nav2_goal_for_current_point();
    void cancel_nav2_goal();
#ifdef TASK_MANAGER_HAS_NAV2_MSGS
    void nav2_goal_response_callback(
        int64_t goal_request_id,
        const GoalHandleNavigateToPose::SharedPtr& goal_handle);
    void nav2_result_callback(
        int64_t goal_request_id,
        const GoalHandleNavigateToPose::WrappedResult& result);
#endif
    void handle_nav2_control_loop();
    geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw) const;

    // --- 控制循环子步骤 ---
    void publish_current_pose_debug();
    void handle_finished_while_enabled();
    bool try_advance_past_empty_path();
    WaypointTrackingErrors compute_waypoint_errors(const ObstacleNavPoint& target) const;
    bool is_waypoint_reached(double dist, double err_yaw) const;
    void on_waypoint_reached(const ObstacleNavPoint& target);
    void compute_pd_velocity_cmd(const WaypointTrackingErrors& errors, double dist);
    bool handle_yaw_then_forward_control(
        const ObstacleNavPoint& target,
        const WaypointTrackingErrors& errors);

    // --- 工具 ---
    bool stop_on_lidar_drift(const char* context);
    double normalize_angle(double angle) const;
    bool yaw_then_forward_enabled_for_current_slot() const;
    double slew_yaw_then_forward_wz(double target_wz);
    void reset_yaw_then_forward_state();
    void stop_robot();
    void publish_nav_status_once();
    void publish_nav_finished_once();
    void publish_debug(const ObstacleNavPoint& target, double err_x_body, double err_y_body, double err_yaw);
    void reset_pd_state();

    // Parameters
    std::string target_frame_;
    std::string child_frame_;
    std::string tf_topic_;
    NavigationBackend navigation_backend_{NavigationBackend::LOCAL_PD};
    std::string navigation_backend_param_;
    NavMode nav_mode_;
    std::string navigation_mode_param_;
    std::string nav2_action_name_;
    std::string goal_frame_id_;
    double nav2_action_server_wait_s_{0.1};
    double nav2_retry_period_s_{0.5};

    std::vector<double> target_points_flat_;
    std::vector<std::string> trajectory_files_;
    ObstacleNavPoint external_target_{};
    int external_target_index_ = -1;
    bool external_target_received_ = false;

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
    double yaw_only_threshold_;
    double yaw_only_max_dyaw_;
    double yaw_only_slowdown_threshold_;
    double yaw_cmd_slew_rate_;
    double long_straight_distance_threshold_;
    double long_straight_yaw_threshold_;
    double long_straight_kp_x_;
    double long_straight_max_vx_;
    double final_yaw_align_distance_;
    double approach_yaw_max_dyaw_;
    std::vector<std::string> yaw_then_forward_enabled_slots_;
    double yaw_then_forward_align_yaw_tolerance_{0.08};
    double yaw_then_forward_finish_yaw_tolerance_{0.10};
    double yaw_then_forward_forward_tolerance_{0.10};
    double yaw_then_forward_lateral_tolerance_{0.30};
    double yaw_then_forward_kp_yaw_{1.20};
    double yaw_then_forward_max_wz_{0.75};
    double yaw_then_forward_wz_slew_rate_{1.0};
    double yaw_then_forward_kp_forward_{0.80};
    double yaw_then_forward_max_vx_{0.40};
    double yaw_then_forward_kp_lateral_{0.25};
    double yaw_then_forward_max_vy_{0.08};
    double control_rate_;

    // Runtime state
    bool nav_enabled_ = false;
    bool got_tf_ = false;
    bool nav_status_sent_ = false;
    bool nav_finished_sent_ = false;
    RuntimeState runtime_state_ = RuntimeState::IDLE;
    std::string active_sequence_slot_;
    YawThenForwardPhase yaw_then_forward_phase_{YawThenForwardPhase::ALIGN_YAW};
    bool yaw_then_forward_initialized_ = false;
    double yaw_then_forward_prev_wz_ = 0.0;
    rclcpp::Time yaw_then_forward_prev_ctrl_time_;

    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;

    std::vector<ObstacleNavPoint> point_targets_;
    std::vector<ObstacleNavPoint> current_path_;
    std::size_t current_target_index_ = 0;
    std::size_t current_path_index_ = 0;

    bool pd_initialized_ = false;
    double prev_err_x_body_ = 0.0;
    double prev_err_y_body_ = 0.0;
    double prev_err_yaw_ = 0.0;
    double prev_cmd_wz_ = 0.0;
    rclcpp::Time prev_ctrl_time_;

    bool nav2_goal_in_flight_ = false;
    bool nav2_goal_accepted_ = false;
    bool nav2_goal_pending_retry_ = false;
    int64_t nav2_goal_request_id_ = 0;
    int64_t active_nav2_goal_request_id_ = 0;
    rclcpp::Time nav2_last_send_attempt_time_;
#ifdef TASK_MANAGER_HAS_NAV2_MSGS
    GoalHandleNavigateToPose::SharedPtr nav2_active_goal_;
#endif

    // ROS interfaces
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nav_enable_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr external_target_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sequence_slot_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr nav_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr nav_finished_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr current_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr current_target_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr debug_error_pub_;
#ifdef TASK_MANAGER_HAS_NAV2_MSGS
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_action_client_;
#endif

    rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // OBSTACLE_NAV_NODE_HPP_
