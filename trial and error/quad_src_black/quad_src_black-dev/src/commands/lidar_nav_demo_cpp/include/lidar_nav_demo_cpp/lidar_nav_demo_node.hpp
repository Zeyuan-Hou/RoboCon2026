#ifndef LIDAR_NAV_DEMO_CPP_LIDAR_NAV_DEMO_NODE_HPP_
#define LIDAR_NAV_DEMO_CPP_LIDAR_NAV_DEMO_NODE_HPP_

#include "lidar_nav_demo_cpp/lidar_a_star.hpp"
#include "lidar_nav_demo_cpp/isolation_band.hpp"
#include "lidar_nav_demo_cpp/nav_controller.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <array>
#include <memory>
#include <vector>
#include <string>

struct PickupDropTask {
    size_t eightboxes_string_idx;  // eightboxes 字符串下标 0~7
    size_t pickup_idx;             // pickup_points 物理下标
    size_t dropoff_idx;
    int cargo_class;
    int occurrence;
};

enum class NavMode {
    LOGISTICS_TASK,
    STRAIGHT_LINE,
    AUTO_PARKU
};

enum class LogisticsEventType : int {
    PRE_SCAN_DONE = 0,
    PICKUP_REACHED = 1,
    DROPOFF_REACHED = 2,
    LOGISTICS_FINISHED = 3
};

struct RecordedPose {
    double t;
    double x;
    double y;
    double yaw;
};

struct TargetPoint {
    double x;
    double y;
    double yaw;
    WaypointType type;
    int action_id = -1;
    bool cross_isolation_band = false;
    bool tight_lateral_zone = false;
    bool corridor_lateral_only_arrival = false;
    bool manip_approach_arrival = false;
    bool post_manip_exit = false;
    bool graph_yaw_align_after_arrival = false;
    bool direct_nearest_approach = false;
    bool manip_to_nearest_arrival = false;
};

class LidarNavControl : public rclcpp::Node {
public:
    LidarNavControl();

private:
    // 回调函数
    void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    void control_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void priority_callback(const std_msgs::msg::String::SharedPtr msg);
    void ocr_result_callback(const std_msgs::msg::String::SharedPtr msg);
    void generate_path(const std::vector<int>& slot_classes);
    void try_complete_pre_scan_handshake();
    void try_generate_path_with_pending_priorities(bool allow_without_ocr = false);
    void arm_ocr_wait_after_eightboxes();
    void clear_ocr_wait_after_eightboxes();
    void maybe_generate_path_without_ocr_on_timeout();
    bool pre_scan_entry_point_required() const;
    bool pre_scan_entry_point_ready() const;
    bool control_pre_scan_entry_point();
    std::vector<int> compute_dropoff_indices(const std::vector<int>& classes) const;
    std::vector<PickupDropTask> build_tasks(const std::vector<int>& classes) const;
    std::string cargo_color_name(int cargo_class) const;
    bool validate_dropoff_color_config() const;
    bool validate_eightboxes_pickup_index_map() const;
    size_t pickup_index_for_eightboxes_slot(size_t string_idx) const;
    size_t eightboxes_slot_for_pickup_index(size_t pickup_idx) const;
    bool is_front_pickup(size_t pickup_idx) const;
    size_t nearest_rear_pickup_for_front(size_t front_pickup_idx) const;
    void play_path_ready_audio() const;
    void publish_boxsuc_handshake();
    void init_astar_graph();
    void update_path_viz_slots(
        const std::vector<int>& slot_classes,
        const std::vector<PickupDropTask>& tasks);
    void write_pickup_order_viz(
        const std::vector<int>& slot_classes,
        const std::vector<PickupDropTask>& tasks) const;
    bool append_astar_segment(double from_x, double from_y,
                              double to_x, double to_y, double to_yaw,
                              WaypointType final_type, int action_id,
                              const std::string& viz_tag);
    bool append_direct_nearest_segment(double from_x, double from_y,
                                       double to_x, double to_y, double to_yaw,
                                       WaypointType final_type, int action_id,
                                       const std::string& viz_tag,
                                       bool yaw_align_at_nearest = true);
    bool append_dropoff_retract_waypoint(double manip_x, double manip_y,
                                         double goal_yaw,
                                         const std::string& viz_tag);
    bool append_direct_manip_travel(
        double to_x, double to_y, double to_yaw,
        WaypointType final_type, int action_id,
        const std::string& viz_tag);
    bool find_pickup_staging_point(double pickup_x, double pickup_y, bool is_rear_row,
                                   double& sx, double& sy, double& syaw) const;
    void publish_logistics_event(LogisticsEventType event_type, size_t waypoint_index);
    void control_loop();

    // 辅助函数
    double normalize_angle(double angle) const;

    /** 路径未生成时原地预热转圈；若已处理本周期 cmd_vel 则返回 true */
    bool handle_pre_path_spin();

    bool load_recorded_trajectory(const std::string& path, const char* log_tag,
                                  std::vector<RecordedPose>& traj) const;
    bool sample_recorded_trajectory(const std::vector<RecordedPose>& traj,
                                    double elapsed, double& x, double& y, double& yaw) const;
    bool load_auto_parku_trajectory(const std::string& path);
    bool sample_auto_parku_reference(double elapsed, double& x, double& y, double& yaw) const;
    struct TrajectoryReplayOptions {
        bool align_to_start_before_play{false};
        double align_position_tolerance{0.12};
        double align_yaw_tolerance{0.12};
        double pause_tracking_error{0.0};  // >0: 跟踪误差超阈值时暂停时间轴
        double replay_start_t{0.0};        // >0: 从轨迹该时刻起回放（含对齐目标）
    };
    bool control_recorded_trajectory_replay(
        const char* log_tag,
        const std::vector<RecordedPose>& traj,
        bool loaded,
        bool& session_started,
        bool& clock_started,
        bool& done,
        rclcpp::Time& replay_t0,
        size_t pd_track_id,
        bool publish_nav_status_on_done,
        const TrajectoryReplayOptions& options);
    void control_logistics_entry_trajectory();
    bool entry_trajectory_ready() const;
    bool entry_trajectory_before_pre_scan() const;
    bool startup_entry_replay_enabled() const;
    void map_error_to_body(double err_x_map, double err_y_map,
                           double& err_x_body, double& err_y_body) const;
    void compute_body_pd_cmd(double err_x_body, double err_y_body, double err_yaw,
                             geometry_msgs::msg::Twist& cmd, size_t pd_track_id);
    geometry_msgs::msg::Twist limit_logistics_cmd_slew(
        const geometry_msgs::msg::Twist& raw_cmd, double dt);
    void publish_logistics_cmd(const geometry_msgs::msg::Twist& raw_cmd, double dt);
    void reset_logistics_cmd_slew();
    void control_auto_parku();
    bool update_arrival_debounce(double err_x_body, double err_y_body, double dist,
                                 double err_yaw, size_t waypoint_index,
                                 WaypointType waypoint_type,
                                 bool corridor_lateral_only_arrival,
                                 bool manip_approach_arrival,
                                 bool direct_nearest_approach,
                                 bool manip_to_nearest_arrival,
                                 double tol_x, double tol_y, double yaw_tolerance,
                                 double dist_tol);
    void get_arrival_tolerances(WaypointType type,
                                bool corridor_lateral_only_arrival,
                                double target_y,
                                double& tol_x, double& tol_y,
                                double& yaw_tolerance) const;
    double get_arrival_stable_time(WaypointType type,
                                   bool corridor_lateral_only_arrival) const;
    void reset_arrival_debounce();
    void reset_graph_yaw_align();
    rcl_interfaces::msg::SetParametersResult on_dynamic_parameters(
        const std::vector<rclcpp::Parameter>& parameters);
    void refresh_logistics_controller_config();
    bool is_dynamic_nav_param(const std::string& name) const;

    // 成员变量
    std::string target_frame_, child_frame_, tf_topic_, control_topic_, enable_topic_;
    std::vector<TargetPoint> waypoints_;
    double kp_x_, kp_y_, kp_yaw_, kd_x_, kd_y_, kd_yaw_;
    double max_vx_, max_vy_, max_dyaw_, control_rate_;
    double yaw_first_threshold_;
    double lin_vel_creep_min_; // >0 时在误差未消前对线速度设下限，避免末段过慢
    bool enable_control_;

    // 到点判定：距离/航向阈值 + 持续稳定时间消抖
    double arrival_position_tolerance_;
    double arrival_yaw_tolerance_;
    double transition_position_tolerance_;
    double transition_position_tolerance_x_;
    double transition_position_tolerance_y_;
    double transition_yaw_tolerance_;
    double manip_position_tolerance_;
    double manip_yaw_tolerance_;
    double arrival_stable_time_s_;
    double transition_arrival_stable_time_s_;
    double cross_band_transition_position_tolerance_y_;
    double cross_band_transition_arrival_stable_time_s_;
    double transition_dist_arrival_tolerance_;
    double manip_approach_arrival_tolerance_;
    double manip_arrival_stable_time_s_;
    double direct_nearest_yaw_align_threshold_{0.35};
    double direct_nearest_max_vx_{0.90};
    double precise_lateral_arrival_tolerance_y_{0.04};
    double precise_lateral_arrival_y_zone1_min_{0.9};
    double precise_lateral_arrival_y_zone1_max_{1.3};
    bool arrival_candidate_active_{false};
    size_t arrival_candidate_wp_idx_{0};
    rclcpp::Time arrival_candidate_start_time_;
    bool graph_yaw_align_active_{false};
    double graph_yaw_align_target_yaw_{0.0};
    size_t graph_yaw_align_wp_idx_{0};

    // 航点跟踪 PD：换 waypoint 时重置上一拍误差
    bool nav_pd_initialized_{false};
    size_t nav_pd_wp_idx_{0};
    double nav_prev_err_x_body_{0.0};
    double nav_prev_err_y_body_{0.0};
    double nav_prev_err_yaw_{0.0};
    rclcpp::Time nav_prev_ctrl_time_;
    
    // 雷达相对机体的平移参数
    double lidar_offset_x_;
    double lidar_offset_y_;

    std::string logistics_controller_type_;
    NavControllerConfig logistics_controller_config_;
    std::unique_ptr<NavController> logistics_controller_;
    rclcpp::Time logistics_prev_ctrl_time_;
    bool logistics_prev_ctrl_time_valid_{false};
    bool logistics_cmd_slew_enable_{false};
    bool logistics_cmd_slew_vy_enable_{false};
    double logistics_cmd_slew_vx_rate_{0.8};
    double logistics_cmd_slew_wz_rate_{1.2};
    bool logistics_cmd_slew_initialized_{false};
    geometry_msgs::msg::Twist last_logistics_cmd_;
    
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pose_pub_; // 当前位姿(x, y, yaw)发布器
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr lidar_pose_pub_;   // /quad/lidar_pose_xyyaw
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_; // 当前目标点(x, y, yaw)发布器
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr control_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr priority_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ocr_result_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr nav_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr boxsuc_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr logistics_event_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // 存储原始配置点位
    std::vector<double> trans_front_;
    std::vector<double> trans_back_;
    std::vector<double> trans_mid_;
    LidarAStar astar_planner_;
    lidar_nav::IsolationBandRegion isolation_band_region_;
    std::string add_point_file_;
    std::string path_viz_dir_;
    double astar_neighbor_radius_{1.0};
    double isolation_band_y_min_{0.9};
    double isolation_band_y_max_{3.8};
    double pickup_rear_staging_y_min_{3.5};
    double pickup_front_staging_y_max_{1.5};
    double rear_corridor_yaw_free_y_max_{4.5};
    double rear_exit_creep_y_min_{3.5};
    double rear_exit_creep_y_max_{4.1};
    std::vector<double> pickup_flat_;
    std::vector<double> dropoff_flat_;
    std::vector<int> dropoff_color_order_;
    std::vector<int> eightboxes_pickup_index_map_;
    std::vector<std::string> cargo_color_names_;
    int dropoff_layer_stride_{4};
    std::array<int, 4> class_to_layer1_slot_{};
    bool startup_pickup_enable_{false};
    int startup_pickup_index_{1};
    bool startup_entry_replay_after_pickup_{false};
    bool front_dependency_enable_{false};
    std::string front_dependency_column_match_{"nearest_x"};
    double pickup_greedy_y_distance_weight_{1.5};
    
    bool path_generated_;
    bool no_path_;
    int no_path_count_;
    bool logistics_finished_sent_{false};

    double x_, y_, yaw_;
    bool got_tf_;
    int tf_count_;
    bool printed_frames_;
    int print_count_;
    size_t current_wp_idx_;
    int logistics_event_seq_{0};

    int ocr_result_;
    bool ocr_result_locked_{false};
    int ocr_streak_candidate_{-1};
    int ocr_streak_count_{0};

    // 路径未生成时的预热：按 TF 反馈依次对准 pre_path_spin_target_yaws（参数来自 prepath.yaml）
    enum class PrePathSpinPhase { Idle, SeekYaw, PauseBetween, Done };
    bool pre_path_spin_enable_;
    std::vector<double> pre_path_spin_target_yaws_;
    double pre_path_spin_pause_s_;
    double pre_path_spin_kp_yaw_;
    double pre_path_spin_max_wz_;
    double pre_path_spin_yaw_tol_;
    bool pre_scan_pause_after_done_;
    double pre_scan_hold_after_done_s_{1.0};
    bool pre_scan_fallback_enable_;
    double pre_scan_timeout_s_;
    std::vector<int> default_priorities_;
    bool pre_scan_spin_done_{false};
    bool pre_scan_hold_after_done_started_{false};
    bool pre_scan_hold_after_done_done_{false};
    bool pre_scan_waiting_resume_{false};
    bool pre_scan_event_sent_{false};
    bool pre_scan_timeout_fallback_{false};
    bool pre_scan_started_{false};
    bool pre_scan_entry_point_arrived_{false};
    bool pre_scan_entry_point_done_{false};
    bool pre_scan_entry_wait_started_{false};
    bool last_enable_control_{false};
    PrePathSpinPhase pre_path_spin_phase_;
    size_t pre_path_spin_yaw_idx_{0};
    rclcpp::Time pre_path_spin_t0_;
    rclcpp::Time pre_scan_hold_after_done_t0_;
    rclcpp::Time pre_scan_start_time_;
    rclcpp::Time pre_scan_entry_wait_t0_;

    std::vector<int> pending_raw_classes_;   // 最近一条 eightboxes 归一化后的 8 个类别；A 区后四位已反转
    std::string pending_eightboxes_raw_text_{"XXXXXXXX"};
    bool pending_priorities_valid_{false};
    bool self_path_valid_{false};

    // 收到 eightboxes 后等待 OCR 的超时（秒）；超时则按左扫顺序生成路径
    rclcpp::Time ocr_wait_t0_;
    bool ocr_wait_armed_{false};
    double ocr_wait_after_eightboxes_sec_{15.0};

    // 尚未收到 eightboxes 时的兜底计时
    rclcpp::Time path_fallback_wait_t0_;
    bool path_fallback_wait_armed_{false};

    // 等待逻辑相关
    double wait_time_;
    bool is_waiting_;
    rclcpp::Time wait_start_time_;
    double path_fallback_timeout_sec_;
    bool external_manipulation_enable_;

    // 直线模式相关参数
    bool straight_line_enable_;
    double straight_target_yaw_;
    double straight_target_dist_;
    double straight_max_speed_;
    double straight_kp_y_;

    // 直线模式状态变量
    NavMode current_mode_{NavMode::LOGISTICS_TASK};
    bool straight_started_{false};
    double straight_start_x_{0.0};
    double straight_start_y_{0.0};
    bool straight_done_{false};

    // AUTO_PARKU 轨迹回放
    std::string auto_parku_trajectory_path_;
    std::vector<RecordedPose> auto_parku_traj_;
    bool auto_parku_loaded_{false};
    bool auto_parku_started_{false};
    bool auto_parku_clock_started_{false};
    bool auto_parku_done_{false};
    rclcpp::Time replay_t0_;

    // 物流 PRE_SCAN：pre_path_spin 完成后回放录轨迹，再 A*
    bool logistics_entry_trajectory_enable_{false};
    std::string logistics_entry_trajectory_path_;
    std::vector<RecordedPose> entry_traj_;
    bool entry_traj_loaded_{false};
    bool entry_traj_session_started_{false};
    bool entry_traj_clock_started_{false};
    bool entry_traj_done_{false};
    bool startup_entry_replay_pending_{false};
    double entry_replay_start_t_{0.0};
    bool startup_first_pickup_wp_valid_{false};
    size_t startup_first_pickup_wp_idx_{0};
    rclcpp::Time entry_replay_t0_;
    double entry_traj_align_position_tolerance_{0.12};
    double entry_traj_align_yaw_tolerance_{0.12};
    double entry_traj_pause_tracking_error_{0.35};
};

#endif  // LIDAR_NAV_DEMO_CPP_LIDAR_NAV_DEMO_NODE_HPP_
