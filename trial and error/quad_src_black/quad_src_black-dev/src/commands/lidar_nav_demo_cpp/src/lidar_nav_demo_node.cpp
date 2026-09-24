#include "lidar_nav_demo_cpp/lidar_nav_demo_node.hpp"
#include "lidar_nav_demo_cpp/field_layout_loader.hpp"

#include <cmath>
#include <functional>
#include <unordered_set>

namespace {

bool read_finite_number(const rclcpp::Parameter& parameter, double& value, std::string& reason) {
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        value = parameter.as_double();
    } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        value = static_cast<double>(parameter.as_int());
    } else {
        reason = parameter.get_name() + " must be a numeric parameter";
        return false;
    }

    if (!std::isfinite(value)) {
        reason = parameter.get_name() + " must be finite";
        return false;
    }
    return true;
}

bool is_positive_limit_param(const std::string& name) {
    return name == "max_vx" ||
           name == "max_vy" ||
           name == "max_dyaw" ||
           name == "approach_max_vx" ||
           name == "approach_max_vy" ||
           name == "approach_max_wz" ||
           name == "align_max_vx" ||
           name == "align_max_vy" ||
           name == "align_max_wz" ||
           name == "dock_max_vx" ||
           name == "dock_max_vy" ||
           name == "dock_max_wz" ||
           name == "logistics_cmd_slew_vx_rate" ||
           name == "logistics_cmd_slew_wz_rate";
}

bool is_nonnegative_param(const std::string& name) {
    return name.find("tolerance") != std::string::npos ||
           name.find("threshold") != std::string::npos ||
           name.find("deadband") != std::string::npos ||
           name.find("creep") != std::string::npos ||
           name.find("_dist") != std::string::npos ||
           name.find("_enter") != std::string::npos ||
           name.find("_exit") != std::string::npos ||
           name == "approach_lateral_yaw_comp_gain" ||
           name == "approach_lateral_yaw_comp_max" ||
           name == "approach_lateral_vy_scale" ||
           name == "reverse_vx_scale" ||
           name == "arrival_stable_time_s" ||
           name == "transition_arrival_stable_time_s" ||
           name == "manip_arrival_stable_time_s" ||
           name == "max_vy_trim";
}

bool read_bool_parameter(const rclcpp::Parameter& parameter, bool& value, std::string& reason) {
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        reason = parameter.get_name() + " must be a bool parameter";
        return false;
    }
    value = parameter.as_bool();
    return true;
}

}  // namespace

LidarNavControl::LidarNavControl()
    : Node("lidar_nav_demo_node"),
      path_generated_(false),
      no_path_(false),
      no_path_count_(0),
      x_(0.0),
      y_(0.0),
      yaw_(0.0),
      got_tf_(false),
      tf_count_(0),
      printed_frames_(false),
      print_count_(0),
      current_wp_idx_(0),
      ocr_result_(0),
      pre_path_spin_phase_(PrePathSpinPhase::Idle),
      is_waiting_(false)
{
    // 声明参数
    this->declare_parameter("target_frame", "camera_init");
    this->declare_parameter("child_frame", "aft_mapped");
    
    // 声明目标点序列参数 (categorized waypoints)
    this->declare_parameter("add_point_file", "");
    this->declare_parameter("path_viz_dir", "");
    this->declare_parameter("astar_neighbor_radius", 1.0);
    this->declare_parameter("isolation_band_y_min", 0.9);
    this->declare_parameter("isolation_band_y_max", 3.8);
    this->declare_parameter("pickup_rear_staging_y_min", 3.5);
    this->declare_parameter("rear_corridor_yaw_free_y_max", 4.5);
    this->declare_parameter("rear_exit_creep_y_min", 3.5);
    this->declare_parameter("rear_exit_creep_y_max", 4.1);
    this->declare_parameter("rear_exit_forward_creep_min", 0.20);
    this->declare_parameter("rear_exit_lateral_creep_min", 0.30);
    this->declare_parameter("pickup_front_staging_y_max", 1.5);
    this->declare_parameter("transition_front", std::vector<double>{1.0, 1.0, 0.0});
    this->declare_parameter("transition_back", std::vector<double>{1.0, -1.0, 0.0});
    this->declare_parameter("transition_mid", std::vector<double>{1.0, 0.0, 0.0});
    this->declare_parameter("pickup_points", std::vector<double>{2.0, 1.0, 1.57, 1.0}); // x, y, yaw, is_front
    this->declare_parameter("dropoff_points", std::vector<double>{2.0, -1.0, -1.57});
    this->declare_parameter("use_field_layout_generated_points", false);
    this->declare_parameter(
        "field_generated_points_file",
        "/home/cat/hitcrt_quad_ws/src/commands/lidar_nav_demo_cpp/config/manip_points_generated.yaml");
    this->declare_parameter(
        "field_anchor_points_file",
        "/home/cat/hitcrt_quad_ws/src/commands/lidar_nav_demo_cpp/config/trajectories/example_point.txt");
    this->declare_parameter("cargo_color_names",
        std::vector<std::string>{"green", "gray", "red", "blue"});
    this->declare_parameter("dropoff_color_order", std::vector<int64_t>{0, 1, 2, 3});
    this->declare_parameter("dropoff_layer_stride", 4);
    this->declare_parameter("eightboxes_pickup_index_map",
        std::vector<int64_t>{4, 5, 6, 7, 0, 1, 2, 3});
    this->declare_parameter("startup_pickup_enable", false);
    this->declare_parameter("startup_pickup_index", 1);
    this->declare_parameter("startup_entry_replay_after_pickup", false);
    this->declare_parameter("front_dependency_enable", false);
    this->declare_parameter("front_dependency_column_match", "nearest_x");
    this->declare_parameter("pickup_greedy_y_distance_weight", 1.5);
    
    this->declare_parameter("wait_time", 2.0); // 到达目标点后等待时间(秒)
    this->declare_parameter("external_manipulation_enable", true);
    this->declare_parameter("path_fallback_timeout_sec", 40.0); // 未收到 eightboxes 时的兜底超时
    this->declare_parameter("ocr_wait_after_eightboxes_sec", 15.0); // 收到 eightboxes 后等待 OCR 的最长时间

    this->declare_parameter("kp_x", 0.5);
    this->declare_parameter("kp_y", 0.5);
    this->declare_parameter("kp_yaw", 1.0);
    this->declare_parameter("kd_x", 0.0);
    this->declare_parameter("kd_y", 0.0);
    this->declare_parameter("kd_yaw", 0.0);
    this->declare_parameter("lin_vel_creep_min", 0.0); // m/s，仅平移阶段；0 表示关闭
    this->declare_parameter("max_vx", 0.5);
    this->declare_parameter("max_vy", 0.5);
    this->declare_parameter("max_dyaw", 0.5);
    this->declare_parameter("yaw_first_threshold", 0.35);
    this->declare_parameter("control_rate", 50.0);
    this->declare_parameter("arrival_position_tolerance", 0.12);
    this->declare_parameter("arrival_yaw_tolerance", 0.10);
    this->declare_parameter("transition_position_tolerance", 0.12);
    this->declare_parameter("transition_position_tolerance_x", -1.0);
    this->declare_parameter("transition_position_tolerance_y", -1.0);
    this->declare_parameter("transition_yaw_tolerance", 0.12);
    this->declare_parameter("manip_position_tolerance", 0.05);
    this->declare_parameter("manip_yaw_tolerance", 0.06);
    this->declare_parameter("arrival_stable_time_s", 0.30);
    this->declare_parameter("transition_arrival_stable_time_s", 0.10);
    this->declare_parameter("cross_band_transition_position_tolerance_y", 0.15);
    this->declare_parameter("cross_band_transition_arrival_stable_time_s", 0.20);
    this->declare_parameter("transition_dist_arrival_tolerance", 0.15);
    this->declare_parameter("manip_approach_arrival_tolerance", 0.15);
    this->declare_parameter("manip_arrival_stable_time_s", 0.25);
    this->declare_parameter("precise_lateral_arrival_tolerance_y", 0.04);
    this->declare_parameter("precise_lateral_arrival_y_zone1_min", 0.9);
    this->declare_parameter("precise_lateral_arrival_y_zone1_max", 1.3);
    this->declare_parameter("logistics_controller_type", "heading_dock");
    this->declare_parameter("approach_to_align_dist", 0.90);
    this->declare_parameter("dock_start_dist", 0.10);
    this->declare_parameter("approach_max_vx", 0.45);
    this->declare_parameter("approach_max_vy", 0.06);
    this->declare_parameter("approach_max_wz", 0.60);
    this->declare_parameter("align_max_vx", 0.40);
    this->declare_parameter("align_max_vy", 0.22);
    this->declare_parameter("align_max_wz", 0.80);
    this->declare_parameter("dock_max_vx", 0.25);
    this->declare_parameter("dock_max_vy", 0.12);
    this->declare_parameter("dock_max_wz", 0.45);
    this->declare_parameter("reverse_vx_scale", 0.5);
    this->declare_parameter("kp_forward", 1.1);
    this->declare_parameter("kp_bearing", 1.2);
    this->declare_parameter("kp_final_yaw", 1.0);
    this->declare_parameter("kp_lateral_to_yaw", 0.6);
    this->declare_parameter("kp_dock_x", 2.0);
    this->declare_parameter("kp_dock_y", 1.5);
    this->declare_parameter("kp_dock_yaw", 1.2);
    this->declare_parameter("lateral_trim_gain", 0.75);
    this->declare_parameter("max_vy_trim", 0.22);
    this->declare_parameter("approach_creep_min", 0.18);
    this->declare_parameter("align_forward_creep_min", 0.20);
    this->declare_parameter("align_lateral_creep_min", 0.18);
    this->declare_parameter("cross_band_align_max_vy", 0.22);
    this->declare_parameter("cross_band_align_lateral_creep_min", 0.15);
    this->declare_parameter("precise_lateral_align_lateral_creep_min", 0.30);
    this->declare_parameter("cross_band_approach_max_vx", 0.50);
    this->declare_parameter("cross_band_align_max_vx", 0.35);
    this->declare_parameter("cross_band_align_forward_creep_min", 0.20);
    this->declare_parameter("align_yaw_creep_min", 0.30);
    this->declare_parameter("dock_creep_min", 0.12);
    this->declare_parameter("dock_forward_creep_min", 0.08);
    this->declare_parameter("dock_lateral_creep_min", 0.12);
    this->declare_parameter("approach_final_yaw_weight", 0.8);
    this->declare_parameter("approach_lateral_yaw_comp_enable", true);
    this->declare_parameter("approach_lateral_yaw_comp_gain", 0.45);
    this->declare_parameter("approach_lateral_yaw_comp_max", 0.30);
    this->declare_parameter("approach_lateral_vy_scale", 0.25);
    this->declare_parameter("align_yaw_priority_threshold", 0.8);
    this->declare_parameter("dock_yaw_priority_threshold", 1.10);
    this->declare_parameter("dock_position_only_disable_yaw", true);
    this->declare_parameter("dock_position_only_yaw_threshold", 0.10);
    this->declare_parameter("align_yaw_first_enter", 0.35);
    this->declare_parameter("align_yaw_first_exit", 0.20);
    this->declare_parameter("dock_yaw_first_enter", 0.25);
    this->declare_parameter("dock_yaw_first_exit", 0.12);
    this->declare_parameter("yaw_deadband", 0.05);
    this->declare_parameter("forward_only_transition_enable", true);
    this->declare_parameter("forward_only_bearing_threshold", 0.20);
    this->declare_parameter("forward_only_bearing_exit_threshold", 0.10);
    this->declare_parameter("forward_only_kp_bearing", 1.2);
    this->declare_parameter("forward_only_max_vx", 0.55);
    this->declare_parameter("forward_only_forward_creep_min", 0.18);
    this->declare_parameter("forward_only_yaw_creep_min", 0.25);
    this->declare_parameter("graph_reverse_yaw_threshold", 1.0);
    this->declare_parameter("direct_nearest_yaw_align_threshold", 0.35);
    this->declare_parameter("direct_nearest_max_vx", 0.90);
    this->declare_parameter("direct_manip_dock_handoff_dist", 0.30);
    this->declare_parameter("logistics_cmd_slew_enable", false);
    this->declare_parameter("logistics_cmd_slew_vx_rate", 0.8);
    this->declare_parameter("logistics_cmd_slew_wz_rate", 1.2);
    this->declare_parameter("logistics_cmd_slew_vy_enable", false);
    this->declare_parameter("enable_control", false); // 默认不使能，等待任务状态机唤醒
    this->declare_parameter("tf_topic", "/tf");
    this->declare_parameter("control_topic", "/lidar_nav_control");
    this->declare_parameter("enable_topic", "/quad/logistics_nav_enable");
    this->declare_parameter("ocr_result", 0);

    this->declare_parameter("pre_path_spin_enable", true);
    this->declare_parameter("pre_path_spin_target_yaws",
        std::vector<double>{0.5, -0.5, 0.5, -0.5, 0.0});
    this->declare_parameter("pre_path_spin_pause_s", 1.0);
    this->declare_parameter("pre_path_spin_kp_yaw", 3.0);
    this->declare_parameter("pre_path_spin_max_wz", 0.8);
    this->declare_parameter("pre_path_spin_yaw_tol", 0.1);
    this->declare_parameter("pre_scan_pause_after_done", true);
    this->declare_parameter("pre_scan_hold_after_done_s", 1.0);
    this->declare_parameter("pre_scan_fallback_enable", true);
    this->declare_parameter("pre_scan_timeout_s", 8.0);
    this->declare_parameter("default_priorities",
        std::vector<int64_t>{0, 0, 1, 1, 0, 0, 0, 0});

    this->declare_parameter("auto_parku_enable", false);
    this->declare_parameter("auto_parku_trajectory_path", "");
    this->declare_parameter("logistics_entry_trajectory_enable", false);
    this->declare_parameter("logistics_entry_trajectory_path",
        "/home/cat/hitcrt_quad_ws/src/commands/lidar_nav_demo_cpp/config/trajectories/record_pass.txt");
    this->declare_parameter("entry_traj_align_position_tolerance", 0.12);
    this->declare_parameter("entry_traj_align_yaw_tolerance", 0.12);
    this->declare_parameter("entry_traj_pause_tracking_error", 0.35);

    // 直线模式参数
    this->declare_parameter("straight_line_enable", false);
    this->declare_parameter("straight_target_yaw", 0.0);
    this->declare_parameter("straight_target_dist", 2.0);
    this->declare_parameter("straight_max_speed", 0.5);
    this->declare_parameter("straight_kp_y", 2.0);

    // 声明雷达到机体的平移参数 (基于机体系下的坐标)
    this->declare_parameter("lidar_offset_x", -0.05); // 雷达在机体系下的前向偏移
    this->declare_parameter("lidar_offset_y", 0.0); // 雷达在机体系下的左向偏移

    // 获取参数
    target_frame_ = this->get_parameter("target_frame").as_string();
    child_frame_ = this->get_parameter("child_frame").as_string();
    
    std::vector<double> trans_front = this->get_parameter("transition_front").as_double_array();
    std::vector<double> trans_back = this->get_parameter("transition_back").as_double_array();
    std::vector<double> trans_mid = this->get_parameter("transition_mid").as_double_array();
    std::vector<double> pickup_flat = this->get_parameter("pickup_points").as_double_array();
    std::vector<double> dropoff_flat = this->get_parameter("dropoff_points").as_double_array();
    
    // 存入成员变量，等待接收优先级信息后再生成路径
    trans_front_ = trans_front;
    trans_back_ = trans_back;
    trans_mid_ = trans_mid;
    pickup_flat_ = pickup_flat;
    dropoff_flat_ = dropoff_flat;

    const bool use_field_generated =
        this->get_parameter("use_field_layout_generated_points").as_bool();
    if (use_field_generated) {
        const std::string generated_path =
            this->get_parameter("field_generated_points_file").as_string();
        lidar_nav::GeneratedManipPoints generated;
        std::string load_error;
        if (lidar_nav::load_generated_manip_points(generated_path, generated, load_error)) {
            pickup_flat_ = generated.pickup_flat;
            dropoff_flat_ = generated.dropoff_flat;
            RCLCPP_INFO(this->get_logger(),
                        "Loaded field-layout generated manip points from %s",
                        generated_path.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "use_field_layout_generated_points=true but load failed: %s; "
                         "using pickup/dropoff from waypoints.yaml",
                         load_error.c_str());
        }
    }

    add_point_file_ = this->get_parameter("add_point_file").as_string();
    path_viz_dir_ = this->get_parameter("path_viz_dir").as_string();
    astar_neighbor_radius_ = this->get_parameter("astar_neighbor_radius").as_double();
    isolation_band_y_min_ = this->get_parameter("isolation_band_y_min").as_double();
    isolation_band_y_max_ = this->get_parameter("isolation_band_y_max").as_double();
    pickup_rear_staging_y_min_ = this->get_parameter("pickup_rear_staging_y_min").as_double();
    rear_corridor_yaw_free_y_max_ =
        this->get_parameter("rear_corridor_yaw_free_y_max").as_double();
    rear_exit_creep_y_min_ = this->get_parameter("rear_exit_creep_y_min").as_double();
    rear_exit_creep_y_max_ = this->get_parameter("rear_exit_creep_y_max").as_double();
    pickup_front_staging_y_max_ = this->get_parameter("pickup_front_staging_y_max").as_double();
    init_astar_graph();

    cargo_color_names_ = this->get_parameter("cargo_color_names").as_string_array();
    dropoff_layer_stride_ = this->get_parameter("dropoff_layer_stride").as_int();
    const auto dropoff_color_order_i64 = this->get_parameter("dropoff_color_order").as_integer_array();
    dropoff_color_order_.clear();
    dropoff_color_order_.reserve(dropoff_color_order_i64.size());
    for (const auto value : dropoff_color_order_i64) {
        dropoff_color_order_.push_back(static_cast<int>(value));
    }
    class_to_layer1_slot_.fill(-1);
    if (dropoff_color_order_.size() == 4) {
        for (int slot = 0; slot < 4; ++slot) {
            const int cls = dropoff_color_order_[static_cast<size_t>(slot)];
            if (cls >= 0 && cls < 4) {
                class_to_layer1_slot_[static_cast<size_t>(cls)] = slot;
            }
        }
    }
    if (!validate_dropoff_color_config()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Invalid dropoff_color_order; using default [0,1,2,3]");
        dropoff_color_order_ = {0, 1, 2, 3};
        class_to_layer1_slot_ = {0, 1, 2, 3};
        dropoff_layer_stride_ = 4;
    }

    const auto eightboxes_pickup_map_i64 =
        this->get_parameter("eightboxes_pickup_index_map").as_integer_array();
    eightboxes_pickup_index_map_.clear();
    eightboxes_pickup_index_map_.reserve(eightboxes_pickup_map_i64.size());
    for (const auto value : eightboxes_pickup_map_i64) {
        eightboxes_pickup_index_map_.push_back(static_cast<int>(value));
    }
    if (!validate_eightboxes_pickup_index_map()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Invalid eightboxes_pickup_index_map; using default [4,5,6,7,0,1,2,3]");
        eightboxes_pickup_index_map_ = {4, 5, 6, 7, 0, 1, 2, 3};
    }
    startup_pickup_enable_ = this->get_parameter("startup_pickup_enable").as_bool();
    startup_pickup_index_ = this->get_parameter("startup_pickup_index").as_int();
    startup_entry_replay_after_pickup_ =
        this->get_parameter("startup_entry_replay_after_pickup").as_bool();
    front_dependency_enable_ = this->get_parameter("front_dependency_enable").as_bool();
    front_dependency_column_match_ =
        this->get_parameter("front_dependency_column_match").as_string();
    if (front_dependency_column_match_ != "nearest_x") {
        RCLCPP_WARN(this->get_logger(),
                    "Unsupported front_dependency_column_match '%s'; using nearest_x.",
                    front_dependency_column_match_.c_str());
        front_dependency_column_match_ = "nearest_x";
    }
    pickup_greedy_y_distance_weight_ =
        this->get_parameter("pickup_greedy_y_distance_weight").as_double();
    if (pickup_greedy_y_distance_weight_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(),
                    "pickup_greedy_y_distance_weight must be > 0; using 1.0.");
        pickup_greedy_y_distance_weight_ = 1.0;
    }
    
    kp_x_ = this->get_parameter("kp_x").as_double();
    kp_y_ = this->get_parameter("kp_y").as_double();
    kp_yaw_ = this->get_parameter("kp_yaw").as_double();
    kd_x_ = this->get_parameter("kd_x").as_double();
    kd_y_ = this->get_parameter("kd_y").as_double();
    kd_yaw_ = this->get_parameter("kd_yaw").as_double();
    lin_vel_creep_min_ = this->get_parameter("lin_vel_creep_min").as_double();
    max_vx_ = this->get_parameter("max_vx").as_double();
    max_vy_ = this->get_parameter("max_vy").as_double();
    max_dyaw_ = this->get_parameter("max_dyaw").as_double();
    yaw_first_threshold_ = this->get_parameter("yaw_first_threshold").as_double();
    control_rate_ = this->get_parameter("control_rate").as_double();
    arrival_position_tolerance_ = this->get_parameter("arrival_position_tolerance").as_double();
    arrival_yaw_tolerance_ = this->get_parameter("arrival_yaw_tolerance").as_double();
    transition_position_tolerance_ = this->get_parameter("transition_position_tolerance").as_double();
    const double transition_tol_x_param =
        this->get_parameter("transition_position_tolerance_x").as_double();
    const double transition_tol_y_param =
        this->get_parameter("transition_position_tolerance_y").as_double();
    transition_position_tolerance_x_ =
        transition_tol_x_param > 0.0 ? transition_tol_x_param : transition_position_tolerance_;
    transition_position_tolerance_y_ =
        transition_tol_y_param > 0.0 ? transition_tol_y_param : transition_position_tolerance_;
    transition_yaw_tolerance_ = this->get_parameter("transition_yaw_tolerance").as_double();
    manip_position_tolerance_ = this->get_parameter("manip_position_tolerance").as_double();
    manip_yaw_tolerance_ = this->get_parameter("manip_yaw_tolerance").as_double();
    arrival_stable_time_s_ = this->get_parameter("arrival_stable_time_s").as_double();
    transition_arrival_stable_time_s_ =
        this->get_parameter("transition_arrival_stable_time_s").as_double();
    cross_band_transition_position_tolerance_y_ =
        this->get_parameter("cross_band_transition_position_tolerance_y").as_double();
    cross_band_transition_arrival_stable_time_s_ =
        this->get_parameter("cross_band_transition_arrival_stable_time_s").as_double();
    transition_dist_arrival_tolerance_ =
        this->get_parameter("transition_dist_arrival_tolerance").as_double();
    manip_approach_arrival_tolerance_ =
        this->get_parameter("manip_approach_arrival_tolerance").as_double();
    manip_arrival_stable_time_s_ =
        this->get_parameter("manip_arrival_stable_time_s").as_double();
    direct_nearest_yaw_align_threshold_ =
        this->get_parameter("direct_nearest_yaw_align_threshold").as_double();
    direct_nearest_max_vx_ =
        this->get_parameter("direct_nearest_max_vx").as_double();
    logistics_cmd_slew_enable_ =
        this->get_parameter("logistics_cmd_slew_enable").as_bool();
    logistics_cmd_slew_vx_rate_ =
        this->get_parameter("logistics_cmd_slew_vx_rate").as_double();
    logistics_cmd_slew_wz_rate_ =
        this->get_parameter("logistics_cmd_slew_wz_rate").as_double();
    logistics_cmd_slew_vy_enable_ =
        this->get_parameter("logistics_cmd_slew_vy_enable").as_bool();
    precise_lateral_arrival_tolerance_y_ =
        this->get_parameter("precise_lateral_arrival_tolerance_y").as_double();
    precise_lateral_arrival_y_zone1_min_ =
        this->get_parameter("precise_lateral_arrival_y_zone1_min").as_double();
    precise_lateral_arrival_y_zone1_max_ =
        this->get_parameter("precise_lateral_arrival_y_zone1_max").as_double();
    logistics_controller_type_ = this->get_parameter("logistics_controller_type").as_string();
    enable_control_ = this->get_parameter("enable_control").as_bool();
    wait_time_ = this->get_parameter("wait_time").as_double();
    external_manipulation_enable_ = this->get_parameter("external_manipulation_enable").as_bool();
    path_fallback_timeout_sec_ = this->get_parameter("path_fallback_timeout_sec").as_double();
    ocr_wait_after_eightboxes_sec_ = this->get_parameter("ocr_wait_after_eightboxes_sec").as_double();
    tf_topic_ = this->get_parameter("tf_topic").as_string();
    control_topic_ = this->get_parameter("control_topic").as_string();
    enable_topic_ = this->get_parameter("enable_topic").as_string();
    ocr_result_ = this->get_parameter("ocr_result").as_int();

    lidar_offset_x_ = this->get_parameter("lidar_offset_x").as_double();
    lidar_offset_y_ = this->get_parameter("lidar_offset_y").as_double();

    refresh_logistics_controller_config();
    if (logistics_controller_type_ != "heading_dock" &&
        logistics_controller_type_ != "body_pd") {
        RCLCPP_WARN(this->get_logger(),
                    "Unknown logistics_controller_type '%s'; using heading_dock.",
                    logistics_controller_type_.c_str());
        logistics_controller_type_ = "heading_dock";
    }
    logistics_controller_ =
        make_nav_controller(logistics_controller_type_, logistics_controller_config_);

    pre_path_spin_enable_ = this->get_parameter("pre_path_spin_enable").as_bool();
    pre_path_spin_target_yaws_ = this->get_parameter("pre_path_spin_target_yaws").as_double_array();
    pre_path_spin_pause_s_ = this->get_parameter("pre_path_spin_pause_s").as_double();
    pre_path_spin_kp_yaw_ = this->get_parameter("pre_path_spin_kp_yaw").as_double();
    pre_path_spin_max_wz_ = this->get_parameter("pre_path_spin_max_wz").as_double();
    pre_path_spin_yaw_tol_ = this->get_parameter("pre_path_spin_yaw_tol").as_double();
    pre_scan_pause_after_done_ = this->get_parameter("pre_scan_pause_after_done").as_bool();
    pre_scan_hold_after_done_s_ = this->get_parameter("pre_scan_hold_after_done_s").as_double();
    pre_scan_fallback_enable_ = this->get_parameter("pre_scan_fallback_enable").as_bool();
    pre_scan_timeout_s_ = this->get_parameter("pre_scan_timeout_s").as_double();
    const auto default_priorities_i64 = this->get_parameter("default_priorities").as_integer_array();
    default_priorities_.clear();
    default_priorities_.reserve(default_priorities_i64.size());
    for (const auto value : default_priorities_i64) {
        default_priorities_.push_back(static_cast<int>(value));
    }

    const bool auto_parku_enable = this->get_parameter("auto_parku_enable").as_bool();
    auto_parku_trajectory_path_ = this->get_parameter("auto_parku_trajectory_path").as_string();
    logistics_entry_trajectory_enable_ =
        this->get_parameter("logistics_entry_trajectory_enable").as_bool();
    logistics_entry_trajectory_path_ =
        this->get_parameter("logistics_entry_trajectory_path").as_string();
    entry_traj_align_position_tolerance_ =
        this->get_parameter("entry_traj_align_position_tolerance").as_double();
    entry_traj_align_yaw_tolerance_ =
        this->get_parameter("entry_traj_align_yaw_tolerance").as_double();
    entry_traj_pause_tracking_error_ =
        this->get_parameter("entry_traj_pause_tracking_error").as_double();

    straight_line_enable_ = this->get_parameter("straight_line_enable").as_bool();
    straight_target_yaw_ = this->get_parameter("straight_target_yaw").as_double();
    straight_target_dist_ = this->get_parameter("straight_target_dist").as_double();
    straight_max_speed_ = this->get_parameter("straight_max_speed").as_double();
    straight_kp_y_ = this->get_parameter("straight_kp_y").as_double();

    if (auto_parku_enable) {
        current_mode_ = NavMode::AUTO_PARKU;
        if (!load_auto_parku_trajectory(auto_parku_trajectory_path_)) {
            RCLCPP_ERROR(this->get_logger(),
                         "AUTO_PARKU: failed to load trajectory '%s', fallback to LOGISTICS_TASK",
                         auto_parku_trajectory_path_.c_str());
            current_mode_ = NavMode::LOGISTICS_TASK;
        }
    } else if (straight_line_enable_) {
        current_mode_ = NavMode::STRAIGHT_LINE;
    } else {
        current_mode_ = NavMode::LOGISTICS_TASK;
    }

    if (logistics_entry_trajectory_enable_ && current_mode_ == NavMode::LOGISTICS_TASK) {
        entry_traj_loaded_ = load_recorded_trajectory(
            logistics_entry_trajectory_path_, "LOGISTICS_ENTRY", entry_traj_);
        if (!entry_traj_loaded_) {
            RCLCPP_WARN(this->get_logger(),
                        "LOGISTICS_ENTRY: failed to load '%s'; skipping entry trajectory",
                        logistics_entry_trajectory_path_.c_str());
            entry_traj_done_ = true;
        }
    } else {
        entry_traj_done_ = true;
    }

    tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        tf_topic_, 100, std::bind(&LidarNavControl::tf_callback, this, std::placeholders::_1));
    
    // 改为相对话题 cmd_vel，使其与 task_state_machine 发布在同一个命名空间下
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/quad/cmd_vel", 10);
    
    // 当前位姿发布器，格式为 [x, y, yaw]
    pose_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/quad/current_pose", 10);
    lidar_pose_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/quad/lidar_pose_xyyaw", 10);
    
    // 当前目标点发布器，格式为 [x, y, yaw]
    target_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/quad/current_target", 10);
    
    // 导航状态发布器：到达目标点时通知上层状态机
    nav_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("nav_status", 10);

    boxsuc_pub_ = this->create_publisher<std_msgs::msg::String>("boxsuc", 10);
    logistics_event_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
        "/quad/logistics_nav_event", 10);

    // eightboxes：8 位字符，每位类别 0~3；与 OCR 配合决定取货顺序与放货映射
    priority_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/eightboxes", 10, std::bind(&LidarNavControl::priority_callback, this, std::placeholders::_1));

    ocr_result_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/stable_arithmetic_result", 10,
        std::bind(&LidarNavControl::ocr_result_callback, this, std::placeholders::_1));

    // 订阅使能话题，由 task_state_machine 控制启停
    enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        enable_topic_, 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
            const bool new_enable = msg->data;
            if (new_enable && !last_enable_control_ && pre_scan_waiting_resume_) {
                pre_scan_waiting_resume_ = false;
                RCLCPP_INFO(this->get_logger(), "Pre-scan resume received. Continue path generation/navigation.");
                try_generate_path_with_pending_priorities();
            }
            enable_control_ = new_enable;
            last_enable_control_ = new_enable;
            // RCLCPP_INFO(this->get_logger(), "Navigation Enable State Updated to: %s", enable_control_ ? "TRUE" : "FALSE");
        });

    control_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        control_topic_, 10, std::bind(&LidarNavControl::control_callback, this, std::placeholders::_1));

    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&LidarNavControl::on_dynamic_parameters, this, std::placeholders::_1));

    double period = 1.0 / control_rate_;
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(period), 
        std::bind(&LidarNavControl::control_loop, this));
        
    if (current_mode_ == NavMode::AUTO_PARKU) {
        RCLCPP_INFO(this->get_logger(),
                    "Started in AUTO_PARKU mode, trajectory: %s (%zu points), TF: %s",
                    auto_parku_trajectory_path_.c_str(), auto_parku_traj_.size(), tf_topic_.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Started. Waiting for material priorities to generate path... TF: %s",
                    tf_topic_.c_str());
        if (logistics_entry_trajectory_enable_ && entry_traj_loaded_) {
            RCLCPP_INFO(this->get_logger(),
                        "LOGISTICS_ENTRY trajectory enabled: %s (%zu points)",
                        logistics_entry_trajectory_path_.c_str(), entry_traj_.size());
        }
    }
}

rcl_interfaces::msg::SetParametersResult LidarNavControl::on_dynamic_parameters(
    const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;

    double new_kp_x = kp_x_;
    double new_kp_y = kp_y_;
    double new_kp_yaw = kp_yaw_;
    double new_kd_x = kd_x_;
    double new_kd_y = kd_y_;
    double new_kd_yaw = kd_yaw_;
    double new_lin_vel_creep_min = lin_vel_creep_min_;
    double new_max_vx = max_vx_;
    double new_max_vy = max_vy_;
    double new_max_dyaw = max_dyaw_;
    double new_yaw_first_threshold = yaw_first_threshold_;
    double new_arrival_position_tolerance = arrival_position_tolerance_;
    double new_arrival_yaw_tolerance = arrival_yaw_tolerance_;
    double new_transition_position_tolerance = transition_position_tolerance_;
    double new_transition_position_tolerance_x = transition_position_tolerance_x_;
    double new_transition_position_tolerance_y = transition_position_tolerance_y_;
    double new_transition_yaw_tolerance = transition_yaw_tolerance_;
    double new_manip_position_tolerance = manip_position_tolerance_;
    double new_manip_yaw_tolerance = manip_yaw_tolerance_;
    double new_arrival_stable_time_s = arrival_stable_time_s_;
    double new_transition_arrival_stable_time_s = transition_arrival_stable_time_s_;
    double new_cross_band_transition_position_tolerance_y =
        cross_band_transition_position_tolerance_y_;
    double new_cross_band_transition_arrival_stable_time_s =
        cross_band_transition_arrival_stable_time_s_;
    double new_transition_dist_arrival_tolerance = transition_dist_arrival_tolerance_;
    double new_manip_approach_arrival_tolerance = manip_approach_arrival_tolerance_;
    double new_manip_arrival_stable_time_s = manip_arrival_stable_time_s_;
    double new_precise_lateral_arrival_tolerance_y = precise_lateral_arrival_tolerance_y_;
    double new_precise_lateral_arrival_y_zone1_min = precise_lateral_arrival_y_zone1_min_;
    double new_precise_lateral_arrival_y_zone1_max = precise_lateral_arrival_y_zone1_max_;
    double new_rear_corridor_yaw_free_y_max = rear_corridor_yaw_free_y_max_;
    double new_rear_exit_creep_y_min = rear_exit_creep_y_min_;
    double new_rear_exit_creep_y_max = rear_exit_creep_y_max_;
    double new_lidar_offset_x = lidar_offset_x_;
    double new_lidar_offset_y = lidar_offset_y_;
    bool new_logistics_cmd_slew_enable = logistics_cmd_slew_enable_;
    bool new_logistics_cmd_slew_vy_enable = logistics_cmd_slew_vy_enable_;
    double new_logistics_cmd_slew_vx_rate = logistics_cmd_slew_vx_rate_;
    double new_logistics_cmd_slew_wz_rate = logistics_cmd_slew_wz_rate_;
    NavControllerConfig new_config = logistics_controller_config_;

    for (const auto& parameter : parameters) {
        const std::string name = parameter.get_name();
        if (name == "logistics_controller_type" || name == "control_rate") {
            result.reason = name + " is startup-only; edit waypoints.yaml and restart the node";
            return result;
        }
        if (!is_dynamic_nav_param(name)) {
            result.reason = name + " is not a runtime navigation tuning parameter";
            return result;
        }

        if (name == "approach_lateral_yaw_comp_enable" ||
            name == "dock_position_only_disable_yaw" ||
            name == "logistics_cmd_slew_enable" ||
            name == "logistics_cmd_slew_vy_enable") {
            bool value = false;
            if (!read_bool_parameter(parameter, value, result.reason)) {
                return result;
            }
            if (name == "approach_lateral_yaw_comp_enable") {
                new_config.approach_lateral_yaw_comp_enable = value;
            } else if (name == "dock_position_only_disable_yaw") {
                new_config.dock_position_only_disable_yaw = value;
            } else if (name == "logistics_cmd_slew_enable") {
                new_logistics_cmd_slew_enable = value;
            } else {
                new_logistics_cmd_slew_vy_enable = value;
            }
            continue;
        }

        double value = 0.0;
        if (!read_finite_number(parameter, value, result.reason)) {
            return result;
        }
        if (is_positive_limit_param(name) && value <= 0.0) {
            result.reason = name + " must be > 0.0";
            return result;
        }
        if (is_nonnegative_param(name) && value < 0.0) {
            result.reason = name + " must be >= 0.0";
            return result;
        }

        if (name == "kp_x") {
            new_kp_x = value;
            new_config.kp_x = value;
        } else if (name == "kp_y") {
            new_kp_y = value;
            new_config.kp_y = value;
        } else if (name == "kp_yaw") {
            new_kp_yaw = value;
            new_config.kp_yaw = value;
        } else if (name == "kd_x") {
            new_kd_x = value;
            new_config.kd_x = value;
        } else if (name == "kd_y") {
            new_kd_y = value;
            new_config.kd_y = value;
        } else if (name == "kd_yaw") {
            new_kd_yaw = value;
            new_config.kd_yaw = value;
        } else if (name == "lin_vel_creep_min") {
            new_lin_vel_creep_min = value;
            new_config.lin_vel_creep_min = value;
        } else if (name == "max_vx") {
            new_max_vx = value;
            new_config.max_vx = value;
        } else if (name == "max_vy") {
            new_max_vy = value;
            new_config.max_vy = value;
        } else if (name == "max_dyaw") {
            new_max_dyaw = value;
            new_config.max_wz = value;
        } else if (name == "yaw_first_threshold") {
            new_yaw_first_threshold = value;
            new_config.yaw_first_threshold = value;
        } else if (name == "arrival_position_tolerance") {
            new_arrival_position_tolerance = value;
        } else if (name == "arrival_yaw_tolerance") {
            new_arrival_yaw_tolerance = value;
        } else if (name == "transition_position_tolerance") {
            new_transition_position_tolerance = value;
        } else if (name == "transition_position_tolerance_x") {
            new_transition_position_tolerance_x = value;
        } else if (name == "transition_position_tolerance_y") {
            new_transition_position_tolerance_y = value;
        } else if (name == "transition_yaw_tolerance") {
            new_transition_yaw_tolerance = value;
        } else if (name == "manip_position_tolerance") {
            new_manip_position_tolerance = value;
        } else if (name == "manip_yaw_tolerance") {
            new_manip_yaw_tolerance = value;
        } else if (name == "arrival_stable_time_s") {
            new_arrival_stable_time_s = value;
        } else if (name == "transition_arrival_stable_time_s") {
            new_transition_arrival_stable_time_s = value;
        } else if (name == "cross_band_transition_position_tolerance_y") {
            new_cross_band_transition_position_tolerance_y = value;
        } else if (name == "cross_band_transition_arrival_stable_time_s") {
            new_cross_band_transition_arrival_stable_time_s = value;
        } else if (name == "transition_dist_arrival_tolerance") {
            new_transition_dist_arrival_tolerance = value;
        } else if (name == "manip_approach_arrival_tolerance") {
            new_manip_approach_arrival_tolerance = value;
        } else if (name == "manip_arrival_stable_time_s") {
            new_manip_arrival_stable_time_s = value;
        } else if (name == "precise_lateral_arrival_tolerance_y") {
            new_precise_lateral_arrival_tolerance_y = value;
        } else if (name == "precise_lateral_arrival_y_zone1_min") {
            new_precise_lateral_arrival_y_zone1_min = value;
        } else if (name == "precise_lateral_arrival_y_zone1_max") {
            new_precise_lateral_arrival_y_zone1_max = value;
        } else if (name == "rear_corridor_yaw_free_y_max") {
            new_rear_corridor_yaw_free_y_max = value;
            new_config.rear_corridor_yaw_free_y_max = value;
        } else if (name == "rear_exit_creep_y_min") {
            new_rear_exit_creep_y_min = value;
        } else if (name == "rear_exit_creep_y_max") {
            new_rear_exit_creep_y_max = value;
        } else if (name == "rear_exit_forward_creep_min") {
            new_config.rear_exit_forward_creep_min = value;
        } else if (name == "rear_exit_lateral_creep_min") {
            new_config.rear_exit_lateral_creep_min = value;
        } else if (name == "lidar_offset_x") {
            new_lidar_offset_x = value;
        } else if (name == "lidar_offset_y") {
            new_lidar_offset_y = value;
        } else if (name == "approach_to_align_dist") {
            new_config.approach_to_align_dist = value;
        } else if (name == "dock_start_dist") {
            new_config.dock_start_dist = value;
        } else if (name == "approach_max_vx") {
            new_config.approach_max_vx = value;
        } else if (name == "approach_max_vy") {
            new_config.approach_max_vy = value;
        } else if (name == "approach_max_wz") {
            new_config.approach_max_wz = value;
        } else if (name == "align_max_vx") {
            new_config.align_max_vx = value;
        } else if (name == "align_max_vy") {
            new_config.align_max_vy = value;
        } else if (name == "align_max_wz") {
            new_config.align_max_wz = value;
        } else if (name == "dock_max_vx") {
            new_config.dock_max_vx = value;
        } else if (name == "dock_max_vy") {
            new_config.dock_max_vy = value;
        } else if (name == "dock_max_wz") {
            new_config.dock_max_wz = value;
        } else if (name == "reverse_vx_scale") {
            new_config.reverse_vx_scale = value;
        } else if (name == "kp_forward") {
            new_config.kp_forward = value;
        } else if (name == "kp_bearing") {
            new_config.kp_bearing = value;
        } else if (name == "kp_final_yaw") {
            new_config.kp_final_yaw = value;
        } else if (name == "kp_lateral_to_yaw") {
            new_config.kp_lateral_to_yaw = value;
        } else if (name == "lateral_trim_gain") {
            new_config.lateral_trim_gain = value;
        } else if (name == "max_vy_trim") {
            new_config.max_vy_trim = value;
        } else if (name == "approach_final_yaw_weight") {
            new_config.approach_final_yaw_weight = value;
        } else if (name == "approach_lateral_yaw_comp_gain") {
            new_config.approach_lateral_yaw_comp_gain = value;
        } else if (name == "approach_lateral_yaw_comp_max") {
            new_config.approach_lateral_yaw_comp_max = value;
        } else if (name == "approach_lateral_vy_scale") {
            new_config.approach_lateral_vy_scale = value;
        } else if (name == "kp_dock_x") {
            new_config.kp_dock_x = value;
        } else if (name == "kp_dock_y") {
            new_config.kp_dock_y = value;
        } else if (name == "kp_dock_yaw") {
            new_config.kp_dock_yaw = value;
        } else if (name == "approach_creep_min") {
            new_config.approach_creep_min = value;
        } else if (name == "align_forward_creep_min") {
            new_config.align_forward_creep_min = value;
        } else if (name == "align_lateral_creep_min") {
            new_config.align_lateral_creep_min = value;
        } else if (name == "cross_band_align_max_vy") {
            new_config.cross_band_align_max_vy = value;
        } else if (name == "cross_band_align_lateral_creep_min") {
            new_config.cross_band_align_lateral_creep_min = value;
        } else if (name == "precise_lateral_align_lateral_creep_min") {
            new_config.precise_lateral_align_lateral_creep_min = value;
        } else if (name == "cross_band_approach_max_vx") {
            new_config.cross_band_approach_max_vx = value;
        } else if (name == "cross_band_align_max_vx") {
            new_config.cross_band_align_max_vx = value;
        } else if (name == "cross_band_align_forward_creep_min") {
            new_config.cross_band_align_forward_creep_min = value;
        } else if (name == "align_yaw_creep_min") {
            new_config.align_yaw_creep_min = value;
        } else if (name == "dock_creep_min") {
            new_config.dock_creep_min = value;
        } else if (name == "dock_forward_creep_min") {
            new_config.dock_forward_creep_min = value;
        } else if (name == "dock_lateral_creep_min") {
            new_config.dock_lateral_creep_min = value;
        } else if (name == "align_yaw_priority_threshold") {
            new_config.align_yaw_priority_threshold = value;
        } else if (name == "dock_yaw_priority_threshold") {
            new_config.dock_yaw_priority_threshold = value;
        } else if (name == "dock_position_only_yaw_threshold") {
            new_config.dock_position_only_yaw_threshold = value;
        } else if (name == "align_yaw_first_enter") {
            new_config.align_yaw_first_enter = value;
        } else if (name == "align_yaw_first_exit") {
            new_config.align_yaw_first_exit = value;
        } else if (name == "dock_yaw_first_enter") {
            new_config.dock_yaw_first_enter = value;
        } else if (name == "dock_yaw_first_exit") {
            new_config.dock_yaw_first_exit = value;
        } else if (name == "yaw_deadband") {
            new_config.yaw_deadband = value;
        } else if (name == "forward_only_bearing_threshold") {
            new_config.forward_only_bearing_threshold = value;
        } else if (name == "forward_only_bearing_exit_threshold") {
            new_config.forward_only_bearing_exit_threshold = value;
        } else if (name == "forward_only_kp_bearing") {
            new_config.forward_only_kp_bearing = value;
        } else if (name == "forward_only_max_vx") {
            new_config.forward_only_max_vx = value;
        } else if (name == "forward_only_forward_creep_min") {
            new_config.forward_only_forward_creep_min = value;
        } else if (name == "forward_only_yaw_creep_min") {
            new_config.forward_only_yaw_creep_min = value;
        } else if (name == "graph_reverse_yaw_threshold") {
            new_config.graph_reverse_yaw_threshold = value;
        } else if (name == "logistics_cmd_slew_vx_rate") {
            new_logistics_cmd_slew_vx_rate = value;
        } else if (name == "logistics_cmd_slew_wz_rate") {
            new_logistics_cmd_slew_wz_rate = value;
        }
    }

    if (new_config.align_yaw_first_enter < new_config.align_yaw_first_exit) {
        RCLCPP_WARN(this->get_logger(),
                    "align_yaw_first_enter %.3f is less than align_yaw_first_exit %.3f",
                    new_config.align_yaw_first_enter,
                    new_config.align_yaw_first_exit);
    }
    if (new_config.dock_yaw_first_enter < new_config.dock_yaw_first_exit) {
        RCLCPP_WARN(this->get_logger(),
                    "dock_yaw_first_enter %.3f is less than dock_yaw_first_exit %.3f",
                    new_config.dock_yaw_first_enter,
                    new_config.dock_yaw_first_exit);
    }

    kp_x_ = new_kp_x;
    kp_y_ = new_kp_y;
    kp_yaw_ = new_kp_yaw;
    kd_x_ = new_kd_x;
    kd_y_ = new_kd_y;
    kd_yaw_ = new_kd_yaw;
    lin_vel_creep_min_ = new_lin_vel_creep_min;
    max_vx_ = new_max_vx;
    max_vy_ = new_max_vy;
    max_dyaw_ = new_max_dyaw;
    yaw_first_threshold_ = new_yaw_first_threshold;
    arrival_position_tolerance_ = new_arrival_position_tolerance;
    arrival_yaw_tolerance_ = new_arrival_yaw_tolerance;
    transition_position_tolerance_ = new_transition_position_tolerance;
    transition_position_tolerance_x_ =
        new_transition_position_tolerance_x > 0.0
            ? new_transition_position_tolerance_x
            : new_transition_position_tolerance;
    transition_position_tolerance_y_ =
        new_transition_position_tolerance_y > 0.0
            ? new_transition_position_tolerance_y
            : new_transition_position_tolerance;
    transition_yaw_tolerance_ = new_transition_yaw_tolerance;
    manip_position_tolerance_ = new_manip_position_tolerance;
    manip_yaw_tolerance_ = new_manip_yaw_tolerance;
    arrival_stable_time_s_ = new_arrival_stable_time_s;
    transition_arrival_stable_time_s_ = new_transition_arrival_stable_time_s;
    cross_band_transition_position_tolerance_y_ =
        new_cross_band_transition_position_tolerance_y;
    cross_band_transition_arrival_stable_time_s_ =
        new_cross_band_transition_arrival_stable_time_s;
    transition_dist_arrival_tolerance_ = new_transition_dist_arrival_tolerance;
    manip_approach_arrival_tolerance_ = new_manip_approach_arrival_tolerance;
    manip_arrival_stable_time_s_ = new_manip_arrival_stable_time_s;
    precise_lateral_arrival_tolerance_y_ = new_precise_lateral_arrival_tolerance_y;
    precise_lateral_arrival_y_zone1_min_ = new_precise_lateral_arrival_y_zone1_min;
    precise_lateral_arrival_y_zone1_max_ = new_precise_lateral_arrival_y_zone1_max;
    rear_corridor_yaw_free_y_max_ = new_rear_corridor_yaw_free_y_max;
    rear_exit_creep_y_min_ = new_rear_exit_creep_y_min;
    rear_exit_creep_y_max_ = new_rear_exit_creep_y_max;
    new_config.rear_corridor_yaw_free_y_min = pickup_rear_staging_y_min_;
    new_config.rear_corridor_yaw_free_y_max = rear_corridor_yaw_free_y_max_;
    lidar_offset_x_ = new_lidar_offset_x;
    lidar_offset_y_ = new_lidar_offset_y;
    const bool slew_config_changed =
        logistics_cmd_slew_enable_ != new_logistics_cmd_slew_enable ||
        logistics_cmd_slew_vy_enable_ != new_logistics_cmd_slew_vy_enable ||
        logistics_cmd_slew_vx_rate_ != new_logistics_cmd_slew_vx_rate ||
        logistics_cmd_slew_wz_rate_ != new_logistics_cmd_slew_wz_rate;
    logistics_cmd_slew_enable_ = new_logistics_cmd_slew_enable;
    logistics_cmd_slew_vy_enable_ = new_logistics_cmd_slew_vy_enable;
    logistics_cmd_slew_vx_rate_ = new_logistics_cmd_slew_vx_rate;
    logistics_cmd_slew_wz_rate_ = new_logistics_cmd_slew_wz_rate;
    logistics_controller_config_ = new_config;
    logistics_controller_ =
        make_nav_controller(logistics_controller_type_, logistics_controller_config_);
    nav_pd_initialized_ = false;
    logistics_prev_ctrl_time_valid_ = false;
    if (slew_config_changed) {
        reset_logistics_cmd_slew();
    }
    reset_arrival_debounce();

    result.successful = true;
    return result;
}

void LidarNavControl::refresh_logistics_controller_config() {
    logistics_controller_config_.kp_x = kp_x_;
    logistics_controller_config_.kp_y = kp_y_;
    logistics_controller_config_.kp_yaw = kp_yaw_;
    logistics_controller_config_.kd_x = kd_x_;
    logistics_controller_config_.kd_y = kd_y_;
    logistics_controller_config_.kd_yaw = kd_yaw_;
    logistics_controller_config_.lin_vel_creep_min = lin_vel_creep_min_;
    logistics_controller_config_.max_vx = max_vx_;
    logistics_controller_config_.max_vy = max_vy_;
    logistics_controller_config_.max_wz = max_dyaw_;
    logistics_controller_config_.yaw_first_threshold = yaw_first_threshold_;
    logistics_controller_config_.approach_to_align_dist =
        this->get_parameter("approach_to_align_dist").as_double();
    logistics_controller_config_.dock_start_dist =
        this->get_parameter("dock_start_dist").as_double();
    logistics_controller_config_.approach_max_vx =
        this->get_parameter("approach_max_vx").as_double();
    logistics_controller_config_.approach_max_vy =
        this->get_parameter("approach_max_vy").as_double();
    logistics_controller_config_.approach_max_wz =
        this->get_parameter("approach_max_wz").as_double();
    logistics_controller_config_.align_max_vx =
        this->get_parameter("align_max_vx").as_double();
    logistics_controller_config_.align_max_vy =
        this->get_parameter("align_max_vy").as_double();
    logistics_controller_config_.align_max_wz =
        this->get_parameter("align_max_wz").as_double();
    logistics_controller_config_.dock_max_vx =
        this->get_parameter("dock_max_vx").as_double();
    logistics_controller_config_.dock_max_vy =
        this->get_parameter("dock_max_vy").as_double();
    logistics_controller_config_.dock_max_wz =
        this->get_parameter("dock_max_wz").as_double();
    logistics_controller_config_.reverse_vx_scale =
        this->get_parameter("reverse_vx_scale").as_double();
    logistics_controller_config_.kp_forward =
        this->get_parameter("kp_forward").as_double();
    logistics_controller_config_.kp_bearing =
        this->get_parameter("kp_bearing").as_double();
    logistics_controller_config_.kp_final_yaw =
        this->get_parameter("kp_final_yaw").as_double();
    logistics_controller_config_.kp_lateral_to_yaw =
        this->get_parameter("kp_lateral_to_yaw").as_double();
    logistics_controller_config_.kp_dock_x =
        this->get_parameter("kp_dock_x").as_double();
    logistics_controller_config_.kp_dock_y =
        this->get_parameter("kp_dock_y").as_double();
    logistics_controller_config_.kp_dock_yaw =
        this->get_parameter("kp_dock_yaw").as_double();
    logistics_controller_config_.lateral_trim_gain =
        this->get_parameter("lateral_trim_gain").as_double();
    logistics_controller_config_.max_vy_trim =
        this->get_parameter("max_vy_trim").as_double();
    logistics_controller_config_.approach_creep_min =
        this->get_parameter("approach_creep_min").as_double();
    logistics_controller_config_.align_forward_creep_min =
        this->get_parameter("align_forward_creep_min").as_double();
    logistics_controller_config_.align_lateral_creep_min =
        this->get_parameter("align_lateral_creep_min").as_double();
    logistics_controller_config_.cross_band_align_max_vy =
        this->get_parameter("cross_band_align_max_vy").as_double();
    logistics_controller_config_.cross_band_align_lateral_creep_min =
        this->get_parameter("cross_band_align_lateral_creep_min").as_double();
    logistics_controller_config_.precise_lateral_align_lateral_creep_min =
        this->get_parameter("precise_lateral_align_lateral_creep_min").as_double();
    logistics_controller_config_.cross_band_approach_max_vx =
        this->get_parameter("cross_band_approach_max_vx").as_double();
    logistics_controller_config_.cross_band_align_max_vx =
        this->get_parameter("cross_band_align_max_vx").as_double();
    logistics_controller_config_.cross_band_align_forward_creep_min =
        this->get_parameter("cross_band_align_forward_creep_min").as_double();
    logistics_controller_config_.align_yaw_creep_min =
        this->get_parameter("align_yaw_creep_min").as_double();
    logistics_controller_config_.dock_creep_min =
        this->get_parameter("dock_creep_min").as_double();
    logistics_controller_config_.dock_forward_creep_min =
        this->get_parameter("dock_forward_creep_min").as_double();
    logistics_controller_config_.dock_lateral_creep_min =
        this->get_parameter("dock_lateral_creep_min").as_double();
    logistics_controller_config_.approach_final_yaw_weight =
        this->get_parameter("approach_final_yaw_weight").as_double();
    logistics_controller_config_.approach_lateral_yaw_comp_enable =
        this->get_parameter("approach_lateral_yaw_comp_enable").as_bool();
    logistics_controller_config_.approach_lateral_yaw_comp_gain =
        this->get_parameter("approach_lateral_yaw_comp_gain").as_double();
    logistics_controller_config_.approach_lateral_yaw_comp_max =
        this->get_parameter("approach_lateral_yaw_comp_max").as_double();
    logistics_controller_config_.approach_lateral_vy_scale =
        this->get_parameter("approach_lateral_vy_scale").as_double();
    logistics_controller_config_.align_yaw_priority_threshold =
        this->get_parameter("align_yaw_priority_threshold").as_double();
    logistics_controller_config_.dock_yaw_priority_threshold =
        this->get_parameter("dock_yaw_priority_threshold").as_double();
    logistics_controller_config_.dock_position_only_disable_yaw =
        this->get_parameter("dock_position_only_disable_yaw").as_bool();
    logistics_controller_config_.dock_position_only_yaw_threshold =
        this->get_parameter("dock_position_only_yaw_threshold").as_double();
    logistics_controller_config_.align_yaw_first_enter =
        this->get_parameter("align_yaw_first_enter").as_double();
    logistics_controller_config_.align_yaw_first_exit =
        this->get_parameter("align_yaw_first_exit").as_double();
    logistics_controller_config_.dock_yaw_first_enter =
        this->get_parameter("dock_yaw_first_enter").as_double();
    logistics_controller_config_.dock_yaw_first_exit =
        this->get_parameter("dock_yaw_first_exit").as_double();
    logistics_controller_config_.yaw_deadband =
        this->get_parameter("yaw_deadband").as_double();
    logistics_controller_config_.rear_corridor_yaw_free_y_min = pickup_rear_staging_y_min_;
    logistics_controller_config_.rear_corridor_yaw_free_y_max = rear_corridor_yaw_free_y_max_;
    logistics_controller_config_.rear_exit_forward_creep_min =
        this->get_parameter("rear_exit_forward_creep_min").as_double();
    logistics_controller_config_.rear_exit_lateral_creep_min =
        this->get_parameter("rear_exit_lateral_creep_min").as_double();
    logistics_controller_config_.forward_only_transition_enable =
        this->get_parameter("forward_only_transition_enable").as_bool();
    logistics_controller_config_.forward_only_bearing_threshold =
        this->get_parameter("forward_only_bearing_threshold").as_double();
    logistics_controller_config_.forward_only_bearing_exit_threshold =
        this->get_parameter("forward_only_bearing_exit_threshold").as_double();
    logistics_controller_config_.forward_only_kp_bearing =
        this->get_parameter("forward_only_kp_bearing").as_double();
    logistics_controller_config_.forward_only_max_vx =
        this->get_parameter("forward_only_max_vx").as_double();
    logistics_controller_config_.forward_only_forward_creep_min =
        this->get_parameter("forward_only_forward_creep_min").as_double();
    logistics_controller_config_.forward_only_yaw_creep_min =
        this->get_parameter("forward_only_yaw_creep_min").as_double();
    logistics_controller_config_.graph_reverse_yaw_threshold =
        this->get_parameter("graph_reverse_yaw_threshold").as_double();
    logistics_controller_config_.direct_nearest_max_vx = direct_nearest_max_vx_;
    logistics_controller_config_.direct_manip_dock_handoff_dist =
        this->get_parameter("direct_manip_dock_handoff_dist").as_double();
}

bool LidarNavControl::is_dynamic_nav_param(const std::string& name) const {
    static const std::unordered_set<std::string> kDynamicParams = {
        "kp_x",
        "kp_y",
        "kp_yaw",
        "kd_x",
        "kd_y",
        "kd_yaw",
        "lin_vel_creep_min",
        "max_vx",
        "max_vy",
        "max_dyaw",
        "yaw_first_threshold",
        "approach_to_align_dist",
        "dock_start_dist",
        "approach_max_vx",
        "approach_max_vy",
        "approach_max_wz",
        "align_max_vx",
        "align_max_vy",
        "align_max_wz",
        "dock_max_vx",
        "dock_max_vy",
        "dock_max_wz",
        "reverse_vx_scale",
        "kp_forward",
        "kp_bearing",
        "kp_final_yaw",
        "kp_lateral_to_yaw",
        "lateral_trim_gain",
        "max_vy_trim",
        "approach_final_yaw_weight",
        "approach_lateral_yaw_comp_enable",
        "approach_lateral_yaw_comp_gain",
        "approach_lateral_yaw_comp_max",
        "approach_lateral_vy_scale",
        "kp_dock_x",
        "kp_dock_y",
        "kp_dock_yaw",
        "approach_creep_min",
        "align_forward_creep_min",
        "align_lateral_creep_min",
        "cross_band_align_max_vy",
        "cross_band_align_lateral_creep_min",
        "precise_lateral_align_lateral_creep_min",
        "cross_band_approach_max_vx",
        "cross_band_align_max_vx",
        "cross_band_align_forward_creep_min",
        "align_yaw_creep_min",
        "dock_creep_min",
        "dock_forward_creep_min",
        "dock_lateral_creep_min",
        "align_yaw_priority_threshold",
        "dock_yaw_priority_threshold",
        "dock_position_only_disable_yaw",
        "dock_position_only_yaw_threshold",
        "align_yaw_first_enter",
        "align_yaw_first_exit",
        "dock_yaw_first_enter",
        "dock_yaw_first_exit",
        "yaw_deadband",
        "forward_only_bearing_threshold",
        "forward_only_bearing_exit_threshold",
        "forward_only_kp_bearing",
        "forward_only_max_vx",
        "forward_only_forward_creep_min",
        "forward_only_yaw_creep_min",
        "graph_reverse_yaw_threshold",
        "logistics_cmd_slew_enable",
        "logistics_cmd_slew_vx_rate",
        "logistics_cmd_slew_wz_rate",
        "logistics_cmd_slew_vy_enable",
        "arrival_position_tolerance",
        "arrival_yaw_tolerance",
        "transition_position_tolerance",
        "transition_position_tolerance_x",
        "transition_position_tolerance_y",
        "transition_yaw_tolerance",
        "manip_position_tolerance",
        "manip_yaw_tolerance",
        "arrival_stable_time_s",
        "transition_arrival_stable_time_s",
        "cross_band_transition_position_tolerance_y",
        "cross_band_transition_arrival_stable_time_s",
        "transition_dist_arrival_tolerance",
        "manip_approach_arrival_tolerance",
        "manip_arrival_stable_time_s",
        "precise_lateral_arrival_tolerance_y",
        "precise_lateral_arrival_y_zone1_min",
        "precise_lateral_arrival_y_zone1_max",
        "rear_corridor_yaw_free_y_max",
        "rear_exit_creep_y_min",
        "rear_exit_creep_y_max",
        "rear_exit_forward_creep_min",
        "rear_exit_lateral_creep_min",
        "lidar_offset_x",
        "lidar_offset_y",
    };
    return kDynamicParams.count(name) > 0;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarNavControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
