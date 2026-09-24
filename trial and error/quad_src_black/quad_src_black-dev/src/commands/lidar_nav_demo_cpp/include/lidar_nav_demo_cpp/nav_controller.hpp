#ifndef LIDAR_NAV_DEMO_CPP_NAV_CONTROLLER_HPP_
#define LIDAR_NAV_DEMO_CPP_NAV_CONTROLLER_HPP_

#include "geometry_msgs/msg/twist.hpp"

#include <cstddef>
#include <memory>
#include <string>

enum class WaypointType {
    START,
    TRANSITION,
    PICKUP,
    DROPOFF
};

enum class NavControlStage {
    BODY_PD,
    FORWARD_ONLY,
    APPROACH,
    ALIGN_YAW,
    ALIGN,
    DOCK_YAW,
    DOCK
};

struct NavControllerConfig {
    double kp_x{0.5};
    double kp_y{0.5};
    double kp_yaw{1.0};
    double kd_x{0.0};
    double kd_y{0.0};
    double kd_yaw{0.0};
    double lin_vel_creep_min{0.0};
    double max_vx{0.5};
    double max_vy{0.5};
    double max_wz{0.5};
    double yaw_first_threshold{0.35};

    double approach_to_align_dist{0.90};
    double dock_start_dist{0.10};

    double approach_max_vx{0.45};
    double approach_max_vy{0.06};
    double approach_max_wz{0.60};
    double align_max_vx{0.40};
    double align_max_vy{0.22};
    double align_max_wz{0.80};
    double dock_max_vx{0.25};
    double dock_max_vy{0.12};
    double dock_max_wz{0.45};
    double reverse_vx_scale{0.5};

    double kp_forward{1.1};
    double kp_bearing{1.2};
    double kp_final_yaw{1.0};
    double kp_lateral_to_yaw{0.6};
    double kp_dock_x{2.0};
    double kp_dock_y{1.5};
    double kp_dock_yaw{1.2};
    double lateral_trim_gain{0.75};
    double max_vy_trim{0.22};
    double approach_creep_min{0.18};
    double align_forward_creep_min{0.20};
    double align_lateral_creep_min{0.18};
    double cross_band_align_max_vy{0.22};
    double cross_band_align_lateral_creep_min{0.30};
    double precise_lateral_align_lateral_creep_min{0.30};
    double cross_band_approach_max_vx{0.50};
    double cross_band_align_max_vx{0.35};
    double cross_band_align_forward_creep_min{0.20};
    double align_yaw_creep_min{0.30};
    double dock_creep_min{0.12};
    double dock_forward_creep_min{0.08};
    double dock_lateral_creep_min{0.12};
    double approach_final_yaw_weight{0.8};
    bool approach_lateral_yaw_comp_enable{true};
    double approach_lateral_yaw_comp_gain{0.45};
    double approach_lateral_yaw_comp_max{0.30};
    double approach_lateral_vy_scale{0.25};
    double align_yaw_priority_threshold{0.8};
    double dock_yaw_priority_threshold{1.10};
    bool dock_position_only_disable_yaw{true};
    double dock_position_only_yaw_threshold{0.10};
    double align_yaw_first_enter{0.35};
    double align_yaw_first_exit{0.20};
    double dock_yaw_first_enter{0.25};
    double dock_yaw_first_exit{0.12};
    double yaw_deadband{0.05};
    double rear_corridor_yaw_free_y_min{3.5};
    double rear_corridor_yaw_free_y_max{4.5};
    double rear_exit_forward_creep_min{0.20};
    double rear_exit_lateral_creep_min{0.30};

    bool forward_only_transition_enable{true};
    double forward_only_bearing_threshold{0.20};
    double forward_only_bearing_exit_threshold{0.10};
    double forward_only_kp_bearing{1.2};
    double forward_only_max_vx{0.55};
    double forward_only_forward_creep_min{0.18};
    double forward_only_yaw_creep_min{0.25};
    double graph_reverse_yaw_threshold{1.0};
    double direct_nearest_max_vx{0.90};
    double direct_manip_dock_handoff_dist{0.30};
};

struct NavControllerInput {
    double err_x_body{0.0};
    double err_y_body{0.0};
    double err_yaw{0.0};
    double dist{0.0};
    double robot_y{0.0};
    double dt{0.02};
    std::size_t track_id{0};
    WaypointType waypoint_type{WaypointType::TRANSITION};
    bool cross_isolation_band{false};
    bool tight_lateral_zone{false};
    bool precise_lateral_zone{false};
    bool corridor_lateral_only_arrival{false};
    bool manip_approach_arrival{false};
    bool manip_to_nearest_arrival{false};
    bool post_manip_exit{false};
    bool direct_nearest_approach{false};
    bool rear_exit_creep_zone{false};
    double target_y{0.0};
    double target_yaw{0.0};
    bool enabled{true};
};

struct NavControllerOutput {
    geometry_msgs::msg::Twist cmd;
    NavControlStage stage{NavControlStage::BODY_PD};
    double bearing_error{0.0};
};

class NavController {
public:
    explicit NavController(NavControllerConfig config);
    virtual ~NavController() = default;

    virtual NavControllerOutput compute(const NavControllerInput& input) = 0;
    virtual const char* name() const = 0;
    virtual void reset();

protected:
    struct ErrorDerivative {
        double dx{0.0};
        double dy{0.0};
        double dyaw{0.0};
    };

    ErrorDerivative update_derivative(const NavControllerInput& input);
    static double normalize_angle(double angle);
    static double clamp(double value, double min_value, double max_value);
    static void apply_creep(double error, double creep_min, double eps, double& value);

    NavControllerConfig config_;
    bool initialized_{false};
    std::size_t track_id_{0};
    double prev_err_x_body_{0.0};
    double prev_err_y_body_{0.0};
    double prev_err_yaw_{0.0};
};

class BodyPdNavController : public NavController {
public:
    explicit BodyPdNavController(NavControllerConfig config);
    NavControllerOutput compute(const NavControllerInput& input) override;
    const char* name() const override;
};

class HeadingDockNavController : public NavController {
public:
    explicit HeadingDockNavController(NavControllerConfig config);
    NavControllerOutput compute(const NavControllerInput& input) override;
    const char* name() const override;
    void reset() override;
};

std::unique_ptr<NavController> make_nav_controller(
    const std::string& controller_type,
    const NavControllerConfig& config);

const char* nav_control_stage_to_string(NavControlStage stage);

#endif  // LIDAR_NAV_DEMO_CPP_NAV_CONTROLLER_HPP_
