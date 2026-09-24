#include "lidar_nav_demo_cpp/isolation_band.hpp"
#include "lidar_nav_demo_cpp/nav_controller.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kDerivativeDtMin = 1e-4;
constexpr double kDerivativeDtMax = 0.25;
constexpr double kSmallErrorEps = 5e-3;
constexpr double kDockForwardCreepError = 0.04;
constexpr double kDockLateralCreepError = 0.02;

bool is_manip_waypoint(WaypointType type) {
    return type == WaypointType::PICKUP || type == WaypointType::DROPOFF;
}

bool suppress_rear_corridor_yaw(
    const NavControllerInput& input, const NavControllerConfig& config)
{
    return input.waypoint_type == WaypointType::TRANSITION &&
           lidar_nav::is_in_rear_corridor_yaw_free_zone(
               input.robot_y,
               config.rear_corridor_yaw_free_y_min,
               config.rear_corridor_yaw_free_y_max);
}

double clamp_forward_reverse_vx(
    double vx, double forward_limit, const NavControllerConfig& config)
{
    const double max_forward = std::max(0.0, forward_limit);
    const double max_reverse = std::max(0.0, config.reverse_vx_scale) * max_forward;
    return std::max(-max_reverse, std::min(vx, max_forward));
}

NavControllerOutput compute_forward_only_transition(
    const NavControllerInput& input, const NavControllerConfig& config)
{
    NavControllerOutput output;
    output.stage = NavControlStage::FORWARD_ONLY;

    // Real-time bearing: body-frame angle from current heading to the target point.
    const double bearing = std::atan2(input.err_y_body, input.err_x_body);
    output.bearing_error = bearing;

    const double max_vx = input.direct_nearest_approach
        ? config.direct_nearest_max_vx
        : config.forward_only_max_vx;
    const double max_wz = config.approach_max_wz;

    // Longitudinal only: forward component of range-to-target (no lateral trim).
    double vx = config.kp_forward * std::max(0.0, input.err_x_body);

    // Yaw target tracks bearing to the target point, not the waypoint final yaw.
    double wz = 0.0;
    if (std::abs(bearing) >= config.yaw_deadband) {
        wz = config.forward_only_kp_bearing * bearing;
        if (config.forward_only_yaw_creep_min > 0.0 &&
            std::abs(wz) < config.forward_only_yaw_creep_min) {
            wz = std::copysign(config.forward_only_yaw_creep_min, bearing);
        }
    }

    // Apply forward creep only when the target is roughly ahead.
    const double heading_alignment = std::cos(bearing);
    if (config.forward_only_forward_creep_min > 0.0 &&
        input.dist > kSmallErrorEps &&
        heading_alignment > 0.2 &&
        vx > 0.0 &&
        vx < config.forward_only_forward_creep_min) {
        vx = config.forward_only_forward_creep_min;
    }

    output.cmd.linear.x = clamp_forward_reverse_vx(vx, max_vx, config);
    output.cmd.linear.y = 0.0;
    output.cmd.angular.z = std::max(-max_wz, std::min(wz, max_wz));
    return output;
}

}  // namespace

NavController::NavController(NavControllerConfig config)
    : config_(config)
{
}

void NavController::reset() {
    initialized_ = false;
}

NavController::ErrorDerivative NavController::update_derivative(
    const NavControllerInput& input)
{
    if (!initialized_ || track_id_ != input.track_id) {
        track_id_ = input.track_id;
        prev_err_x_body_ = input.err_x_body;
        prev_err_y_body_ = input.err_y_body;
        prev_err_yaw_ = input.err_yaw;
        initialized_ = true;
        return {};
    }

    const double dt = clamp(input.dt, kDerivativeDtMin, kDerivativeDtMax);
    ErrorDerivative derivative;
    derivative.dx = (input.err_x_body - prev_err_x_body_) / dt;
    derivative.dy = (input.err_y_body - prev_err_y_body_) / dt;
    derivative.dyaw = normalize_angle(input.err_yaw - prev_err_yaw_) / dt;

    prev_err_x_body_ = input.err_x_body;
    prev_err_y_body_ = input.err_y_body;
    prev_err_yaw_ = input.err_yaw;
    return derivative;
}

double NavController::normalize_angle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double NavController::clamp(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(value, max_value));
}

void NavController::apply_creep(
    double error, double creep_min, double eps, double& value)
{
    if (creep_min <= 0.0 || std::abs(error) <= eps || std::abs(value) >= creep_min) {
        return;
    }
    value = std::copysign(creep_min, error);
}

BodyPdNavController::BodyPdNavController(NavControllerConfig config)
    : NavController(config)
{
}

const char* BodyPdNavController::name() const {
    return "body_pd";
}

NavControllerOutput BodyPdNavController::compute(const NavControllerInput& input) {
    NavControllerOutput output;
    output.stage = NavControlStage::BODY_PD;

    if (!input.enabled) {
        reset();
        return output;
    }

    const ErrorDerivative derivative = update_derivative(input);
    const bool suppress_yaw = suppress_rear_corridor_yaw(input, config_);

    if (std::abs(input.err_yaw) > config_.yaw_first_threshold && !suppress_yaw) {
        const double wz = config_.kp_yaw * input.err_yaw + config_.kd_yaw * derivative.dyaw;
        output.cmd.angular.z = clamp(wz, -config_.max_wz, config_.max_wz);
        return output;
    }

    double vx = config_.kp_x * input.err_x_body + config_.kd_x * derivative.dx;
    double vy = config_.kp_y * input.err_y_body + config_.kd_y * derivative.dy;
    apply_creep(input.err_x_body, config_.lin_vel_creep_min, kSmallErrorEps, vx);
    apply_creep(input.err_y_body, config_.lin_vel_creep_min, kSmallErrorEps, vy);

    const double wz = suppress_yaw
        ? 0.0
        : config_.kp_yaw * input.err_yaw + config_.kd_yaw * derivative.dyaw;
    output.cmd.linear.x = clamp(vx, -config_.max_vx, config_.max_vx);
    output.cmd.linear.y = clamp(vy, -config_.max_vy, config_.max_vy);
    output.cmd.angular.z = clamp(wz, -config_.max_wz, config_.max_wz);
    return output;
}

HeadingDockNavController::HeadingDockNavController(NavControllerConfig config)
    : NavController(config)
{
}

const char* HeadingDockNavController::name() const {
    return "heading_dock";
}

void HeadingDockNavController::reset() {
    NavController::reset();
}

NavControllerOutput HeadingDockNavController::compute(const NavControllerInput& input) {
    NavControllerOutput output;

    if (!input.enabled) {
        reset();
        return output;
    }

    update_derivative(input);

    const bool direct_manip_handoff =
        input.direct_nearest_approach &&
        is_manip_waypoint(input.waypoint_type) &&
        input.dist <= config_.direct_manip_dock_handoff_dist;

    if (!direct_manip_handoff &&
        (input.direct_nearest_approach ||
         (input.waypoint_type == WaypointType::TRANSITION &&
          !input.manip_approach_arrival &&
          !input.manip_to_nearest_arrival &&
          !input.post_manip_exit &&
          config_.forward_only_transition_enable))) {
        return compute_forward_only_transition(input, config_);
    }

    const bool require_final_yaw = is_manip_waypoint(input.waypoint_type);
    const bool cross_band_segment_end_allow_yaw =
        input.waypoint_type == WaypointType::TRANSITION &&
        input.cross_isolation_band &&
        !input.corridor_lateral_only_arrival &&
        input.target_y > config_.rear_corridor_yaw_free_y_min + 1e-9;
    const bool suppress_yaw =
        input.corridor_lateral_only_arrival ||
        input.manip_approach_arrival ||
        input.manip_to_nearest_arrival ||
        cross_band_segment_end_allow_yaw
            ? false
            : (suppress_rear_corridor_yaw(input, config_) ||
               (input.waypoint_type == WaypointType::TRANSITION &&
                input.cross_isolation_band));

    if (input.waypoint_type == WaypointType::TRANSITION) {
        const bool cross_band = input.cross_isolation_band;
        const bool tight_lateral = input.tight_lateral_zone && !cross_band;
        const double transition_align_max_vy = cross_band
            ? config_.cross_band_align_max_vy
            : (tight_lateral ? config_.cross_band_align_max_vy : config_.align_max_vy);
        const double transition_lateral_creep_min = input.rear_exit_creep_zone
            ? config_.rear_exit_lateral_creep_min
            : (input.precise_lateral_zone
                ? config_.precise_lateral_align_lateral_creep_min
                : (cross_band ? config_.cross_band_align_lateral_creep_min
                              : config_.align_lateral_creep_min));
        const double transition_approach_max_vx = cross_band
            ? config_.cross_band_approach_max_vx
            : config_.approach_max_vx;
        const double transition_align_max_vx = cross_band
            ? config_.cross_band_align_max_vx
            : config_.align_max_vx;
        const double transition_forward_creep_min = input.rear_exit_creep_zone
            ? config_.rear_exit_forward_creep_min
            : (cross_band ? config_.cross_band_align_forward_creep_min
                          : config_.align_forward_creep_min);
        const double transition_approach_forward_creep_min = input.rear_exit_creep_zone
            ? config_.rear_exit_forward_creep_min
            : config_.approach_creep_min;

        double max_vx = config_.approach_max_vx;
        double max_vy = config_.approach_max_vy;
        double max_wz = config_.approach_max_wz;

        if (input.dist > config_.approach_to_align_dist) {
            output.stage = NavControlStage::APPROACH;
            max_vx = transition_approach_max_vx;
            // Lateral-dominant targets cannot shrink dist below the ALIGN
            // threshold with approach_max_vy alone; allow full trim speed.
            if (std::abs(input.err_y_body) > std::abs(input.err_x_body)) {
                max_vy = std::min(transition_align_max_vy, config_.max_vy_trim);
            }
        } else {
            output.stage = NavControlStage::ALIGN;
            max_vx = transition_align_max_vx;
            max_vy = std::min(transition_align_max_vy, config_.max_vy_trim);
            max_wz = config_.align_max_wz;
        }

        double vx = config_.kp_forward * input.err_x_body;
        double vy = config_.lateral_trim_gain * input.err_y_body;
        double wz = suppress_yaw ? 0.0 : config_.kp_final_yaw * input.err_yaw;

        if (std::abs(input.err_x_body) > kSmallErrorEps) {
            apply_creep(
                input.err_x_body,
                output.stage == NavControlStage::APPROACH
                    ? transition_approach_forward_creep_min
                    : transition_forward_creep_min,
                kSmallErrorEps,
                vx);
        }
        if (std::abs(input.err_y_body) > kSmallErrorEps) {
            apply_creep(
                input.err_y_body,
                transition_lateral_creep_min,
                kSmallErrorEps,
                vy);
        }

        output.cmd.linear.x = clamp_forward_reverse_vx(vx, max_vx, config_);
        output.cmd.linear.y = clamp(vy, -max_vy, max_vy);
        output.cmd.angular.z = clamp(wz, -max_wz, max_wz);
        return output;
    }

    if (input.dist > config_.approach_to_align_dist) {
        output.stage = NavControlStage::APPROACH;

        double vx = config_.kp_forward * input.err_x_body;
        const double lateral_vy_scale =
            require_final_yaw && config_.approach_lateral_yaw_comp_enable
                ? config_.approach_lateral_vy_scale
                : 1.0;
        double vy = lateral_vy_scale * config_.lateral_trim_gain * input.err_y_body;
        const double abs_yaw = require_final_yaw ? std::abs(input.err_yaw) : 0.0;
        const double yaw_for_control =
            require_final_yaw && abs_yaw >= config_.yaw_deadband ? input.err_yaw : 0.0;
        const double lateral_yaw_comp =
            require_final_yaw && config_.approach_lateral_yaw_comp_enable
                ? clamp(
                    config_.approach_lateral_yaw_comp_gain * input.err_y_body,
                    -config_.approach_lateral_yaw_comp_max,
                    config_.approach_lateral_yaw_comp_max)
                : 0.0;
        const double wz = config_.approach_final_yaw_weight *
            config_.kp_final_yaw * (yaw_for_control + lateral_yaw_comp);

        if (std::abs(input.err_x_body) > kSmallErrorEps) {
            apply_creep(
                input.err_x_body,
                config_.approach_creep_min,
                kSmallErrorEps,
                vx);
        }
        if (std::abs(input.err_y_body) > kSmallErrorEps) {
            apply_creep(
                input.err_y_body,
                lateral_vy_scale * config_.align_lateral_creep_min,
                kSmallErrorEps,
                vy);
        }

        output.cmd.linear.x =
            clamp_forward_reverse_vx(vx, config_.approach_max_vx, config_);
        output.cmd.linear.y = clamp(vy, -config_.approach_max_vy, config_.approach_max_vy);
        output.cmd.angular.z = clamp(wz, -config_.approach_max_wz, config_.approach_max_wz);
        return output;
    }

    if (input.dist > config_.dock_start_dist) {
        output.stage = NavControlStage::ALIGN;

        const double abs_yaw = require_final_yaw ? std::abs(input.err_yaw) : 0.0;
        const double yaw_for_control =
            require_final_yaw && abs_yaw >= config_.yaw_deadband ? input.err_yaw : 0.0;

        const double max_trim_vy = std::min(config_.align_max_vy, config_.max_vy_trim);
        const double yaw_linear_scale = clamp(
            1.0 - abs_yaw / std::max(config_.align_yaw_priority_threshold, kSmallErrorEps),
            0.35,
            1.0);
        double vx = config_.kp_forward * input.err_x_body;
        double vy = config_.lateral_trim_gain * input.err_y_body;
        const double wz = config_.kp_final_yaw * yaw_for_control;

        vx *= yaw_linear_scale;
        vy *= clamp(yaw_linear_scale, 0.85, 1.0);

        if (std::abs(input.err_x_body) > 0.04) {
            apply_creep(
                input.err_x_body,
                config_.align_forward_creep_min,
                kSmallErrorEps,
                vx);
        }
        if (std::abs(input.err_y_body) > 0.04) {
            apply_creep(
                input.err_y_body,
                config_.align_lateral_creep_min,
                kSmallErrorEps,
                vy);
        }

        output.cmd.linear.x = clamp_forward_reverse_vx(vx, config_.align_max_vx, config_);
        output.cmd.linear.y = clamp(vy, -max_trim_vy, max_trim_vy);
        output.cmd.angular.z = clamp(wz, -config_.align_max_wz, config_.align_max_wz);
        return output;
    }

    if (!require_final_yaw) {
        output.stage = NavControlStage::ALIGN;
        const double max_trim_vy = std::min(config_.align_max_vy, config_.max_vy_trim);
        double vx = config_.kp_forward * input.err_x_body;
        double vy = config_.lateral_trim_gain * input.err_y_body;
        const double wz = config_.kp_final_yaw * input.err_yaw;

        if (std::abs(input.err_x_body) > 0.04) {
            apply_creep(
                input.err_x_body,
                config_.align_forward_creep_min,
                kSmallErrorEps,
                vx);
        }
        if (std::abs(input.err_y_body) > 0.04) {
            apply_creep(
                input.err_y_body,
                config_.align_lateral_creep_min,
                kSmallErrorEps,
                vy);
        }

        output.cmd.linear.x = clamp_forward_reverse_vx(vx, config_.align_max_vx, config_);
        output.cmd.linear.y = clamp(vy, -max_trim_vy, max_trim_vy);
        output.cmd.angular.z = clamp(wz, -config_.align_max_wz, config_.align_max_wz);
        return output;
    }

    const double abs_yaw = std::abs(input.err_yaw);
    if (config_.dock_position_only_disable_yaw &&
        abs_yaw > config_.dock_position_only_yaw_threshold) {
        output.stage = NavControlStage::DOCK_YAW;

        const double yaw_for_control =
            abs_yaw >= config_.yaw_deadband ? input.err_yaw : 0.0;
        const double yaw_linear_scale = clamp(
            1.0 - abs_yaw / std::max(config_.dock_yaw_priority_threshold, kSmallErrorEps),
            0.35,
            1.0);
        double vx = config_.kp_forward * input.err_x_body * yaw_linear_scale;
        double vy = config_.lateral_trim_gain * input.err_y_body * yaw_linear_scale;
        double wz = config_.kp_final_yaw * yaw_for_control;

        if (std::abs(input.err_x_body) > kDockForwardCreepError) {
            apply_creep(
                input.err_x_body,
                config_.dock_forward_creep_min,
                kSmallErrorEps,
                vx);
        }
        if (std::abs(input.err_y_body) > kDockLateralCreepError) {
            apply_creep(
                input.err_y_body,
                config_.dock_lateral_creep_min,
                kSmallErrorEps,
                vy);
        }

        output.cmd.linear.x = clamp_forward_reverse_vx(vx, config_.dock_max_vx, config_);
        output.cmd.linear.y = clamp(vy, -config_.dock_max_vy, config_.dock_max_vy);
        output.cmd.angular.z = clamp(wz, -config_.dock_max_wz, config_.dock_max_wz);
        return output;
    }

    output.stage = NavControlStage::DOCK;
    double vx = config_.kp_dock_x * input.err_x_body;
    double vy = config_.kp_dock_y * input.err_y_body;
    const bool position_only_dock =
        config_.dock_position_only_disable_yaw &&
        abs_yaw <= config_.dock_position_only_yaw_threshold;
    const double yaw_for_control =
        abs_yaw >= config_.yaw_deadband ? input.err_yaw : 0.0;
    double wz = position_only_dock ? 0.0 : config_.kp_dock_yaw * yaw_for_control;

    if (!position_only_dock) {
        const double yaw_linear_scale = clamp(
            1.0 - abs_yaw / std::max(config_.dock_yaw_priority_threshold, kSmallErrorEps),
            0.35,
            1.0);
        vx *= yaw_linear_scale;
        vy *= yaw_linear_scale;
    }

    if (std::abs(input.err_x_body) > kDockForwardCreepError) {
        apply_creep(
            input.err_x_body,
            config_.dock_forward_creep_min,
            kSmallErrorEps,
            vx);
    }
    if (std::abs(input.err_y_body) > kDockLateralCreepError) {
        apply_creep(
            input.err_y_body,
            config_.dock_lateral_creep_min,
            kSmallErrorEps,
            vy);
    }

    output.cmd.linear.x = clamp_forward_reverse_vx(vx, config_.dock_max_vx, config_);
    output.cmd.linear.y = clamp(vy, -config_.dock_max_vy, config_.dock_max_vy);
    output.cmd.angular.z = clamp(wz, -config_.dock_max_wz, config_.dock_max_wz);
    return output;
}

std::unique_ptr<NavController> make_nav_controller(
    const std::string& controller_type,
    const NavControllerConfig& config)
{
    if (controller_type == "body_pd") {
        return std::make_unique<BodyPdNavController>(config);
    }
    return std::make_unique<HeadingDockNavController>(config);
}

const char* nav_control_stage_to_string(NavControlStage stage) {
    switch (stage) {
        case NavControlStage::BODY_PD:
            return "BODY_PD";
        case NavControlStage::FORWARD_ONLY:
            return "FORWARD_ONLY";
        case NavControlStage::APPROACH:
            return "APPROACH";
        case NavControlStage::ALIGN_YAW:
            return "ALIGN_YAW";
        case NavControlStage::ALIGN:
            return "ALIGN";
        case NavControlStage::DOCK_YAW:
            return "DOCK_YAW";
        case NavControlStage::DOCK:
            return "DOCK";
    }
    return "UNKNOWN";
}
