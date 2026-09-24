#include "lidar_nav_demo_cpp/lidar_nav_demo_node.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

double LidarNavControl::normalize_angle(double angle) const {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

geometry_msgs::msg::Twist LidarNavControl::limit_logistics_cmd_slew(
    const geometry_msgs::msg::Twist& raw_cmd, double dt)
{
    if (!logistics_cmd_slew_enable_) {
        last_logistics_cmd_ = raw_cmd;
        logistics_cmd_slew_initialized_ = true;
        return raw_cmd;
    }

    const double safe_dt = std::clamp(
        std::isfinite(dt) ? dt : (1.0 / std::max(control_rate_, 1.0)),
        0.005,
        0.05);
    const geometry_msgs::msg::Twist prev =
        logistics_cmd_slew_initialized_ ? last_logistics_cmd_
                                        : geometry_msgs::msg::Twist{};

    auto limit_axis = [safe_dt](double target, double previous, double rate) {
        const double max_delta = std::max(0.0, rate) * safe_dt;
        return std::clamp(target, previous - max_delta, previous + max_delta);
    };

    geometry_msgs::msg::Twist limited = raw_cmd;
    limited.linear.x = limit_axis(
        raw_cmd.linear.x, prev.linear.x, logistics_cmd_slew_vx_rate_);
    if (logistics_cmd_slew_vy_enable_) {
        limited.linear.y = limit_axis(
            raw_cmd.linear.y, prev.linear.y, logistics_cmd_slew_vx_rate_);
    }
    limited.angular.z = limit_axis(
        raw_cmd.angular.z, prev.angular.z, logistics_cmd_slew_wz_rate_);

    last_logistics_cmd_ = limited;
    logistics_cmd_slew_initialized_ = true;
    return limited;
}

void LidarNavControl::publish_logistics_cmd(
    const geometry_msgs::msg::Twist& raw_cmd, double dt)
{
    cmd_vel_pub_->publish(limit_logistics_cmd_slew(raw_cmd, dt));
}

void LidarNavControl::reset_logistics_cmd_slew() {
    logistics_cmd_slew_initialized_ = false;
    last_logistics_cmd_ = geometry_msgs::msg::Twist{};
}

bool LidarNavControl::handle_pre_path_spin() {
    const auto now = this->get_clock()->now();
    geometry_msgs::msg::Twist cmd;

    if (pre_scan_waiting_resume_) {
        cmd_vel_pub_->publish(cmd);
        return true;
    }
    if (path_generated_) {
        return false;
    }
    if ((!pre_path_spin_enable_ || pre_path_spin_target_yaws_.empty()) && pre_scan_pause_after_done_) {
        if (!pre_scan_spin_done_) {
            pre_scan_spin_done_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "Pre-scan disabled/empty. Going to record_pass first point before path generation.");
        }
        cmd_vel_pub_->publish(cmd);
        return false;
    }
    if (!pre_path_spin_enable_ || pre_path_spin_target_yaws_.empty()) {
        return false;
    }
    if (!pre_scan_started_) {
        pre_scan_started_ = true;
        pre_scan_start_time_ = now;
    }

    const double pre_scan_elapsed = (now - pre_scan_start_time_).seconds();
    if (pre_scan_fallback_enable_ &&
        !pre_scan_spin_done_ &&
        pre_scan_elapsed >= pre_scan_timeout_s_) {
        pre_scan_timeout_fallback_ = true;
        pre_path_spin_phase_ = PrePathSpinPhase::Done;
        pre_scan_spin_done_ = true;
        RCLCPP_WARN(this->get_logger(),
                    "Pre-scan timeout after %.1f s. Waiting for OCR/eightboxes before arm lift.",
                    pre_scan_timeout_s_);
        cmd_vel_pub_->publish(cmd);
        return false;
    }

    if (!got_tf_ || !enable_control_) {
        cmd_vel_pub_->publish(cmd);
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Waiting for TF/control enable during pre-scan...");
        return true;
    }

    // Done：继续发零速，但返回 false，让 control_loop 执行路径超时与 fallback，否则会在这里永远提前 return
    if (pre_path_spin_phase_ == PrePathSpinPhase::Done) {
        cmd_vel_pub_->publish(cmd);
        return false;
    }

    if (pre_path_spin_phase_ == PrePathSpinPhase::Idle) {
        pre_path_spin_phase_ = PrePathSpinPhase::SeekYaw;
        pre_path_spin_yaw_idx_ = 0;
    }

    switch (pre_path_spin_phase_) {
        case PrePathSpinPhase::SeekYaw: {
            const double target_yaw = pre_path_spin_target_yaws_[pre_path_spin_yaw_idx_];
            const double err_yaw = normalize_angle(target_yaw - yaw_);
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 500,
                "pre_path_spin: at [%zu/%zu] going to yaw=%.3f rad (current yaw=%.3f err=%.3f)",
                pre_path_spin_yaw_idx_ + 1,
                pre_path_spin_target_yaws_.size(),
                target_yaw, yaw_, err_yaw);
            cmd.angular.z = std::max(-pre_path_spin_max_wz_,
                std::min(pre_path_spin_kp_yaw_ * err_yaw, pre_path_spin_max_wz_));
            if (std::abs(err_yaw) < pre_path_spin_yaw_tol_) {
                cmd.angular.z = 0.0;
                cmd_vel_pub_->publish(cmd);
                if (pre_path_spin_yaw_idx_ + 1 >= pre_path_spin_target_yaws_.size()) {
                    pre_path_spin_phase_ = PrePathSpinPhase::Done;
                    pre_scan_spin_done_ = true;
                    RCLCPP_INFO(this->get_logger(),
                                "Pre-scan spin done. Going to record_pass first point before path generation.");
                } else {
                    pre_path_spin_phase_ = PrePathSpinPhase::PauseBetween;
                    pre_path_spin_t0_ = now;
                }
                return true;
            }
            cmd_vel_pub_->publish(cmd);
            return true;
        }
        case PrePathSpinPhase::PauseBetween:
            if ((now - pre_path_spin_t0_).seconds() >= pre_path_spin_pause_s_) {
                pre_path_spin_yaw_idx_++;
                pre_path_spin_phase_ = PrePathSpinPhase::SeekYaw;
            }
            cmd_vel_pub_->publish(cmd);
            return true;
        case PrePathSpinPhase::Idle:
        case PrePathSpinPhase::Done:
            break;
    }
    return false;
}

bool LidarNavControl::pre_scan_entry_point_required() const {
    return pre_scan_pause_after_done_ &&
           logistics_entry_trajectory_enable_ &&
           entry_traj_loaded_ &&
           !entry_traj_.empty();
}

bool LidarNavControl::pre_scan_entry_point_ready() const {
    return !pre_scan_entry_point_required() || pre_scan_entry_point_done_;
}

bool LidarNavControl::control_pre_scan_entry_point() {
    if (!pre_scan_entry_point_required()) {
        pre_scan_entry_point_done_ = true;
        return false;
    }
    if (pre_scan_entry_point_done_) {
        return false;
    }

    geometry_msgs::msg::Twist stop_cmd;
    if (!enable_control_) {
        cmd_vel_pub_->publish(stop_cmd);
        return true;
    }

    const auto now = this->get_clock()->now();
    if (!pre_scan_hold_after_done_done_) {
        if (!pre_scan_hold_after_done_started_) {
            pre_scan_hold_after_done_started_ = true;
            pre_scan_hold_after_done_t0_ = now;
            RCLCPP_INFO(this->get_logger(),
                        "PRE_SCAN_ENTRY_POINT: pre-scan done, holding %.1fs before moving to record_pass first point.",
                        pre_scan_hold_after_done_s_);
        }

        const double hold_elapsed = (now - pre_scan_hold_after_done_t0_).seconds();
        if (hold_elapsed < pre_scan_hold_after_done_s_) {
            cmd_vel_pub_->publish(stop_cmd);
            if (print_count_ % static_cast<int>(std::max(control_rate_, 1.0)) == 0) {
                RCLCPP_INFO(this->get_logger(),
                            "PRE_SCAN_ENTRY_POINT: holding after pre-scan %.1f / %.1fs",
                            hold_elapsed, pre_scan_hold_after_done_s_);
            }
            return true;
        }

        pre_scan_hold_after_done_done_ = true;
        nav_pd_initialized_ = false;
        logistics_prev_ctrl_time_valid_ = false;
        reset_logistics_cmd_slew();
        if (logistics_controller_) {
            logistics_controller_->reset();
        }
        RCLCPP_INFO(this->get_logger(),
                    "PRE_SCAN_ENTRY_POINT: hold done, moving to record_pass first point.");
    }

    if (!got_tf_) {
        cmd_vel_pub_->publish(stop_cmd);
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "PRE_SCAN_ENTRY_POINT: waiting for TF...");
        return true;
    }

    const auto& target = entry_traj_.front();
    const double target_x = target.x;
    const double target_y = target.y-0.1;
    const double target_yaw = target.yaw;

    std_msgs::msg::Float32MultiArray target_msg;
    target_msg.data.push_back(static_cast<float>(target_x));
    target_msg.data.push_back(static_cast<float>(target_y));
    target_msg.data.push_back(static_cast<float>(target_yaw));
    target_pub_->publish(target_msg);

    const double err_x_map = target_x - x_;
    const double err_y_map = target_y - y_;
    const double err_yaw = normalize_angle(target_yaw - yaw_);
    double err_x_body = 0.0;
    double err_y_body = 0.0;
    map_error_to_body(err_x_map, err_y_map, err_x_body, err_y_body);
    const double dist = std::hypot(err_x_body, err_y_body);
    const double yaw_err_abs = std::abs(err_yaw);

    if (!pre_scan_entry_point_arrived_) {
        const bool position_ready = dist <= entry_traj_align_position_tolerance_;
        const bool yaw_ready = yaw_err_abs <= entry_traj_align_yaw_tolerance_;

        if (position_ready && yaw_ready) {
            pre_scan_entry_point_arrived_ = true;
            pre_scan_entry_wait_started_ = true;
            pre_scan_entry_wait_t0_ = now;
            nav_pd_initialized_ = false;
            const auto ctrl_now = this->get_clock()->now();
            double ctrl_dt = 1.0 / std::max(control_rate_, 1.0);
            if (logistics_prev_ctrl_time_valid_) {
                ctrl_dt = (ctrl_now - logistics_prev_ctrl_time_).seconds();
            }
            logistics_prev_ctrl_time_ = ctrl_now;
            logistics_prev_ctrl_time_valid_ = true;
            cmd_vel_pub_->publish(limit_logistics_cmd_slew(stop_cmd, ctrl_dt));
            RCLCPP_INFO(this->get_logger(),
                        "PRE_SCAN_ENTRY_POINT: reached record_pass first point "
                        "(%.3f, %.3f, %.3f). Waiting 1.0s for eightboxes.",
                        target_x, target_y, target_yaw);
            return true;
        }

        const auto ctrl_now = this->get_clock()->now();
        double ctrl_dt = 1.0 / std::max(control_rate_, 1.0);
        if (logistics_prev_ctrl_time_valid_) {
            ctrl_dt = (ctrl_now - logistics_prev_ctrl_time_).seconds();
        }
        logistics_prev_ctrl_time_ = ctrl_now;
        logistics_prev_ctrl_time_valid_ = true;

        NavControllerInput controller_input;
        controller_input.err_x_body = err_x_body;
        controller_input.err_y_body = err_y_body;
        controller_input.err_yaw = err_yaw;
        controller_input.dist = dist;
        controller_input.robot_y = y_;
        controller_input.dt = ctrl_dt;
        controller_input.track_id = 901;
        controller_input.waypoint_type = WaypointType::PICKUP;
        controller_input.target_y = target_y;
        controller_input.target_yaw = target_yaw;
        controller_input.precise_lateral_zone = lidar_nav::is_in_precise_lateral_arrival_y_zone(
            target_y,
            precise_lateral_arrival_y_zone1_min_,
            precise_lateral_arrival_y_zone1_max_);
        controller_input.rear_exit_creep_zone = lidar_nav::is_in_rear_exit_creep_y_zone(
            target_y, rear_exit_creep_y_min_, rear_exit_creep_y_max_);
        controller_input.enabled = enable_control_;

        NavControllerConfig entry_controller_config = logistics_controller_config_;
        entry_controller_config.approach_to_align_dist = 1.0;
        entry_controller_config.dock_start_dist = 0.15;
        HeadingDockNavController entry_controller(entry_controller_config);
        const NavControllerOutput controller_output = entry_controller.compute(controller_input);
        const geometry_msgs::msg::Twist limited_cmd =
            limit_logistics_cmd_slew(controller_output.cmd, ctrl_dt);
        cmd_vel_pub_->publish(limited_cmd);
        if (print_count_ % 25 == 0) {
            RCLCPP_INFO(this->get_logger(),
                        "PRE_SCAN_ENTRY_POINT: moving to first point pos(%.2f,%.2f,%.2f) "
                        "target(%.2f,%.2f,%.2f) stage=%s dist=%.3f yaw_err=%.3f "
                        "raw_cmd(%.2f,%.2f,%.2f) cmd(%.2f,%.2f,%.2f)",
                        x_, y_, yaw_, target_x, target_y, target_yaw,
                        nav_control_stage_to_string(controller_output.stage),
                        dist, err_yaw,
                        controller_output.cmd.linear.x,
                        controller_output.cmd.linear.y,
                        controller_output.cmd.angular.z,
                        limited_cmd.linear.x,
                        limited_cmd.linear.y,
                        limited_cmd.angular.z);
        }
        return true;
    }

    cmd_vel_pub_->publish(stop_cmd);
    if (!pre_scan_entry_wait_started_) {
        pre_scan_entry_wait_started_ = true;
        pre_scan_entry_wait_t0_ = now;
    }

    const double wait_elapsed = (now - pre_scan_entry_wait_t0_).seconds();
    if (wait_elapsed < 1.0) {
        if (print_count_ % static_cast<int>(std::max(control_rate_, 1.0)) == 0) {
            RCLCPP_INFO(this->get_logger(),
                        "PRE_SCAN_ENTRY_POINT: waiting eightboxes %.1f / 1.0s",
                        wait_elapsed);
        }
        return true;
    }

    if (pending_priorities_valid_ && pending_raw_classes_.size() == 8) {
        if (!ocr_result_locked_) {
            if (!ocr_wait_armed_) {
                arm_ocr_wait_after_eightboxes();
            }
            const double ocr_wait_elapsed =
                std::max(0.0, (now - ocr_wait_t0_).seconds());
            if (ocr_wait_elapsed < ocr_wait_after_eightboxes_sec_) {
                if (print_count_ % static_cast<int>(std::max(control_rate_, 1.0)) == 0) {
                    RCLCPP_INFO(this->get_logger(),
                                "PRE_SCAN_ENTRY_POINT: eightboxes ready; waiting OCR %.1f / %.1fs",
                                ocr_wait_elapsed, ocr_wait_after_eightboxes_sec_);
                }
                return true;
            }
            RCLCPP_WARN(this->get_logger(),
                        "PRE_SCAN_ENTRY_POINT: no OCR within %.1fs after eightboxes; "
                        "generating path from eightboxes.",
                        ocr_wait_after_eightboxes_sec_);
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "PRE_SCAN_ENTRY_POINT: OCR locked; generating path from eightboxes.");
        }
        pre_scan_entry_point_done_ = true;
        entry_traj_done_ = true;
        nav_pd_initialized_ = false;
        reset_logistics_cmd_slew();
        try_generate_path_with_pending_priorities(!ocr_result_locked_);
    } else {
        pre_scan_entry_point_done_ = true;
        entry_traj_done_ = true;
        nav_pd_initialized_ = false;
        reset_logistics_cmd_slew();
        RCLCPP_WARN(this->get_logger(),
                    "PRE_SCAN_ENTRY_POINT: no eightboxes within 1s; using default priorities.");
        generate_path(default_priorities_);
    }
    if (path_generated_) {
        try_complete_pre_scan_handshake();
    }
    return true;
}

void LidarNavControl::reset_arrival_debounce() {
    arrival_candidate_active_ = false;
}

void LidarNavControl::reset_graph_yaw_align() {
    graph_yaw_align_active_ = false;
    graph_yaw_align_target_yaw_ = 0.0;
    graph_yaw_align_wp_idx_ = 0;
}

bool LidarNavControl::update_arrival_debounce(
    double err_x_body,
    double err_y_body,
    double dist,
    double err_yaw,
    size_t waypoint_index,
    WaypointType waypoint_type,
    bool corridor_lateral_only_arrival,
    bool manip_approach_arrival,
    bool direct_nearest_approach,
    bool manip_to_nearest_arrival,
    double tol_x,
    double tol_y,
    double yaw_tolerance,
    double dist_tol)
{
    (void)err_x_body;
    const bool require_yaw =
        waypoint_type == WaypointType::PICKUP || waypoint_type == WaypointType::DROPOFF ||
        manip_to_nearest_arrival;
    const double pos_tol = manip_to_nearest_arrival ? dist_tol : tol_x;
    const bool within_arrival = require_yaw
        ? (dist < pos_tol && std::abs(err_yaw) < yaw_tolerance)
        : corridor_lateral_only_arrival
            ? (std::abs(err_y_body) < tol_y)
            : (dist < dist_tol);

    if (!within_arrival) {
        reset_arrival_debounce();
        return false;
    }

    const auto now = this->get_clock()->now();
    if (!arrival_candidate_active_ || arrival_candidate_wp_idx_ != waypoint_index) {
        arrival_candidate_active_ = true;
        arrival_candidate_wp_idx_ = waypoint_index;
        arrival_candidate_start_time_ = now;
    }

    const double stable_required =
        get_arrival_stable_time(waypoint_type, corridor_lateral_only_arrival);
    const double stable_s = (now - arrival_candidate_start_time_).seconds();
    if (print_count_ % 25 == 0) {
        if (require_yaw) {
            RCLCPP_INFO(this->get_logger(),
                        "Arrival candidate WP[%zu]: dist=%.3f/%.3f yaw=%.3f/%.3f stable=%.2f/%.2fs",
                        waypoint_index, dist, tol_x,
                        std::abs(err_yaw), yaw_tolerance,
                        stable_s, stable_required);
        } else if (corridor_lateral_only_arrival) {
            RCLCPP_INFO(this->get_logger(),
                        "Arrival candidate WP[%zu]: err_y=%.3f/%.3f yaw=ignored (x ignored) stable=%.2f/%.2fs",
                        waypoint_index,
                        std::abs(err_y_body), tol_y,
                        stable_s, stable_required);
        } else if (manip_approach_arrival) {
            RCLCPP_INFO(this->get_logger(),
                        "Arrival candidate WP[%zu]: manip_approach dist=%.3f/%.3f yaw=ignored stable=%.2f/%.2fs",
                        waypoint_index, dist, dist_tol,
                        stable_s, stable_required);
        } else if (direct_nearest_approach) {
            RCLCPP_INFO(this->get_logger(),
                        "Arrival candidate WP[%zu]: direct_nearest dist=%.3f/%.3f yaw=ignored stable=%.2f/%.2fs",
                        waypoint_index, dist, dist_tol,
                        stable_s, stable_required);
        } else if (manip_to_nearest_arrival) {
            RCLCPP_INFO(this->get_logger(),
                        "Arrival candidate WP[%zu]: manip_to_nearest dist=%.3f/%.3f yaw=%.3f/%.3f stable=%.2f/%.2fs",
                        waypoint_index, dist, dist_tol,
                        std::abs(err_yaw), yaw_tolerance,
                        stable_s, stable_required);
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "Arrival candidate WP[%zu]: dist=%.3f/%.3f yaw=ignored stable=%.2f/%.2fs",
                        waypoint_index, dist, dist_tol,
                        stable_s, stable_required);
        }
    }

    return stable_s >= stable_required;
}

double LidarNavControl::get_arrival_stable_time(
    WaypointType type, bool corridor_lateral_only_arrival) const
{
    if (type == WaypointType::PICKUP || type == WaypointType::DROPOFF) {
        return manip_arrival_stable_time_s_;
    }
    if (type == WaypointType::TRANSITION) {
        return corridor_lateral_only_arrival
            ? cross_band_transition_arrival_stable_time_s_
            : transition_arrival_stable_time_s_;
    }
    return arrival_stable_time_s_;
}

void LidarNavControl::get_arrival_tolerances(
    WaypointType type,
    bool corridor_lateral_only_arrival,
    double target_y,
    double& tol_x,
    double& tol_y,
    double& yaw_tolerance) const
{
    if (type == WaypointType::PICKUP || type == WaypointType::DROPOFF) {
        tol_x = manip_position_tolerance_;
        tol_y = manip_position_tolerance_;
        yaw_tolerance = manip_yaw_tolerance_;
        return;
    }

    tol_x = transition_position_tolerance_x_;
    tol_y = corridor_lateral_only_arrival
        ? cross_band_transition_position_tolerance_y_
        : transition_position_tolerance_y_;
    yaw_tolerance = transition_yaw_tolerance_;

    if (corridor_lateral_only_arrival &&
        lidar_nav::is_in_precise_lateral_arrival_y_zone(
            target_y,
            precise_lateral_arrival_y_zone1_min_,
            precise_lateral_arrival_y_zone1_max_)) {
        tol_y = precise_lateral_arrival_tolerance_y_;
    }
}

void LidarNavControl::control_loop() {
    // 发布当前坐标 [x, y, yaw]
    if (got_tf_) {
        std_msgs::msg::Float32MultiArray pose_msg;
        pose_msg.data.push_back(x_);
        pose_msg.data.push_back(y_);
        pose_msg.data.push_back(yaw_);
        pose_pub_->publish(pose_msg);
    }

    if (!enable_control_) {
        reset_arrival_debounce();
        reset_graph_yaw_align();
        logistics_prev_ctrl_time_valid_ = false;
        reset_logistics_cmd_slew();
        if (logistics_controller_) {
            logistics_controller_->reset();
        }
        return; // 未使能时，不计算也不发布 cmd_vel，交出控制权
    }
    if (pre_scan_waiting_resume_) {
        geometry_msgs::msg::Twist cmd;
        reset_logistics_cmd_slew();
        cmd_vel_pub_->publish(cmd);
        return;
    }

    // ============== 直线模式 =================
    if (current_mode_ == NavMode::STRAIGHT_LINE) {
        if (!got_tf_) {
            if (tf_count_ == 0 && print_count_ % static_cast<int>(control_rate_ * 2) == 0) {
                RCLCPP_WARN(this->get_logger(), "STRAIGHT_LINE: No TF received yet...");
            }
            print_count_++;
            return;
        }

        if (!straight_started_) {
            straight_start_x_ = x_;
            straight_start_y_ = y_;
            straight_started_ = true;
            straight_done_ = false;
            RCLCPP_INFO(this->get_logger(), "Straight line mode started at (%.2f, %.2f), target_yaw: %.2f, dist: %.2f", 
                x_, y_, straight_target_yaw_, straight_target_dist_);
        }

        if (straight_done_) {
            geometry_msgs::msg::Twist cmd;
            cmd_vel_pub_->publish(cmd);
            std_msgs::msg::Bool status_msg;
            status_msg.data = true;
            nav_status_pub_->publish(status_msg);
            if (print_count_ % static_cast<int>(control_rate_ * 2) == 0) {
                RCLCPP_INFO(this->get_logger(), "Straight line task completed.");
            }
            print_count_++;
            return;
        }

        double dx_map = x_ - straight_start_x_;
        double dy_map = y_ - straight_start_y_;
        
        // 参考原任务中经过实验验证的 Map -> Lidar -> Body 坐标变换规律
        // 将走过的绝对位移 (dx_map, dy_map) 投影到以 straight_target_yaw_ 为方向的“理想目标坐标系”下
        const double cos_ideal = std::cos(straight_target_yaw_);
        const double sin_ideal = std::sin(straight_target_yaw_);
        
        // 1. 投影到“理想雷达系” (Lidar Frame)
        double dx_ideal_lidar = cos_ideal * dx_map + sin_ideal * dy_map;
        double dy_ideal_lidar = -sin_ideal * dx_map + cos_ideal * dy_map;
        
        // 2. 转换为“理想机体系” (Body Frame)
        // 根据验证规律：Body.x (前) = Lidar.y, Body.y (左) = -Lidar.x
        double traveled_dist = dy_ideal_lidar;       // 机体系x轴进度 (有效前进距离)
        double cross_track_error = -dx_ideal_lidar;  // 机体系y轴偏移 (横向偏离量，>0说明在直线左侧)

        double remain_dist = straight_target_dist_ - traveled_dist;
        
        double err_yaw = normalize_angle(straight_target_yaw_ - yaw_);

        if (remain_dist < 0.05 && std::abs(err_yaw) < 0.1) {
            straight_done_ = true;
            geometry_msgs::msg::Twist cmd;
            cmd_vel_pub_->publish(cmd);
            std_msgs::msg::Bool status_msg;
            status_msg.data = true;
            nav_status_pub_->publish(status_msg);
            RCLCPP_INFO(this->get_logger(), "Reached straight line target!");
            return;
        }

        geometry_msgs::msg::Twist cmd;
        if (std::abs(err_yaw) > 0.2) {
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.angular.z = std::max(-max_dyaw_, std::min(kp_yaw_ * err_yaw, max_dyaw_));
        } else {
            cmd.linear.x = std::max(-straight_max_speed_, std::min(kp_x_ * remain_dist, straight_max_speed_));
            // 消除横向误差：如果在左侧（cross_track_error > 0），则需要向右走（即负的 y 速度），因此乘 -1
            cmd.linear.y = std::max(-max_vy_, std::min(straight_kp_y_ * (-cross_track_error), max_vy_));
            cmd.angular.z = std::max(-max_dyaw_, std::min(kp_yaw_ * err_yaw, max_dyaw_));
        }

        cmd_vel_pub_->publish(cmd);

        if (print_count_ % 25 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                "STRAIGHT: rem=%.2f, cross=%.2f, err_yaw=%.2f, vx=%.2f, vy=%.2f, wz=%.2f",
                remain_dist, cross_track_error, err_yaw, cmd.linear.x, cmd.linear.y, cmd.angular.z);
        }
        print_count_++;
        return;
    }

    // ============== AUTO_PARKU 轨迹回放 =================
    if (current_mode_ == NavMode::AUTO_PARKU) {
        control_auto_parku();
        return;
    }

    // ============== 原有任务模式 =================

    try_complete_pre_scan_handshake();

    if (pre_scan_pause_after_done_ &&
        pre_scan_spin_done_ &&
        !pre_scan_entry_point_ready()) {
        if (control_pre_scan_entry_point()) {
            print_count_++;
            no_path_count_++;
            return;
        }
    }

    if (!path_generated_) {
        const auto now = this->get_clock()->now();

        if (handle_pre_path_spin()) {
            print_count_++;
            no_path_count_++;
            return;
        }
        if (pre_scan_pause_after_done_ &&
            pre_path_spin_enable_ &&
            pre_path_spin_phase_ != PrePathSpinPhase::Done) {
            geometry_msgs::msg::Twist cmd;
            cmd_vel_pub_->publish(cmd);
            if (print_count_ % static_cast<int>(control_rate_ * 2) == 0) {
                RCLCPP_WARN(this->get_logger(), "Waiting for TF/pre-scan before path generation...");
            }
            print_count_++;
            no_path_count_++;
            return;
        }

        maybe_generate_path_without_ocr_on_timeout();
        if (!path_generated_ &&
            ocr_result_locked_ &&
            pending_priorities_valid_ &&
            pending_raw_classes_.size() == 8) {
            try_generate_path_with_pending_priorities();
        }

        if (pending_priorities_valid_ && !ocr_result_locked_ && ocr_wait_armed_) {
            if (print_count_ % static_cast<int>(control_rate_ * 2) == 0) {
                const double elapsed = (now - ocr_wait_t0_).seconds();
                RCLCPP_WARN(this->get_logger(),
                            "eightboxes ready; waiting for OCR (%.1f / %.1f s)...",
                            elapsed, ocr_wait_after_eightboxes_sec_);
            }
        }

        if (pre_scan_timeout_fallback_) {
            pre_scan_timeout_fallback_ = false;
            self_path_valid_ = false;
            no_path_ = true;
            no_path_count_ = 0;
            if (pending_priorities_valid_ && pending_raw_classes_.size() == 8) {
                RCLCPP_WARN(this->get_logger(),
                            "Pre-scan fallback: generating path from eightboxes (no OCR).");
                try_generate_path_with_pending_priorities(true);
            } else {
                RCLCPP_WARN(this->get_logger(),
                            "Pre-scan fallback: generating path with default priorities.");
                generate_path(default_priorities_);
            }
        }
        if (path_generated_) {
            try_complete_pre_scan_handshake();
            return;
        }
        if (pending_priorities_valid_ && !ocr_result_locked_ && ocr_wait_armed_) {
            print_count_++;
            no_path_count_++;
            return;
        }
        if (!path_fallback_wait_armed_) {
            path_fallback_wait_t0_ = now;
            path_fallback_wait_armed_ = true;
        }
        if (print_count_ % static_cast<int>(control_rate_ * 2) == 0) {
            RCLCPP_WARN(this->get_logger(), "Waiting for material priorities to generate path...");
        }
        print_count_++;
        no_path_count_++;
        if (!self_path_valid_) {
            const double elapsed = (now - path_fallback_wait_t0_).seconds();
            if (elapsed >= path_fallback_timeout_sec_) {
                self_path_valid_ = true;
                RCLCPP_WARN(this->get_logger(),
                            "No path after %.1f s; enabling fallback priorities.",
                            path_fallback_timeout_sec_);
            }
        }
        if (self_path_valid_) {
            no_path_ = true;
            no_path_count_ = 0;
            RCLCPP_WARN(this->get_logger(),
                        "No eightboxes received; using default priorities fallback.");
            generate_path(default_priorities_);
            try_complete_pre_scan_handshake();
        }
        return;
    }

    if (pre_scan_pause_after_done_ && !pre_scan_event_sent_) {
        if (entry_trajectory_before_pre_scan() && !entry_traj_done_) {
            control_logistics_entry_trajectory();
            print_count_++;
            return;
        }
        try_complete_pre_scan_handshake();
        if (!pre_scan_event_sent_) {
            print_count_++;
            return;
        }
    }

    if (!got_tf_) {
        if (tf_count_ == 0 && print_count_ % static_cast<int>(control_rate_ * 2) == 0) {
            RCLCPP_WARN(this->get_logger(), "No TF received yet...");
        }
        print_count_++;
        return;
    }

    // 序列执行完成
    if (current_wp_idx_ >= waypoints_.size()) {
        geometry_msgs::msg::Twist cmd;
        reset_logistics_cmd_slew();
        cmd_vel_pub_->publish(cmd);
        if (print_count_ % static_cast<int>(control_rate_ * 2) == 0) {
            RCLCPP_INFO(this->get_logger(), "All tasks completed. Holding position.");
        }
        
        // 发布一个特殊的状态，通知上层状态机所有导航任务已经彻底结束
        // (取决于你的上层 task_state_machine 逻辑，如果不需要特殊标志，发 true 也行)
        std_msgs::msg::Bool status_msg;
        status_msg.data = true; 
        nav_status_pub_->publish(status_msg);
        if (!logistics_finished_sent_) {
            publish_logistics_event(LogisticsEventType::LOGISTICS_FINISHED, current_wp_idx_);
            logistics_finished_sent_ = true;
        }

        print_count_++;
        return;
    }

    // 处理等待逻辑
    if (is_waiting_) {
        geometry_msgs::msg::Twist cmd; // 发布0速度
        reset_logistics_cmd_slew();
        cmd_vel_pub_->publish(cmd);
        
        auto now = this->get_clock()->now();
        double elapsed = (now - wait_start_time_).seconds();
        if (elapsed >= wait_time_) {
            RCLCPP_INFO(this->get_logger(), "Wait completed. Resuming navigation to next waypoint.");
            is_waiting_ = false;
        } else {
            if (print_count_ % static_cast<int>(control_rate_) == 0) {
                RCLCPP_INFO(this->get_logger(), "Waiting at zone... %.1f / %.1f seconds", elapsed, wait_time_);
            }
            print_count_++;
            return;
        }
    }

    if (startup_entry_replay_pending_) {
        if (!entry_traj_loaded_ || entry_traj_done_) {
            startup_entry_replay_pending_ = false;
            RCLCPP_WARN(this->get_logger(),
                        "Startup entry replay requested but trajectory is unavailable/done; "
                        "continuing to dropoff.");
        } else {
            control_logistics_entry_trajectory();
            if (entry_traj_done_) {
                startup_entry_replay_pending_ = false;
                logistics_prev_ctrl_time_valid_ = false;
                nav_pd_initialized_ = false;
                reset_logistics_cmd_slew();
                if (logistics_controller_) {
                    logistics_controller_->reset();
                }
                RCLCPP_INFO(this->get_logger(),
                            "Startup entry trajectory finished. Continuing to first dropoff.");
            }
            return;
        }
    }

    if (graph_yaw_align_active_) {
        const double err = normalize_angle(graph_yaw_align_target_yaw_ - yaw_);
        geometry_msgs::msg::Twist cmd;
        if (std::abs(err) < transition_yaw_tolerance_) {
            RCLCPP_INFO(this->get_logger(),
                        "Graph yaw aligned at WP[%zu]: yaw=%.3f (target %.3f), advancing",
                        graph_yaw_align_wp_idx_, yaw_, graph_yaw_align_target_yaw_);
            current_wp_idx_ = graph_yaw_align_wp_idx_ + 1;
            reset_graph_yaw_align();
            reset_arrival_debounce();
            logistics_prev_ctrl_time_valid_ = false;
            reset_logistics_cmd_slew();
            if (logistics_controller_) {
                logistics_controller_->reset();
            }
            cmd_vel_pub_->publish(cmd);
            std_msgs::msg::Bool status_msg;
            status_msg.data = true;
            nav_status_pub_->publish(status_msg);
            print_count_++;
            return;
        }

        const double max_wz = logistics_controller_config_.align_max_wz;
        double wz = std::max(
            -max_wz,
            std::min(logistics_controller_config_.kp_final_yaw * err, max_wz));
        const double yaw_creep_min = logistics_controller_config_.align_yaw_creep_min;
        const double yaw_deadband = logistics_controller_config_.yaw_deadband;
        if (yaw_creep_min > 0.0 && std::abs(err) > yaw_deadband &&
            std::abs(wz) < yaw_creep_min) {
            wz = std::copysign(yaw_creep_min, err);
        }
        cmd.angular.z = wz;
        const auto ctrl_now = this->get_clock()->now();
        double ctrl_dt = 1.0 / std::max(control_rate_, 1.0);
        if (logistics_prev_ctrl_time_valid_) {
            ctrl_dt = (ctrl_now - logistics_prev_ctrl_time_).seconds();
        }
        logistics_prev_ctrl_time_ = ctrl_now;
        logistics_prev_ctrl_time_valid_ = true;
        const geometry_msgs::msg::Twist limited_cmd =
            limit_logistics_cmd_slew(cmd, ctrl_dt);
        cmd_vel_pub_->publish(limited_cmd);
        if (print_count_ % 25 == 0) {
            RCLCPP_INFO(this->get_logger(),
                        "WP[%zu] GRAPH_YAW_ALIGN target=%.3f pos_yaw=%.3f err=%.3f raw_wz=%.2f cmd_wz=%.2f",
                        graph_yaw_align_wp_idx_, graph_yaw_align_target_yaw_, yaw_, err,
                        cmd.angular.z, limited_cmd.angular.z);
        }
        print_count_++;
        return;
    }

    // 当前目标点
    TargetPoint current_wp = waypoints_[current_wp_idx_];
    double target_x = current_wp.x;
    double target_y = current_wp.y;
    double target_yaw = current_wp.yaw;

    // 发布当前目标点坐标 [x, y, yaw]
    std_msgs::msg::Float32MultiArray target_msg;
    target_msg.data.push_back(target_x);
    target_msg.data.push_back(target_y);
    target_msg.data.push_back(target_yaw);
    target_pub_->publish(target_msg);

    double err_x_map = target_x - x_;
    double err_y_map = target_y - y_;
    double err_yaw = normalize_angle(target_yaw - yaw_);

    double err_x_body = 0.0, err_y_body = 0.0;
    map_error_to_body(err_x_map, err_y_map, err_x_body, err_y_body);
    double dist = std::hypot(err_x_body, err_y_body);

    double tol_x = arrival_position_tolerance_;
    double tol_y = arrival_position_tolerance_;
    double yaw_tolerance = arrival_yaw_tolerance_;
    get_arrival_tolerances(
        current_wp.type, current_wp.corridor_lateral_only_arrival, target_y,
        tol_x, tol_y, yaw_tolerance);

    double dist_tol = transition_dist_arrival_tolerance_;
    if (current_wp.manip_approach_arrival || current_wp.direct_nearest_approach) {
        dist_tol = manip_approach_arrival_tolerance_;
    }

    // 判断是否到达当前目标点
    if (update_arrival_debounce(
            err_x_body, err_y_body, dist, err_yaw, current_wp_idx_, current_wp.type,
            current_wp.corridor_lateral_only_arrival,
            current_wp.manip_approach_arrival,
            current_wp.direct_nearest_approach,
            current_wp.manip_to_nearest_arrival,
            tol_x, tol_y, yaw_tolerance, dist_tol)) {
        const char* wp_type_name = "TRANSITION";
        if (current_wp.type == WaypointType::PICKUP) {
            wp_type_name = "PICKUP";
        } else if (current_wp.type == WaypointType::DROPOFF) {
            wp_type_name = "DROPOFF";
        }
        RCLCPP_INFO(this->get_logger(),
                    "Reached waypoint %zu %s (%.3f, %.3f, %.3f yaw)!",
                    current_wp_idx_, wp_type_name, target_x, target_y, target_yaw);

        const double yaw_align_threshold = current_wp.direct_nearest_approach
            ? direct_nearest_yaw_align_threshold_
            : transition_yaw_tolerance_;
        if (current_wp.graph_yaw_align_after_arrival &&
            std::abs(err_yaw) >= yaw_align_threshold) {
            graph_yaw_align_active_ = true;
            graph_yaw_align_target_yaw_ = target_yaw;
            graph_yaw_align_wp_idx_ = current_wp_idx_;
            reset_arrival_debounce();
            logistics_prev_ctrl_time_valid_ = false;
            if (logistics_controller_) {
                logistics_controller_->reset();
            }
            RCLCPP_INFO(this->get_logger(),
                        "WP[%zu] position reached, aligning to graph yaw %.3f before next segment",
                        current_wp_idx_, target_yaw);
            geometry_msgs::msg::Twist cmd;
            reset_logistics_cmd_slew();
            cmd_vel_pub_->publish(cmd);
            print_count_++;
            return;
        }

        current_wp_idx_++;
        reset_arrival_debounce();
        logistics_prev_ctrl_time_valid_ = false;
        if (logistics_controller_) {
            logistics_controller_->reset();
        }
        geometry_msgs::msg::Twist cmd;
         // 停止当前运动
        reset_logistics_cmd_slew();
        cmd_vel_pub_->publish(cmd);
        
        // 触发等待逻辑 (仅在取货或放货点等待)
        if (current_wp.type == WaypointType::PICKUP || current_wp.type == WaypointType::DROPOFF) {
            const bool is_pickup = current_wp.type == WaypointType::PICKUP;
            if (is_pickup &&
                startup_entry_replay_enabled() &&
                startup_first_pickup_wp_valid_ &&
                current_wp_idx_ - 1 == startup_first_pickup_wp_idx_) {
                entry_traj_session_started_ = false;
                entry_traj_clock_started_ = false;
                entry_traj_done_ = !entry_traj_loaded_;
                entry_replay_start_t_ = 0.0;
                startup_entry_replay_pending_ = entry_traj_loaded_;
                RCLCPP_INFO(this->get_logger(),
                            "Startup pickup reached at WP[%zu]. Entry trajectory replay %s.",
                            current_wp_idx_ - 1,
                            startup_entry_replay_pending_ ? "armed" : "skipped");
            }
            publish_logistics_event(
                is_pickup ? LogisticsEventType::PICKUP_REACHED : LogisticsEventType::DROPOFF_REACHED,
                current_wp_idx_ - 1);
            if (external_manipulation_enable_) {
                enable_control_ = false;
                RCLCPP_INFO(this->get_logger(), "Reached %s zone. Waiting for external manipulation manager.",
                    is_pickup ? "PICKUP" : "DROPOFF");
            } else {
                is_waiting_ = true;
                wait_start_time_ = this->get_clock()->now();
                RCLCPP_INFO(this->get_logger(), "Reached %s zone. Starting local wait for %.1f seconds...",
                    is_pickup ? "PICKUP" : "DROPOFF", wait_time_);
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Reached TRANSITION point. Moving to next immediately.");
        }
        
        // 发布到达状态，通知 task_state_machine
        std_msgs::msg::Bool status_msg;
        status_msg.data = true;
        nav_status_pub_->publish(status_msg);
        return;
    }

    const auto ctrl_now = this->get_clock()->now();
    double ctrl_dt = 1.0 / std::max(control_rate_, 1.0);
    if (logistics_prev_ctrl_time_valid_) {
        ctrl_dt = (ctrl_now - logistics_prev_ctrl_time_).seconds();
    }
    logistics_prev_ctrl_time_ = ctrl_now;
    logistics_prev_ctrl_time_valid_ = true;

    NavControllerInput controller_input;
    controller_input.err_x_body = err_x_body;
    controller_input.err_y_body = err_y_body;
    controller_input.err_yaw = err_yaw;
    controller_input.dist = dist;
    controller_input.robot_y = y_;
    controller_input.dt = ctrl_dt;
    controller_input.track_id = current_wp_idx_;
    controller_input.waypoint_type = current_wp.type;
    controller_input.cross_isolation_band = current_wp.cross_isolation_band;
    controller_input.tight_lateral_zone = current_wp.tight_lateral_zone;
    controller_input.corridor_lateral_only_arrival =
        current_wp.corridor_lateral_only_arrival;
    controller_input.manip_approach_arrival = current_wp.manip_approach_arrival;
    controller_input.manip_to_nearest_arrival = current_wp.manip_to_nearest_arrival;
    controller_input.direct_nearest_approach = current_wp.direct_nearest_approach;
    controller_input.post_manip_exit = current_wp.post_manip_exit;
    controller_input.target_y = target_y;
    controller_input.target_yaw = target_yaw;
    controller_input.precise_lateral_zone = lidar_nav::is_in_precise_lateral_arrival_y_zone(
        target_y,
        precise_lateral_arrival_y_zone1_min_,
        precise_lateral_arrival_y_zone1_max_);
    controller_input.rear_exit_creep_zone = lidar_nav::is_in_rear_exit_creep_y_zone(
        target_y, rear_exit_creep_y_min_, rear_exit_creep_y_max_);
    controller_input.enabled = enable_control_;

    const NavControllerOutput controller_output =
        logistics_controller_->compute(controller_input);
    const geometry_msgs::msg::Twist limited_cmd =
        limit_logistics_cmd_slew(controller_output.cmd, ctrl_dt);
    cmd_vel_pub_->publish(limited_cmd);

    if (print_count_ % 25 == 0) {
        const bool rear_corridor_yaw_free =
            current_wp.type == WaypointType::TRANSITION &&
            lidar_nav::is_in_rear_corridor_yaw_free_zone(
                y_, pickup_rear_staging_y_min_, rear_corridor_yaw_free_y_max_);
        const bool dist_only_arrival =
            current_wp.type == WaypointType::TRANSITION &&
            !current_wp.corridor_lateral_only_arrival;
        RCLCPP_INFO(this->get_logger(),
            "WP[%zu] stage=%s cross_band=%s tight_lat=%s manip_appr=%s direct_near=%s post_exit=%s yaw_free=%s dist=%.3f tol=(%.3f,%.3f) dist_tol=%.3f pos(%.2f,%.2f,%.2f) target(%.2f,%.2f,%.2f) err_map(%.2f,%.2f,%.2f) err_body(%.2f,%.2f,%.2f) raw_cmd(%.2f,%.2f,%.2f) cmd(%.2f,%.2f,%.2f)",
            current_wp_idx_,
            nav_control_stage_to_string(controller_output.stage),
            current_wp.cross_isolation_band ? "true" : "false",
            current_wp.tight_lateral_zone ? "true" : "false",
            current_wp.manip_approach_arrival ? "true" : "false",
            current_wp.direct_nearest_approach ? "true" : "false",
            current_wp.post_manip_exit ? "true" : "false",
            rear_corridor_yaw_free ? "true" : "false",
            dist, tol_x, tol_y, dist_only_arrival ? dist_tol : 0.0,
            x_, y_, yaw_, target_x, target_y, target_yaw,
            err_x_map, err_y_map, err_yaw,
            err_x_body, err_y_body, err_yaw,
            controller_output.cmd.linear.x,
            controller_output.cmd.linear.y,
            controller_output.cmd.angular.z,
            limited_cmd.linear.x,
            limited_cmd.linear.y,
            limited_cmd.angular.z);
    }
    print_count_++;
}
