#include "lidar_nav_demo_cpp/lidar_nav_demo_node.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

bool LidarNavControl::load_recorded_trajectory(
    const std::string& path, const char* log_tag,
    std::vector<RecordedPose>& traj) const
{
    traj.clear();

    if (path.empty()) {
        RCLCPP_ERROR(this->get_logger(), "%s: trajectory path is empty", log_tag);
        return false;
    }

    std::ifstream in(path);
    if (!in.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "%s: cannot open trajectory file: %s",
                     log_tag, path.c_str());
        return false;
    }

    std::string line;
    size_t line_no = 0;
    while (std::getline(in, line)) {
        ++line_no;
        size_t start = line.find_first_not_of(" \t\r\n");
        if (start == std::string::npos) {
            continue;
        }
        if (line[start] == '#') {
            continue;
        }

        std::istringstream iss(line.substr(start));
        RecordedPose p{};
        if (!(iss >> p.t >> p.x >> p.y >> p.yaw)) {
            RCLCPP_WARN(this->get_logger(),
                        "%s: skip invalid line %zu in %s", log_tag, line_no, path.c_str());
            continue;
        }
        traj.push_back(p);
    }

    if (traj.size() < 2) {
        RCLCPP_ERROR(this->get_logger(),
                     "%s: need at least 2 valid points, got %zu in %s",
                     log_tag, traj.size(), path.c_str());
        traj.clear();
        return false;
    }

    for (size_t i = 1; i < traj.size(); ++i) {
        if (traj[i].t <= traj[i - 1].t) {
            RCLCPP_ERROR(this->get_logger(),
                         "%s: time must be strictly increasing at index %zu", log_tag, i);
            traj.clear();
            return false;
        }
    }

    const double t0 = traj.front().t;
    for (auto& p : traj) {
        p.t -= t0;
    }

    RCLCPP_INFO(this->get_logger(),
                "%s: loaded %zu points, duration %.3f s from %s",
                log_tag, traj.size(), traj.back().t, path.c_str());
    return true;
}

bool LidarNavControl::sample_recorded_trajectory(
    const std::vector<RecordedPose>& traj,
    double elapsed, double& x, double& y, double& yaw) const
{
    if (traj.empty()) {
        return false;
    }

    if (elapsed <= traj.front().t) {
        x = traj.front().x;
        y = traj.front().y;
        yaw = traj.front().yaw;
        return true;
    }

    const double t_end = traj.back().t;
    if (elapsed >= t_end) {
        x = traj.back().x;
        y = traj.back().y;
        yaw = traj.back().yaw;
        return true;
    }

    for (size_t i = 0; i + 1 < traj.size(); ++i) {
        const auto& a = traj[i];
        const auto& b = traj[i + 1];
        if (elapsed >= a.t && elapsed <= b.t) {
            const double dt_seg = b.t - a.t;
            const double alpha = (dt_seg > 1e-9) ? (elapsed - a.t) / dt_seg : 0.0;
            x = a.x + alpha * (b.x - a.x);
            y = a.y + alpha * (b.y - a.y);
            const double dyaw = normalize_angle(b.yaw - a.yaw);
            yaw = normalize_angle(a.yaw + alpha * dyaw);
            return true;
        }
    }

    x = traj.back().x;
    y = traj.back().y;
    yaw = traj.back().yaw;
    return true;
}

bool LidarNavControl::load_auto_parku_trajectory(const std::string& path) {
    auto_parku_loaded_ = load_recorded_trajectory(path, "AUTO_PARKU", auto_parku_traj_);
    if (!auto_parku_loaded_) {
        auto_parku_traj_.clear();
    }
    return auto_parku_loaded_;
}

bool LidarNavControl::sample_auto_parku_reference(
    double elapsed, double& x, double& y, double& yaw) const
{
    if (!auto_parku_loaded_) {
        return false;
    }
    return sample_recorded_trajectory(auto_parku_traj_, elapsed, x, y, yaw);
}

bool LidarNavControl::entry_trajectory_ready() const {
    return !entry_trajectory_before_pre_scan() || entry_traj_done_;
}

bool LidarNavControl::entry_trajectory_before_pre_scan() const {
    return logistics_entry_trajectory_enable_ && !startup_entry_replay_enabled();
}

bool LidarNavControl::startup_entry_replay_enabled() const {
    return startup_pickup_enable_ &&
           startup_entry_replay_after_pickup_ &&
           logistics_entry_trajectory_enable_;
}

void LidarNavControl::map_error_to_body(
    double err_x_map, double err_y_map,
    double& err_x_body, double& err_y_body) const
{
    const double cos_yaw = std::cos(yaw_);
    const double sin_yaw = std::sin(yaw_);
    const double err_x_lidar = cos_yaw * err_y_map - sin_yaw * err_x_map;
    const double err_y_lidar = -sin_yaw * err_y_map - cos_yaw * err_x_map;

    err_x_body = err_x_lidar + lidar_offset_x_;
    err_y_body = err_y_lidar + lidar_offset_y_;
}

void LidarNavControl::compute_body_pd_cmd(
    double err_x_body, double err_y_body, double err_yaw,
    geometry_msgs::msg::Twist& cmd, size_t pd_track_id)
{
    cmd = geometry_msgs::msg::Twist();

    if (!enable_control_) {
        return;
    }

    const auto ctrl_now = this->get_clock()->now();
    if (!nav_pd_initialized_ || nav_pd_wp_idx_ != pd_track_id) {
        nav_pd_wp_idx_ = pd_track_id;
        nav_prev_err_x_body_ = err_x_body;
        nav_prev_err_y_body_ = err_y_body;
        nav_prev_err_yaw_ = err_yaw;
        nav_prev_ctrl_time_ = ctrl_now;
        nav_pd_initialized_ = true;
    }

    double dt = (ctrl_now - nav_prev_ctrl_time_).seconds();
    dt = std::max(1e-4, std::min(dt, 0.25));

    const double dex_dt = (err_x_body - nav_prev_err_x_body_) / dt;
    const double dey_dt = (err_y_body - nav_prev_err_y_body_) / dt;
    const double deyaw_dt = normalize_angle(err_yaw - nav_prev_err_yaw_) / dt;

    nav_prev_err_x_body_ = err_x_body;
    nav_prev_err_y_body_ = err_y_body;
    nav_prev_err_yaw_ = err_yaw;
    nav_prev_ctrl_time_ = ctrl_now;

    if (std::abs(err_yaw) > yaw_first_threshold_) {
        const double wz = kp_yaw_ * err_yaw + kd_yaw_ * deyaw_dt;
        cmd.angular.z = std::max(-max_dyaw_, std::min(wz, max_dyaw_));
        return;
    }

    double vx = kp_x_ * err_x_body + kd_x_ * dex_dt;
    double vy = kp_y_ * err_y_body + kd_y_ * dey_dt;
    if (lin_vel_creep_min_ > 0.0) {
        constexpr double eps = 5e-3;
        if (std::abs(err_x_body) > eps && std::abs(vx) < lin_vel_creep_min_) {
            vx = std::copysign(lin_vel_creep_min_, err_x_body);
        }
        if (std::abs(err_y_body) > eps && std::abs(vy) < lin_vel_creep_min_) {
            vy = std::copysign(lin_vel_creep_min_, err_y_body);
        }
    }
    cmd.linear.x = std::max(-max_vx_, std::min(vx, max_vx_));
    cmd.linear.y = std::max(-max_vy_, std::min(vy, max_vy_));
    const double wz = kp_yaw_ * err_yaw + kd_yaw_ * deyaw_dt;
    cmd.angular.z = std::max(-max_dyaw_, std::min(wz, max_dyaw_));
}

bool LidarNavControl::control_recorded_trajectory_replay(
    const char* log_tag,
    const std::vector<RecordedPose>& traj,
    bool loaded,
    bool& session_started,
    bool& clock_started,
    bool& done,
    rclcpp::Time& replay_t0,
    size_t pd_track_id,
    bool publish_nav_status_on_done,
    const TrajectoryReplayOptions& options)
{
    if (!loaded || traj.size() < 2) {
        if (print_count_ % static_cast<int>(control_rate_ * 2) == 0) {
            RCLCPP_WARN(this->get_logger(), "%s: trajectory not loaded", log_tag);
        }
        return false;
    }

    if (!enable_control_) {
        return false;
    }

    if (!got_tf_) {
        if (tf_count_ == 0 && print_count_ % static_cast<int>(control_rate_ * 2) == 0) {
            RCLCPP_WARN(this->get_logger(), "%s: No TF received yet...", log_tag);
        }
        return false;
    }

    if (x_ > 5.0 || x_ < -5.0 || y_ > 10.0 || y_ < -2.0) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "%s: pose(%.2f, %.2f) out of bounds, holding zero cmd",
            log_tag, x_, y_);
        geometry_msgs::msg::Twist cmd;
        cmd_vel_pub_->publish(cmd);
        return true;
    }

    const double replay_start_t = std::max(0.0, options.replay_start_t);

    if (!session_started) {
        session_started = true;
        nav_pd_initialized_ = false;
        double ref0_x = traj.front().x;
        double ref0_y = traj.front().y;
        double ref0_yaw = traj.front().yaw;
        if (replay_start_t > 0.0) {
            sample_recorded_trajectory(traj, replay_start_t, ref0_x, ref0_y, ref0_yaw);
        }
        RCLCPP_INFO(this->get_logger(),
                    "%s: session started, duration %.3f s, %zu points, "
                    "start_t=%.3f ref(%.3f, %.3f, %.3f), pose(%.3f, %.3f, %.3f)",
                    log_tag, traj.back().t, traj.size(),
                    replay_start_t, ref0_x, ref0_y, ref0_yaw, x_, y_, yaw_);
    }

    if (done) {
        geometry_msgs::msg::Twist cmd;
        cmd_vel_pub_->publish(cmd);
        if (publish_nav_status_on_done) {
            std_msgs::msg::Bool status_msg;
            status_msg.data = true;
            nav_status_pub_->publish(status_msg);
        }
        return true;
    }

    double ref_x = 0.0;
    double ref_y = 0.0;
    double ref_yaw = 0.0;
    double elapsed = 0.0;
    const double t_end = traj.back().t;

    if (options.align_to_start_before_play && !clock_started) {
        if (replay_start_t > 0.0) {
            sample_recorded_trajectory(traj, replay_start_t, ref_x, ref_y, ref_yaw);
        } else {
            ref_x = traj.front().x;
            ref_y = traj.front().y;
            ref_yaw = traj.front().yaw;
        }

        const double align_dist = std::hypot(ref_x - x_, ref_y - y_);
        const double align_yaw_err = std::abs(normalize_angle(ref_yaw - yaw_));
        if (align_dist <= options.align_position_tolerance &&
            align_yaw_err <= options.align_yaw_tolerance) {
            replay_t0 = this->get_clock()->now();
            clock_started = true;
            nav_pd_initialized_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "%s: aligned to start (dist=%.3f yaw=%.3f), replay clock started",
                        log_tag, align_dist, align_yaw_err);
        }
    } else {
        if (!clock_started) {
            replay_t0 = this->get_clock()->now();
            clock_started = true;
            nav_pd_initialized_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "%s: replay clock started, duration %.3f s, %zu points",
                        log_tag, t_end, traj.size());
        }

        elapsed = (this->get_clock()->now() - replay_t0).seconds();
        const double traj_t = replay_start_t + elapsed;
        if (!sample_recorded_trajectory(traj, traj_t, ref_x, ref_y, ref_yaw)) {
            return true;
        }

        if (options.pause_tracking_error > 0.0) {
            const double track_err = std::hypot(ref_x - x_, ref_y - y_);
            if (track_err > options.pause_tracking_error) {
                const double dt_pause = 1.0 / std::max(control_rate_, 1.0);
                replay_t0 = replay_t0 + rclcpp::Duration::from_seconds(dt_pause);
                elapsed = (this->get_clock()->now() - replay_t0).seconds();
                sample_recorded_trajectory(traj, replay_start_t + elapsed, ref_x, ref_y, ref_yaw);
            }
        }
    }

    std_msgs::msg::Float32MultiArray target_msg;
    target_msg.data.push_back(static_cast<float>(ref_x));
    target_msg.data.push_back(static_cast<float>(ref_y));
    target_msg.data.push_back(static_cast<float>(ref_yaw));
    target_pub_->publish(target_msg);

    const double err_x_map = ref_x - x_;
    const double err_y_map = ref_y - y_;
    const double err_yaw = normalize_angle(ref_yaw - yaw_);

    double err_x_body = 0.0;
    double err_y_body = 0.0;
    map_error_to_body(err_x_map, err_y_map, err_x_body, err_y_body);

    geometry_msgs::msg::Twist cmd;
    compute_body_pd_cmd(err_x_body, err_y_body, err_yaw, cmd, pd_track_id);
    cmd_vel_pub_->publish(cmd);

    const double traj_t_done = replay_start_t + elapsed;
    if (clock_started && traj_t_done >= t_end) {
        done = true;
        RCLCPP_INFO(this->get_logger(),
                    "%s: reached end of trajectory (t=%.3f s), pose(%.3f, %.3f, %.3f yaw)",
                    log_tag, t_end, x_, y_, yaw_);
    }

    if (print_count_ % 25 == 0) {
        RCLCPP_INFO(this->get_logger(),
                    "%s: clock=%s t=%.2f/%.2f pos(%.2f,%.2f,%.2f) ref(%.2f,%.2f,%.2f) "
                    "err_body(%.2f,%.2f,%.2f) cmd(%.2f,%.2f,%.2f)",
                    log_tag, clock_started ? "run" : "align",
                    traj_t_done, t_end, x_, y_, yaw_, ref_x, ref_y, ref_yaw,
                    err_x_body, err_y_body, err_yaw,
                    cmd.linear.x, cmd.linear.y, cmd.angular.z);
    }
    return true;
}

void LidarNavControl::control_logistics_entry_trajectory() {
    TrajectoryReplayOptions options;
    options.align_to_start_before_play = true;
    options.align_position_tolerance = entry_traj_align_position_tolerance_;
    options.align_yaw_tolerance = entry_traj_align_yaw_tolerance_;
    options.pause_tracking_error = entry_traj_pause_tracking_error_;

    if (startup_entry_replay_pending_) {
        if (!entry_traj_session_started_) {
            const double y_thresh = y_ + 0.3;
            entry_replay_start_t_ = 0.0;
            for (const auto& p : entry_traj_) {
                if (p.y > y_thresh) {
                    entry_replay_start_t_ = p.t;
                    break;
                }
            }
            RCLCPP_INFO(this->get_logger(),
                        "Startup entry replay: robot_y=%.3f, start from first y>%.3f at t=%.3f",
                        y_, y_thresh, entry_replay_start_t_);
        }
        options.replay_start_t = entry_replay_start_t_;
    }

    control_recorded_trajectory_replay(
        "LOGISTICS_ENTRY", entry_traj_, entry_traj_loaded_,
        entry_traj_session_started_, entry_traj_clock_started_, entry_traj_done_,
        entry_replay_t0_, 0, false, options);
    print_count_++;
}

void LidarNavControl::control_auto_parku() {
    TrajectoryReplayOptions options;
    control_recorded_trajectory_replay(
        "AUTO_PARKU", auto_parku_traj_, auto_parku_loaded_,
        auto_parku_started_, auto_parku_clock_started_, auto_parku_done_,
        replay_t0_, 0, true, options);
    print_count_++;
}
