#include "lidar_nav_demo_cpp/lidar_nav_demo_node.hpp"
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdlib>
#include <string>
#include <vector>

void LidarNavControl::priority_callback(const std_msgs::msg::String::SharedPtr msg) {

    if (path_generated_) {
        // RCLCPP_WARN(this->get_logger(), "Path already generated. Ignoring new priority sequence.");
        return;
    }

    const std::string &s = msg->data;
    if (s.size() != 8) {
        RCLCPP_ERROR(this->get_logger(),
                     "Invalid eightboxes string length: %zu, expect 8. raw='%s'",
                     s.size(), s.c_str());
        return;
    }

    std::vector<int> priorities;
    priorities.reserve(8);
    for (char c : s) {
        if (c < '0' || c > '9') {
            RCLCPP_ERROR(this->get_logger(),
                         "Invalid char in eightboxes string: '%c'. raw='%s'",
                         c, s.c_str());
            return;
        }
        const int v = c - '0';
        if (v < 0 || v > 6) {
            RCLCPP_ERROR(this->get_logger(),
                         "Class id out of range [0,6]: %d. raw='%s'",
                         v, s.c_str());
            return;
        }
        priorities.push_back(v);
    }

    const std::vector<int> raw_classes = priorities;
    std::vector<int> normalized_classes = raw_classes;
    if (normalized_classes.size() == 8) {
        std::reverse(normalized_classes.begin() + 4, normalized_classes.end());
    }
    pending_eightboxes_raw_text_ = s;

    if (!ocr_result_locked_) {
        const bool duplicate_eightboxes =
            pending_priorities_valid_ && pending_raw_classes_ == normalized_classes;
        pending_raw_classes_ = normalized_classes;
        pending_priorities_valid_ = true;

        if (duplicate_eightboxes && ocr_wait_armed_) {
            return;
        }

        arm_ocr_wait_after_eightboxes();
        RCLCPP_INFO(this->get_logger(),
                    "eightboxes stored ('%s'); waiting up to %.1f s for OCR before left-to-right fallback",
                    s.c_str(), ocr_wait_after_eightboxes_sec_);
        return;
    }

    pending_raw_classes_ = normalized_classes;
    pending_priorities_valid_ = true;
    try_generate_path_with_pending_priorities();
}

void LidarNavControl::ocr_result_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (ocr_result_locked_) {
        return;
    }

    constexpr int kOcrStableStreak = 1;

    try {
        const auto j = nlohmann::json::parse(msg->data);

        if (!j.contains("result")) {
            RCLCPP_WARN(this->get_logger(), "JSON 中没有 result 字段: %s", msg->data.c_str());
            return;
        }

        const std::string expression = j.value("expression", "");
        const double result_value = j.at("result").get<double>();

        const long long n = std::llround(result_value);
        const int rem = static_cast<int>((((n % 4) + 4) % 4));

        if (rem == ocr_streak_candidate_) {
            ocr_streak_count_++;
        } else {
            ocr_streak_candidate_ = rem;
            ocr_streak_count_ = 1;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "expression='%s', result=%.6f, int=%lld, int%%4=%d (streak %d/%d for rem=%d)",
            expression.c_str(), result_value, static_cast<long long>(n), rem,
            ocr_streak_count_, kOcrStableStreak, ocr_streak_candidate_);

        if (ocr_streak_count_ >= kOcrStableStreak) {
            ocr_result_ = rem;
            ocr_result_locked_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "OCR result locked: ocr_result_=%d after %d consecutive identical rem",
                        ocr_result_, kOcrStableStreak);
            try_generate_path_with_pending_priorities();
        }
    } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(),
                    "无法解析 OCR JSON: '%s', err=%s",
                    msg->data.c_str(), e.what());
    }
}

void LidarNavControl::tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    tf_count_++;
    if (!printed_frames_ && tf_count_ <= 5) {
        // 打印前几个TF帧的信息以供调试
        if (tf_count_ == 5) printed_frames_ = true;
    }

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

            geometry_msgs::msg::Point pose_msg;
            pose_msg.x = x_;
            pose_msg.y = y_;
            pose_msg.z = yaw_;
            lidar_pose_pub_->publish(pose_msg);
            break;
        }
    }
}

void LidarNavControl::control_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 4) {
        RCLCPP_WARN(this->get_logger(), "Control array too short, need 4 elements");
        return;
    }
    enable_control_ = static_cast<bool>(msg->data[0]);
    
    // 如果收到新的话题控制指令，则覆盖现有的序列，仅执行该单一目标点作为临时过渡
    waypoints_.clear();
    waypoints_.push_back({msg->data[1], msg->data[2], msg->data[3], WaypointType::TRANSITION});
    current_wp_idx_ = 0;
    reset_arrival_debounce();
    reset_graph_yaw_align();
    logistics_prev_ctrl_time_valid_ = false;
    reset_logistics_cmd_slew();
    if (logistics_controller_) {
        logistics_controller_->reset();
    }
    is_waiting_ = false; // 收到新指令时重置等待状态
    
    RCLCPP_INFO(this->get_logger(), "Control updated via topic: enable=%d, target=(%.2f, %.2f, %.2f)",
        enable_control_, waypoints_[0].x, waypoints_[0].y, waypoints_[0].yaw);
}
