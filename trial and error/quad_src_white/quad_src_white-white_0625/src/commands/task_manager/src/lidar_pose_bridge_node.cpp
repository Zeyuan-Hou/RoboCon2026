/**
 * @brief 独立雷达位姿桥接节点：订阅雷达 TF，发布旧流程位姿和 Nav2 所需 odom/TF。
 *
 * 不依赖任务赛状态机、遥控器 ENTER_AUTO、logistics_nav_enable 等。
 * 只要 point_lio（或其它定位）在 TF 树中提供 target_frame -> child_frame，即可输出位姿。
 */
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

class LidarPoseBridgeNode : public rclcpp::Node {
public:
    LidarPoseBridgeNode()
        : Node("lidar_pose_bridge_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_),
          tf_broadcaster_(this)
    {
        this->declare_parameter<std::string>("target_frame", "camera_init");
        this->declare_parameter<std::string>("child_frame", "aft_mapped");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<std::string>(
            "lidar_pose_topic", "/quad/lidar_pose_xyyaw");
        this->declare_parameter<std::string>("odom_topic", "/quad/odom");
        this->declare_parameter<bool>("publish_nav2_tf", true);
        this->declare_parameter<bool>("publish_odom", true);
        this->declare_parameter<double>("publish_rate_hz", 20.0);
        this->declare_parameter<double>("velocity_filter_alpha", 0.20);
        this->declare_parameter<double>("linear_velocity_deadband", 0.02);
        this->declare_parameter<double>("angular_velocity_deadband", 0.02);
        this->declare_parameter<double>("max_linear_velocity", 0.80);
        this->declare_parameter<double>("max_angular_velocity", 1.50);
        this->declare_parameter<double>("velocity_window_s", 0.25);

        target_frame_ = this->get_parameter("target_frame").as_string();
        child_frame_ = this->get_parameter("child_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        lidar_pose_topic_ = this->get_parameter("lidar_pose_topic").as_string();
        odom_topic_ = this->get_parameter("odom_topic").as_string();
        publish_nav2_tf_ = this->get_parameter("publish_nav2_tf").as_bool();
        publish_odom_ = this->get_parameter("publish_odom").as_bool();
        const double rate_hz = this->get_parameter("publish_rate_hz").as_double();
        velocity_filter_alpha_ = this->get_parameter("velocity_filter_alpha").as_double();
        linear_velocity_deadband_ = this->get_parameter("linear_velocity_deadband").as_double();
        angular_velocity_deadband_ = this->get_parameter("angular_velocity_deadband").as_double();
        max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
        max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
        velocity_window_s_ = this->get_parameter("velocity_window_s").as_double();

        velocity_filter_alpha_ = std::min(1.0, std::max(0.0, velocity_filter_alpha_));
        linear_velocity_deadband_ = std::max(0.0, linear_velocity_deadband_);
        angular_velocity_deadband_ = std::max(0.0, angular_velocity_deadband_);
        max_linear_velocity_ = std::max(0.0, max_linear_velocity_);
        max_angular_velocity_ = std::max(0.0, max_angular_velocity_);
        velocity_window_s_ = std::max(0.02, velocity_window_s_);

        pose_pub_ = this->create_publisher<geometry_msgs::msg::Point>(lidar_pose_topic_, 10);
        if (publish_odom_) {
            odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
        }

        const auto period = std::chrono::duration<double>(1.0 / std::max(rate_hz, 1.0));
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&LidarPoseBridgeNode::publishPose, this));

        RCLCPP_INFO(this->get_logger(), "Lidar pose bridge started (no nav mode required).");
        RCLCPP_INFO(this->get_logger(), "  TF: %s -> %s", target_frame_.c_str(), child_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  publish: %s @ %.1f Hz", lidar_pose_topic_.c_str(), rate_hz);
        if (publish_odom_) {
            RCLCPP_INFO(this->get_logger(), "  publish: %s", odom_topic_.c_str());
        }
        if (publish_nav2_tf_) {
            RCLCPP_INFO(
                this->get_logger(), "  Nav2 TF: %s -> %s -> %s",
                map_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
        }
        RCLCPP_INFO(
            this->get_logger(),
            "  velocity filter: window=%.2fs alpha=%.2f linear_deadband=%.3f angular_deadband=%.3f max_linear=%.2f max_angular=%.2f",
            velocity_window_s_,
            velocity_filter_alpha_, linear_velocity_deadband_, angular_velocity_deadband_,
            max_linear_velocity_, max_angular_velocity_);
    }

private:
    struct PoseSample {
        rclcpp::Time stamp;
        double x{0.0};
        double y{0.0};
        double z{0.0};
        double yaw{0.0};
    };

    static double clampAbs(double value, double limit)
    {
        if (limit <= 0.0) {
            return value;
        }
        return std::max(-limit, std::min(value, limit));
    }

    static double applyDeadband(double value, double deadband)
    {
        return std::abs(value) < deadband ? 0.0 : value;
    }

    static double normalizeAngle(double angle)
    {
        while (angle > M_PI) {
            angle -= 2.0 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2.0 * M_PI;
        }
        return angle;
    }

    void publishPose()
    {
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_.lookupTransform(
                target_frame_, child_frame_, tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            if (!warned_no_tf_) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "Waiting for TF %s -> %s: %s",
                    target_frame_.c_str(), child_frame_.c_str(), ex.what());
                warned_no_tf_ = true;
            }
            return;
        }

        warned_no_tf_ = false;
        const auto& t = tf.transform.translation;
        const auto& q = tf.transform.rotation;

        const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        const double yaw = std::atan2(siny_cosp, cosy_cosp);
        const auto stamp = this->now();

        geometry_msgs::msg::Point msg;
        msg.x = t.x;
        msg.y = t.y;
        msg.z = yaw;
        pose_pub_->publish(msg);

        pose_history_.push_back(PoseSample{stamp, t.x, t.y, t.z, yaw});
        while (pose_history_.size() > 2 &&
               (stamp - pose_history_.front().stamp).seconds() > velocity_window_s_ * 2.0) {
            pose_history_.pop_front();
        }

        if (publish_odom_ && odom_pub_) {
            nav_msgs::msg::Odometry odom;
            odom.header.stamp = stamp;
            odom.header.frame_id = odom_frame_;
            odom.child_frame_id = base_frame_;
            odom.pose.pose.position.x = t.x;
            odom.pose.pose.position.y = t.y;
            odom.pose.pose.position.z = t.z;
            odom.pose.pose.orientation = q;
            odom.pose.covariance[0] = 0.02;
            odom.pose.covariance[7] = 0.02;
            odom.pose.covariance[14] = 0.25;
            odom.pose.covariance[21] = 0.25;
            odom.pose.covariance[28] = 0.25;
            odom.pose.covariance[35] = 0.05;

            const PoseSample* velocity_reference = nullptr;
            for (const auto& sample : pose_history_) {
                const double age = (stamp - sample.stamp).seconds();
                if (age >= velocity_window_s_) {
                    velocity_reference = &sample;
                } else {
                    break;
                }
            }
            if (!velocity_reference && pose_history_.size() >= 2) {
                velocity_reference = &pose_history_.front();
            }

            if (velocity_reference) {
                const double dt = (stamp - velocity_reference->stamp).seconds();
                if (dt > 1e-3 && dt < 1.0) {
                    const double dx = t.x - velocity_reference->x;
                    const double dy = t.y - velocity_reference->y;
                    const double dz = t.z - velocity_reference->z;
                    const double cos_yaw = std::cos(yaw);
                    const double sin_yaw = std::sin(yaw);
                    const double raw_vx = clampAbs(
                        (cos_yaw * dx + sin_yaw * dy) / dt, max_linear_velocity_);
                    const double raw_vy = clampAbs(
                        (-sin_yaw * dx + cos_yaw * dy) / dt, max_linear_velocity_);
                    const double raw_vz = clampAbs(dz / dt, max_linear_velocity_);
                    const double raw_wz = clampAbs(
                        normalizeAngle(yaw - velocity_reference->yaw) / dt, max_angular_velocity_);

                    if (!has_filtered_twist_) {
                        filtered_vx_ = raw_vx;
                        filtered_vy_ = raw_vy;
                        filtered_vz_ = raw_vz;
                        filtered_wz_ = raw_wz;
                        has_filtered_twist_ = true;
                    } else {
                        filtered_vx_ += velocity_filter_alpha_ * (raw_vx - filtered_vx_);
                        filtered_vy_ += velocity_filter_alpha_ * (raw_vy - filtered_vy_);
                        filtered_vz_ += velocity_filter_alpha_ * (raw_vz - filtered_vz_);
                        filtered_wz_ += velocity_filter_alpha_ * (raw_wz - filtered_wz_);
                    }

                    odom.twist.twist.linear.x =
                        applyDeadband(filtered_vx_, linear_velocity_deadband_);
                    odom.twist.twist.linear.y =
                        applyDeadband(filtered_vy_, linear_velocity_deadband_);
                    odom.twist.twist.linear.z =
                        applyDeadband(filtered_vz_, linear_velocity_deadband_);
                    odom.twist.twist.angular.z =
                        applyDeadband(filtered_wz_, angular_velocity_deadband_);
                }
            }
            odom.twist.covariance[0] = 0.05;
            odom.twist.covariance[7] = 0.05;
            odom.twist.covariance[14] = 0.50;
            odom.twist.covariance[21] = 0.50;
            odom.twist.covariance[28] = 0.50;
            odom.twist.covariance[35] = 0.10;
            odom_pub_->publish(odom);
        }

        if (publish_nav2_tf_) {
            geometry_msgs::msg::TransformStamped map_to_odom;
            map_to_odom.header.stamp = stamp;
            map_to_odom.header.frame_id = map_frame_;
            map_to_odom.child_frame_id = odom_frame_;
            map_to_odom.transform.rotation.w = 1.0;

            geometry_msgs::msg::TransformStamped odom_to_base;
            odom_to_base.header.stamp = stamp;
            odom_to_base.header.frame_id = odom_frame_;
            odom_to_base.child_frame_id = base_frame_;
            odom_to_base.transform.translation = t;
            odom_to_base.transform.rotation = q;

            tf_broadcaster_.sendTransform(
                std::vector<geometry_msgs::msg::TransformStamped>{
                    map_to_odom,
                    odom_to_base});
        }
    }

    std::string target_frame_;
    std::string child_frame_;
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string lidar_pose_topic_;
    std::string odom_topic_;
    bool publish_nav2_tf_{true};
    bool publish_odom_{true};
    bool warned_no_tf_{false};
    bool has_filtered_twist_{false};
    double velocity_filter_alpha_{0.20};
    double linear_velocity_deadband_{0.02};
    double angular_velocity_deadband_{0.02};
    double max_linear_velocity_{0.80};
    double max_angular_velocity_{1.50};
    double velocity_window_s_{0.25};
    double filtered_vx_{0.0};
    double filtered_vy_{0.0};
    double filtered_vz_{0.0};
    double filtered_wz_{0.0};
    std::deque<PoseSample> pose_history_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarPoseBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
