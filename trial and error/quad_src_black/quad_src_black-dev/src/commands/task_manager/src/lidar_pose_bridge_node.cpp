/**
 * @brief 独立雷达位姿桥接节点：仅订阅 TF，持续发布 /quad/lidar_pose_xyyaw。
 *
 * 不依赖任务赛状态机、遥控器 ENTER_AUTO、logistics_nav_enable 等。
 * 只要 point_lio（或其它定位）在 TF 树中提供 target_frame -> child_frame，即可输出位姿。
 */
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class LidarPoseBridgeNode : public rclcpp::Node {
public:
    LidarPoseBridgeNode()
        : Node("lidar_pose_bridge_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        this->declare_parameter<std::string>("target_frame", "camera_init");
        this->declare_parameter<std::string>("child_frame", "aft_mapped");
        this->declare_parameter<std::string>(
            "lidar_pose_topic", "/quad/lidar_pose_xyyaw");
        this->declare_parameter<double>("publish_rate_hz", 20.0);

        target_frame_ = this->get_parameter("target_frame").as_string();
        child_frame_ = this->get_parameter("child_frame").as_string();
        lidar_pose_topic_ = this->get_parameter("lidar_pose_topic").as_string();
        const double rate_hz = this->get_parameter("publish_rate_hz").as_double();

        pose_pub_ = this->create_publisher<geometry_msgs::msg::Point>(lidar_pose_topic_, 10);

        const auto period = std::chrono::duration<double>(1.0 / std::max(rate_hz, 1.0));
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&LidarPoseBridgeNode::publishPose, this));

        RCLCPP_INFO(this->get_logger(), "Lidar pose bridge started (no nav mode required).");
        RCLCPP_INFO(this->get_logger(), "  TF: %s -> %s", target_frame_.c_str(), child_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  publish: %s @ %.1f Hz", lidar_pose_topic_.c_str(), rate_hz);
    }

private:
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

        geometry_msgs::msg::Point msg;
        msg.x = t.x;
        msg.y = t.y;
        msg.z = yaw;
        pose_pub_->publish(msg);
    }

    std::string target_frame_;
    std::string child_frame_;
    std::string lidar_pose_topic_;
    bool warned_no_tf_{false};

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarPoseBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
