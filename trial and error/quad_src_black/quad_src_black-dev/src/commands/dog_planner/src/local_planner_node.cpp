#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

struct State {
    double x, y, theta;
    double v, w;
};

class LocalPlannerNode : public rclcpp::Node
{
public:
    LocalPlannerNode() : Node("local_planner_node")
    {
        // Parameters
        this->declare_parameter("max_vel", 0.5);
        this->declare_parameter("max_omega", 1.0);
        this->declare_parameter("sim_time", 2.0);
        this->declare_parameter("dt", 0.1);
        this->declare_parameter("v_samples", 10);
        this->declare_parameter("w_samples", 20);
        this->declare_parameter("w_path", 1.0);
        this->declare_parameter("w_obstacle", 2.0);
        this->declare_parameter("w_goal", 1.0);
        this->declare_parameter("w_speed", 0.5);
        this->declare_parameter("robot_radius", 0.2);

        max_vel_ = this->get_parameter("max_vel").as_double();
        max_omega_ = this->get_parameter("max_omega").as_double();
        sim_time_ = this->get_parameter("sim_time").as_double();
        dt_ = this->get_parameter("dt").as_double();
        v_samples_ = this->get_parameter("v_samples").as_int();
        w_samples_ = this->get_parameter("w_samples").as_int();
        w_path_ = this->get_parameter("w_path").as_double();
        w_obstacle_ = this->get_parameter("w_obstacle").as_double();
        w_goal_ = this->get_parameter("w_goal").as_double();
        w_speed_ = this->get_parameter("w_speed").as_double();
        robot_radius_ = this->get_parameter("robot_radius").as_double();

        // Subscribers
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/global_path", 10, std::bind(&LocalPlannerNode::pathCallback, this, std::placeholders::_1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LocalPlannerNode::scanCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&LocalPlannerNode::odomCallback, this, std::placeholders::_1));

        // Publishers
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Timer for control loop (e.g. 10 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&LocalPlannerNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Local Planner Node has been started.");
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        global_path_ = *msg;
        has_path_ = true;
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        latest_scan_ = *msg;
        has_scan_ = true;
        
        // Convert scan to obstacles in base_link frame
        obstacles_.clear();
        double angle = latest_scan_.angle_min;
        for (size_t i = 0; i < latest_scan_.ranges.size(); ++i) {
            double r = latest_scan_.ranges[i];
            if (std::isfinite(r) && r > latest_scan_.range_min && r < latest_scan_.range_max) {
                double x = r * std::cos(angle);
                double y = r * std::sin(angle);
                obstacles_.push_back({x, y});
            }
            angle += latest_scan_.angle_increment;
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_odom_ = *msg;
        
        // Update current state
        current_state_.x = msg->pose.pose.position.x;
        current_state_.y = msg->pose.pose.position.y;
        current_state_.theta = tf2::getYaw(msg->pose.pose.orientation);
        current_state_.v = msg->twist.twist.linear.x;
        current_state_.w = msg->twist.twist.angular.z;
        
        has_odom_ = true;
    }

    std::vector<State> generateTrajectory(double v, double w) {
        std::vector<State> trajectory;
        // Simulate in base_link frame
        State local_state = {0.0, 0.0, 0.0, v, w};
        double time = 0.0;
        
        while (time <= sim_time_) {
            local_state.x += v * std::cos(local_state.theta) * dt_;
            local_state.y += v * std::sin(local_state.theta) * dt_;
            local_state.theta += w * dt_;
            trajectory.push_back(local_state);
            time += dt_;
        }
        return trajectory;
    }

    double calculateObstacleCost(const std::vector<State>& trajectory) {
        double min_dist = std::numeric_limits<double>::max();
        for (const auto& state : trajectory) {
            for (const auto& obs : obstacles_) {
                double dist = std::hypot(state.x - obs.first, state.y - obs.second);
                if (dist <= robot_radius_) {
                    return std::numeric_limits<double>::max(); // Collision
                }
                if (dist < min_dist) {
                    min_dist = dist;
                }
            }
        }
        return 1.0 / (min_dist + 1e-5);
    }
    
    bool getLocalGoal(double& gx, double& gy) {
        if (global_path_.poses.empty()) return false;
        
        // Find the closest point on the global path
        double min_dist = std::numeric_limits<double>::max();
        int closest_idx = 0;
        for (size_t i = 0; i < global_path_.poses.size(); ++i) {
            double dx = global_path_.poses[i].pose.position.x - current_state_.x;
            double dy = global_path_.poses[i].pose.position.y - current_state_.y;
            double dist = std::hypot(dx, dy);
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }
        
        // Lookahead distance
        double lookahead_dist = 1.0;
        int goal_idx = closest_idx;
        for (size_t i = closest_idx; i < global_path_.poses.size(); ++i) {
            double dx = global_path_.poses[i].pose.position.x - current_state_.x;
            double dy = global_path_.poses[i].pose.position.y - current_state_.y;
            double dist = std::hypot(dx, dy);
            if (dist >= lookahead_dist) {
                goal_idx = i;
                break;
            }
            goal_idx = i;
        }
        
        // Transform goal to base_link frame
        double global_gx = global_path_.poses[goal_idx].pose.position.x;
        double global_gy = global_path_.poses[goal_idx].pose.position.y;
        
        double dx = global_gx - current_state_.x;
        double dy = global_gy - current_state_.y;
        
        gx = std::cos(-current_state_.theta) * dx - std::sin(-current_state_.theta) * dy;
        gy = std::sin(-current_state_.theta) * dx + std::cos(-current_state_.theta) * dy;
        
        return true;
    }

    void controlLoop()
    {
        if (!has_odom_ || !has_path_ || !has_scan_) {
            return;
        }
        
        if (global_path_.poses.empty()) {
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_pub_->publish(cmd);
            return;
        }

        double local_gx, local_gy;
        if (!getLocalGoal(local_gx, local_gy)) return;
        
        // Check if reached final goal
        double final_dx = global_path_.poses.back().pose.position.x - current_state_.x;
        double final_dy = global_path_.poses.back().pose.position.y - current_state_.y;
        if (std::hypot(final_dx, final_dy) < 0.2) {
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_pub_->publish(cmd);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Goal reached!");
            return;
        }

        double best_v = 0.0;
        double best_w = 0.0;
        double min_cost = std::numeric_limits<double>::max();

        double v_step = max_vel_ / std::max(1, v_samples_ - 1);
        double w_step = 2.0 * max_omega_ / std::max(1, w_samples_ - 1);

        for (int i = 0; i < v_samples_; ++i) {
            double v = i * v_step; // 0 to max_vel
            for (int j = 0; j < w_samples_; ++j) {
                double w = -max_omega_ + j * w_step;

                auto trajectory = generateTrajectory(v, w);
                
                double obs_cost = calculateObstacleCost(trajectory);
                if (obs_cost == std::numeric_limits<double>::max()) continue; // Collision

                const auto& last_state = trajectory.back();
                
                // Goal cost (angle to local goal)
                double angle_to_goal = std::atan2(local_gy - last_state.y, local_gx - last_state.x);
                double goal_cost = std::abs(angle_to_goal - last_state.theta);
                
                // Path cost (distance to local goal)
                double path_cost = std::hypot(local_gx - last_state.x, local_gy - last_state.y);
                
                // Speed cost
                double speed_cost = max_vel_ - v;
                
                double total_cost = w_path_ * path_cost + w_obstacle_ * obs_cost + w_goal_ * goal_cost + w_speed_ * speed_cost;

                if (total_cost < min_cost) {
                    min_cost = total_cost;
                    best_v = v;
                    best_w = w;
                }
            }
        }

        geometry_msgs::msg::Twist cmd;
        if (min_cost == std::numeric_limits<double>::max()) {
            // Recovery behavior: spin in place
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.5;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No valid trajectory found, recovering!");
        } else {
            cmd.linear.x = best_v;
            cmd.angular.z = best_w;
        }

        cmd_pub_->publish(cmd);
    }

    double max_vel_, max_omega_, sim_time_, dt_;
    int v_samples_, w_samples_;
    double w_path_, w_obstacle_, w_goal_, w_speed_, robot_radius_;

    nav_msgs::msg::Path global_path_;
    sensor_msgs::msg::LaserScan latest_scan_;
    nav_msgs::msg::Odometry current_odom_;
    State current_state_;
    std::vector<std::pair<double, double>> obstacles_;

    bool has_path_ = false;
    bool has_scan_ = false;
    bool has_odom_ = false;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
