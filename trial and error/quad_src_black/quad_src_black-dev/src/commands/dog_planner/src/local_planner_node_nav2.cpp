#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "geometry_msgs/msg/twist.hpp"

using FollowPath = nav2_msgs::action::FollowPath;
using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

class LocalPlannerNodeNav2 : public rclcpp::Node
{
public:
    LocalPlannerNodeNav2() : Node("local_planner_node_nav2")
    {
        // 订阅 Global Planner 发布的路径
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/global_path", 10, std::bind(&LocalPlannerNodeNav2::pathCallback, this, std::placeholders::_1));
        
        // 创建 Action Client 连接官方的 Nav2 Controller Server
        action_client_ = rclcpp_action::create_client<FollowPath>(
            this, "local_costmap/follow_path"); // 注意: Nav2 默认使用 local_costmap/follow_path 或 follow_path

        RCLCPP_INFO(this->get_logger(), "Nav2 DWB Bridge Node started. Waiting for global paths to forward...");
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Nav2 FollowPath action server not available. Make sure nav2_controller is running!");
            return;
        }

        auto goal_msg = FollowPath::Goal();
        goal_msg.path = *msg;
        // 告诉 Nav2 Controller 使用哪个插件来跟随路径（在 nav2_params.yaml 中配置）
        goal_msg.controller_id = "FollowPath"; 
        goal_msg.goal_checker_id = "goal_checker";

        auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
            [this](const GoalHandleFollowPath::SharedPtr & goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Nav2 Controller rejected the path!");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Nav2 Controller accepted the path, DWB is now driving...");
                }
            };

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp_action::Client<FollowPath>::SharedPtr action_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalPlannerNodeNav2>());
    rclcpp::shutdown();
    return 0;
}
