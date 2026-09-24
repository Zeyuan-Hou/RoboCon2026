#include <unistd.h>
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include "quad/msg/quadimu.hpp"
#include "quad/msg/state.hpp"
#include <iomanip>

rclcpp::Node::SharedPtr nh = nullptr;
using namespace std;

void topic_callback(const quad::msg::State::SharedPtr msg)
{
	static int i = 0;
	if (i++ % 100)
		return ;
    RCLCPP_INFO(rclcpp::get_logger("imu_sub"),
        "\n"
        "orientation [rad]:\n"
        "  x: %.18f\n"
        "  y: %.18f\n"
        "  z: %.18f\n"
        "  w: %.18f\n"
        "angular_velocity [rad/s]:\n"
        "  x: %.18f\n"
        "  y: %.18f\n"
        "  z: %.18f",
        msg->imu_world.x,
        msg->imu_world.y,
        msg->imu_world.z,
        msg->imu_world.w,
        msg->imu_world.ang_vel[0],
        msg->imu_world.ang_vel[1],
        msg->imu_world.ang_vel[2]);
}

int main(int argc,const char* argv[])
{
	rclcpp::init(argc, argv);
	nh = std::make_shared<rclcpp::Node>("imu_sub");
	rclcpp::Subscription<quad::msg::State>::SharedPtr imu_sub ;
	imu_sub = nh->create_subscription<quad::msg::State>("/quad/test_estm_state", 10,topic_callback);
	rclcpp::spin(nh);
	rclcpp::shutdown();

	return 0;
}


