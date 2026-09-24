#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <quad/msg/motor_state.hpp>
#include <quad/msg/claw.hpp>
#include <cmath>

class SimpleClawControlNode : public rclcpp::Node {
public:
    SimpleClawControlNode() : Node("simple_claw_control") {
        this->declare_parameter("close_current", 5000.0);
        this->declare_parameter("open_target_pos", 0.05);
        this->declare_parameter("pd_kp", 6000.0);
        this->declare_parameter("pd_kd", 3.0);
        this->declare_parameter("homing_current", 500.0);
        this->declare_parameter("homing_duration_sec", 1.5);
        this->declare_parameter("control_freq_hz", 200.0);
        this->declare_parameter("motor_dir_left", 1.0);
        this->declare_parameter("motor_dir_right", -1.0);

        close_current_ = this->get_parameter("close_current").as_double();
        open_target_pos_ = this->get_parameter("open_target_pos").as_double();
        pd_kp_ = this->get_parameter("pd_kp").as_double();
        pd_kd_ = this->get_parameter("pd_kd").as_double();
        homing_current_ = this->get_parameter("homing_current").as_double();
        homing_duration_sec_ = this->get_parameter("homing_duration_sec").as_double();
        control_freq_hz_ = this->get_parameter("control_freq_hz").as_double();
        motor_dir_left_ = this->get_parameter("motor_dir_left").as_double();
        motor_dir_right_ = this->get_parameter("motor_dir_right").as_double();
        dt_ = 1.0 / control_freq_hz_;

        claw_state_ = 1;
        is_homing_ = true;
        homing_timer_ = 0.0;
        zero_position_left_ = 0.0;
        zero_position_right_ = 0.0;
        pos_left_ = 0.0;
        pos_right_ = 0.0;
        vel_left_ = 0.0;
        vel_right_ = 0.0;

        motor_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/quad/manipulator/motor_cmd", 10);

        motor_sub_ = this->create_subscription<quad::msg::MotorState>(
            "/quad/manipulator/motor_state", 10,
            [this](const quad::msg::MotorState::SharedPtr msg) {
                if (msg->pos.size() >= 2) {
                    pos_left_ = msg->pos[0] - zero_position_left_;
                    pos_right_ = msg->pos[1] - zero_position_right_;
                }
                if (msg->vel.size() >= 2) {
                    vel_left_ = msg->vel[0];
                    vel_right_ = msg->vel[1];
                }
            });

        claw_sub_ = this->create_subscription<quad::msg::Claw>(
            "/quad/manipulator/claw_state", 10,
            [this](const quad::msg::Claw::SharedPtr msg) {
                claw_state_ = msg->claw_state;
                RCLCPP_INFO(this->get_logger(), "Claw state changed to: %d", claw_state_);
            });

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_),
            std::bind(&SimpleClawControlNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(),
            "SimpleClawControl started: homing=%.1fs, close_current=%.0f, open_target=%.1f, dir=(%.0f,%.0f)",
            homing_duration_sec_, close_current_, open_target_pos_, motor_dir_left_, motor_dir_right_);
    }

    ~SimpleClawControlNode() {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = {0.0, 0.0};
        motor_pub_->publish(msg);
    }

private:
    void controlLoop() {
		pd_kp_ = this->get_parameter("pd_kp").as_double();
		pd_kd_ = this->get_parameter("pd_kd").as_double();
		
		double current_left = 0.0;
        double current_right = 0.0;

        if (is_homing_) {
            homing_timer_ += dt_;
            if (homing_timer_ < homing_duration_sec_) {
                double current = -homing_current_;
                current_left = current * motor_dir_left_;
                current_right = current * motor_dir_right_;
            } else {
                zero_position_left_ = pos_left_;
                zero_position_right_ = pos_right_;
                is_homing_ = false;
                RCLCPP_INFO(this->get_logger(),
                    "Homing complete. Zero position: (%.3f, %.3f)",
                    zero_position_left_, zero_position_right_);
            }
        } else {
            if (claw_state_ == 0) {
                double current = close_current_;
                current_left = current * motor_dir_left_;
                current_right = current * motor_dir_right_;
            } else {
                double error_left = open_target_pos_*motor_dir_left_ - pos_left_;
                double error_right = open_target_pos_*motor_dir_right_ - pos_right_;

                current_left = pd_kp_ * error_left - pd_kd_ * vel_left_;
                current_right = pd_kp_ * error_right - pd_kd_ * vel_right_;
            }
        }

        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.resize(2);
        msg.data[0] = current_left;
        msg.data[1] = current_right;
        motor_pub_->publish(msg);
    }

    double close_current_;
    double open_target_pos_;
    double pd_kp_;
    double pd_kd_;
    double homing_current_;
    double homing_duration_sec_;
    double control_freq_hz_;
    double dt_;
    double motor_dir_left_;
    double motor_dir_right_;

    uint8_t claw_state_;
    bool is_homing_;
    double homing_timer_;
    double zero_position_left_;
    double zero_position_right_;
    double pos_left_;
    double pos_right_;
    double vel_left_;
    double vel_right_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_pub_;
    rclcpp::Subscription<quad::msg::MotorState>::SharedPtr motor_sub_;
    rclcpp::Subscription<quad::msg::Claw>::SharedPtr claw_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleClawControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
