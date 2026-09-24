#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "quad/msg/motor_state.hpp"

#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

class M3508ManipulatorNode : public rclcpp::Node {
public:
    M3508ManipulatorNode() : Node("m3508_manipulator_node"), sockfd_(-1), zero_offset_initialized_(false) {
        this->declare_parameter("interface", "can0");
        this->declare_parameter("current_limit",8000);
        std::string interface = this->get_parameter("interface").as_string();
        current_limit_ = this->get_parameter("current_limit").as_int();

        if (!init_can_socket(interface)) {
            RCLCPP_FATAL(this->get_logger(), "无法初始化 CAN 接口: %s", interface.c_str());
            return;
        }

        motor_cmds_.assign(2, 0);
        motor_states_.pos.assign(2, 0.0);
        motor_states_.vel.assign(2, 0.0);
        motor_states_.cur.assign(2, 0.0);
        last_raw_angle_.assign(2, 0.0);
        total_round_.assign(2,0.0);
        zero_offset_.assign(2, 0.0);

        state_pub_ = this->create_publisher<quad::msg::MotorState>(
            "/quad/manipulator/motor_state", 10);
        
        cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/quad/manipulator/motor_cmd", 10, 
            std::bind(&M3508ManipulatorNode::cmd_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2), 
            std::bind(&M3508ManipulatorNode::control_loop, this));

        refresh_mission_mode_from_task_manager();

        RCLCPP_INFO(this->get_logger(),
                    "3508电机驱动节点启动 (mission_mode=%s, CAN send errors %s)",
                    mission_mode_.c_str(),
                    log_can_send_failed_ ? "enabled" : "suppressed");
    }

    ~M3508ManipulatorNode() {
        if (sockfd_ >= 0) {
            send_can_frame(0, 0);
            close(sockfd_);
        }
    }

private:
    int sockfd_;
    int current_limit_;
    std::mutex data_mutex_;
    std::vector<int16_t> motor_cmds_;
    quad::msg::MotorState motor_states_;
    std::vector<double> last_raw_angle_;
    std::vector<double> total_round_;
    std::vector<double> zero_offset_;  ///< 零位偏置（启动时的位置）
    bool zero_offset_initialized_;      ///< 零位是否已初始化
    std::string mission_mode_{"logistics"};
    bool log_can_send_failed_{true};

    rclcpp::Publisher<quad::msg::MotorState>::SharedPtr state_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool init_can_socket(const std::string& interface) {
        struct ifreq ifr{};
        struct sockaddr_can addr{};

        sockfd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sockfd_ < 0) {
            return false;
        }

        std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';

        if (ioctl(sockfd_, SIOCGIFINDEX, &ifr) < 0) {
            close(sockfd_);
            sockfd_ = -1;
            return false;
        }

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(sockfd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
            close(sockfd_);
            sockfd_ = -1;
            return false;
        }

        int flags = fcntl(sockfd_, F_GETFL, 0);
        if (flags < 0 || fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK) < 0) {
            close(sockfd_);
            sockfd_ = -1;
            return false;
        }
        
        return true;
    }

    void refresh_mission_mode_from_task_manager() {
        auto param_client =
            std::make_shared<rclcpp::SyncParametersClient>(this, "/quad/task_manager_node");
        if (!param_client->wait_for_service(std::chrono::seconds(3))) {
            RCLCPP_WARN(this->get_logger(),
                        "task_manager_node unavailable; default mission_mode=logistics");
            mission_mode_ = "logistics";
            log_can_send_failed_ = true;
            return;
        }

        if (!param_client->has_parameter("mission_mode")) {
            RCLCPP_WARN(this->get_logger(),
                        "task_manager_node has no mission_mode; default mission_mode=logistics");
            mission_mode_ = "logistics";
            log_can_send_failed_ = true;
            return;
        }

        mission_mode_ = param_client->get_parameter<std::string>("mission_mode");
        log_can_send_failed_ = (mission_mode_ != "obstacle");
    }

    void cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (size_t i = 0; i < msg->data.size() && i < 2; ++i) {
            motor_cmds_[i] = std::clamp(static_cast<int16_t>(msg->data[i]), 
                                        static_cast<int16_t>(-current_limit_), 
                                        static_cast<int16_t>(current_limit_));
        }
    }

    double process_pos_feedback(int16_t current_raw_angle, int idx) {
        int16_t diff = current_raw_angle - last_raw_angle_[idx];
        
        // 处理过零跳变
        if (diff > 4096) total_round_[idx]--;      // 反向过零
        else if (diff < -4096) total_round_[idx]++; // 正向过零
        
        last_raw_angle_[idx] = current_raw_angle;
        
        // 计算总脉冲数
        long total_count = total_round_[idx] * 8192 + current_raw_angle;
        // 换算成输出轴弧度 (考虑1:19减速比)
        double output_rad = (total_count / 8192.0 / 19.2) * 2.0 * M_PI;
        return output_rad;
    }

    void control_loop() {
        struct can_frame rx_frame;
        while (read(sockfd_, &rx_frame, sizeof(struct can_frame)) > 0) {
            if (rx_frame.can_id >= 0x201 && rx_frame.can_id <= 0x202) {
                int i = rx_frame.can_id - 0x201;

                uint16_t raw_angle = (rx_frame.data[0] << 8) | rx_frame.data[1];
                int16_t  raw_rpm   = (rx_frame.data[2] << 8) | rx_frame.data[3];
                int16_t  raw_cur   = (rx_frame.data[4] << 8) | rx_frame.data[5];

                std::lock_guard<std::mutex> lock(data_mutex_);
                // 计算绝对位置
                double absolute_pos = process_pos_feedback(static_cast<int16_t>(raw_angle), i);

                // 首次运行时记录零位偏置
                if (!zero_offset_initialized_) {
                    zero_offset_[i] = absolute_pos;
                    if (i == 1) {  // 两个电机都初始化完成后标记为已初始化
                        zero_offset_initialized_ = true;
                        RCLCPP_INFO(this->get_logger(), "零位已初始化: motor0=%.4f rad, motor1=%.4f rad",
                                    zero_offset_[0], zero_offset_[1]);
                    }
                }

                // 发布相对位置（减去零位偏置）
                motor_states_.pos[i] = absolute_pos - zero_offset_[i];
                motor_states_.vel[i] = raw_rpm * (2.0 * M_PI / 60.0);
                motor_states_.cur[i] = static_cast<double>(raw_cur);
            }
        }

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            // motor_states_.header.stamp = this->now();
            state_pub_->publish(motor_states_);
        }

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            send_can_frame(motor_cmds_[0], motor_cmds_[1]);
        }
    }

    void send_can_frame(int16_t i1, int16_t i2) {
        struct can_frame tx_frame;
        tx_frame.can_id = 0x200;
        tx_frame.can_dlc = 8;
        tx_frame.data[0] = i1 >> 8; tx_frame.data[1] = i1 & 0xFF;
        tx_frame.data[2] = i2 >> 8; tx_frame.data[3] = i2 & 0xFF;
        tx_frame.data[4] = 0;       tx_frame.data[5] = 0;
        tx_frame.data[6] = 0;       tx_frame.data[7] = 0;

        if (write(sockfd_, &tx_frame, sizeof(tx_frame)) < 0 && log_can_send_failed_) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "CAN send failed: %s", strerror(errno));
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<M3508ManipulatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}