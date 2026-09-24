#include <asm/termbits.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <iostream>
#include <sensor_msgs/msg/imu.hpp>

#include "quad/msg/quadimu.hpp"
#include "quad/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

#ifdef __cplusplus
extern "C" {
#endif
#include <poll.h>

#include "hipnuc_lib_package/hipnuc_dec.h"

#define GRA_ACC (9.8)
#define DEG_TO_RAD (0.01745329)
#define BUF_SIZE (1024)
#ifdef __cplusplus
}
#endif

namespace hipnuc_driver {
using namespace std::chrono_literals;
using namespace std;
static hipnuc_raw_t raw;

class IMUPublisher : public rclcpp::Node {
   public:
    int fd = -1;  // 初始化为 -1 表示未连接
    uint8_t buf[BUF_SIZE] = {0};
    bool is_connected_ = false;
    rclcpp::Time last_data_time_;           // 最后收到数据的时间
    rclcpp::Time last_reconnect_attempt_;   // 上次重连尝试时间
    const double RECONNECT_INTERVAL = 1.0;  // 重连间隔（秒）
    const double DATA_TIMEOUT = 2.0;        // 数据超时时间（秒）

    IMUPublisher(const rclcpp::NodeOptions& options)
        : Node("IMU_publisher", options) {
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate",460800);
        this->declare_parameter<std::string>("frame_id", "base_link");
        this->declare_parameter<std::string>("imu_topic",
                                             "/quad/test_estm_state");

        this->get_parameter("serial_port", serial_port);
        this->get_parameter("baud_rate", baud_rate);
        this->get_parameter("frame_id", frame_id);
        this->get_parameter("imu_topic", imu_topic);

        RCLCPP_INFO(this->get_logger(), "serial_port: %s\r\n",
                    serial_port.c_str());
        RCLCPP_INFO(this->get_logger(), "baud_rate: %d\r\n", baud_rate);
        RCLCPP_INFO(this->get_logger(), "frame_id: %s\r\n", frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "imu_topic: %s\r\n", imu_topic.c_str());

        imu_pub = this->create_publisher<quad::msg::State>(imu_topic, 20);

        // ✅ 修改：初始连接
        if (connect_serial() < 0) {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to connect to IMU initially");
        }

        // ✅ 修改：使用定时器而不是阻塞循环
        // 创建定时器，1ms 周期（1000Hz）
        read_timer_ = this->create_wall_timer(
            1ms, std::bind(&IMUPublisher::imu_read_timer_callback, this));

        // ✅ 添加：连接监控定时器（100ms 检查一次）
        monitor_timer_ = this->create_wall_timer(
            100ms, std::bind(&IMUPublisher::monitor_connection, this));
    }

    ~IMUPublisher() {
        // ✅ 添加：析构时关闭串口
        if (fd >= 0) {
            close(fd);
            fd = -1;
        }
    }

   private:
    // ✅ 添加：连接串口的函数
    int connect_serial() {
        // 如果已经连接，先关闭
        if (fd >= 0) {
            close(fd);
            fd = -1;
            is_connected_ = false;
        }

        RCLCPP_INFO(this->get_logger(), "Attempting to connect to %s...",
                    serial_port.c_str());

        fd = open_ttyport(serial_port, baud_rate);

        if (fd < 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to open serial port: %s",
                        serial_port.c_str());
            is_connected_ = false;
            return -1;
        }

        is_connected_ = true;
        last_data_time_ = this->now();
        last_reconnect_attempt_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Successfully connected to IMU on %s",
                    serial_port.c_str());
        return 0;
    }

    // ✅ 添加：监控连接状态
    void monitor_connection() {
        if (!is_connected_ || fd < 0) {
            // 未连接，尝试重连
            rclcpp::Time now = this->now();
            if ((now - last_reconnect_attempt_).seconds() >=
                RECONNECT_INTERVAL) {
                last_reconnect_attempt_ = now;
                connect_serial();
            }
            return;
        }

        // 检查数据超时
        rclcpp::Time now = this->now();
        double time_since_data = (now - last_data_time_).seconds();

        if (time_since_data > DATA_TIMEOUT) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(),
                2000,  // 每2秒打印一次
                "No data received from IMU for %.2f seconds, reconnecting...",
                time_since_data);

            // 标记为断开，下次重连
            is_connected_ = false;
            if (fd >= 0) {
                close(fd);
                fd = -1;
            }
        }
    }

    // ✅ 修改：定时器回调函数
    void imu_read_timer_callback(void) {
        if (!is_connected_ || fd < 0) {
            // 未连接，不读取
            return;
        }

        imu_read();
    }

    // ✅ 修改：添加错误检测
    void imu_read(void) {
        if (fd < 0) {
            return;
        }

        struct pollfd p;
        p.fd = fd;
        p.events = POLLIN;

        int rpoll = poll(&p, 1, 1);

        if (rpoll == 0) {
            return;  // 没有数据，正常返回
        }

        // ✅ 添加：检查 poll 错误
        if (rpoll < 0) {
            if (errno == EINTR) {
                // 被信号中断，忽略
                return;
            }
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Poll error on serial port: %s (errno=%d)",
                                 strerror(errno), errno);
            // 标记为断开
            is_connected_ = false;
            close(fd);
            fd = -1;
            return;
        }

        // ✅ 添加：检查 POLLERR 和 POLLHUP
        if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            RCLCPP_WARN(
                this->get_logger(),
                "Serial port error detected (revents=0x%x), reconnecting...",
                p.revents);
            is_connected_ = false;
            close(fd);
            fd = -1;
            return;
        }

        int n = read(fd, buf, sizeof(buf));

        // ✅ 添加：检查 read 错误
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // 非阻塞模式，没有数据是正常的
                return;
            }

            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Read error on serial port: %s (errno=%d)",
                                 strerror(errno), errno);

            // 标记为断开
            is_connected_ = false;
            close(fd);
            fd = -1;
            return;
        }

        // ✅ 添加：read 返回 0 可能表示连接断开
        if (n == 0) {
            // 在某些情况下，read 返回 0 可能表示 EOF
            // 但非阻塞模式下，这通常只是没有数据
            return;
        }

        // ✅ 更新最后收到数据的时间
        last_data_time_ = this->now();

        // 处理接收到的数据
        for (int i = 0; i < n; i++) {
            int rev = hipnuc_input(&raw, buf[i]);

            if (rev) {
                auto imu_msg = quad::msg::State();
                if (raw.hi91.tag == 0x91) {
                    double rw = raw.hi91.quat[0];
                    double rx = raw.hi91.quat[1];
                    double ry = raw.hi91.quat[2];
                    double rz = raw.hi91.quat[3];

                    // imu_msg.imu_world.w = -rz;
                    // imu_msg.imu_world.x = ry;
                    // imu_msg.imu_world.y = -rx;
                    // imu_msg.imu_world.z = rw;

                    imu_msg.imu_world.w = rw;
                    imu_msg.imu_world.x = rx;
                    imu_msg.imu_world.y = ry;
                    imu_msg.imu_world.z = rz;

                    imu_msg.imu_world.ang_vel = {raw.hi91.gyr[0] * DEG_TO_RAD,
                                                 raw.hi91.gyr[1] * DEG_TO_RAD,
                                                 raw.hi91.gyr[2] * DEG_TO_RAD};
                }
                imu_pub->publish(std::move(imu_msg));
            }
        }
        memset(buf, 0, sizeof(buf));
    }

    int open_ttyport(std::string tty_port, int baud) {
        const char* port_device = tty_port.c_str();
        int serial_port = open(port_device, O_RDWR | O_NOCTTY | O_NONBLOCK);

        if (serial_port < 0) {
            perror("Error opening serial port");
            const std::string fallback_port = "/dev/ttyUSB5";
            RCLCPP_INFO(this->get_logger(), "Trying fallback port: %s",
                        fallback_port.c_str());
            serial_port =
                open(fallback_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (serial_port < 0) {
                perror("Error opening fallback serial port");
                return -1;
            }
            RCLCPP_INFO(this->get_logger(), "Using fallback port: %s",
                        fallback_port.c_str());
        }

        struct termios2 tty;

        if (ioctl(serial_port, TCGETS2, &tty) != 0) {
            perror("Error from TCGETS2 ioctl");
            close(serial_port);
            return -1;
        }

        tty.c_cflag &= ~CBAUD;
        tty.c_cflag |= BOTHER;

        tty.c_ispeed = baud;
        tty.c_ospeed = baud;

        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &=
            ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

        tty.c_cc[VTIME] = 10;
        tty.c_cc[VMIN] = 0;

        if (ioctl(serial_port, TCSETS2, &tty) != 0) {
            perror("Error from TCSETS2 IOCTL");
            close(serial_port);
            return -1;
        }

        return serial_port;
    }

    std::string serial_port;
    int baud_rate;
    std::string frame_id;
    std::string imu_topic;

    rclcpp::Publisher<quad::msg::State>::SharedPtr imu_pub;
    rclcpp::TimerBase::SharedPtr read_timer_;     // ✅ 添加：读取定时器
    rclcpp::TimerBase::SharedPtr monitor_timer_;  // ✅ 添加：监控定时器
};
};  // namespace hipnuc_driver

int main(int argc, const char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::spin(std::make_shared<hipnuc_driver::IMUPublisher>(options));
    rclcpp::shutdown();

    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hipnuc_driver::IMUPublisher)