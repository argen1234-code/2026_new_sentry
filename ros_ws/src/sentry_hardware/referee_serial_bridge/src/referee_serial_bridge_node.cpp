#include <rclcpp/rclcpp.hpp>
#include <sentry_interfaces/msg/gimbal_data.hpp>

#include <vector>
#include <cstdint>
#include <iostream>
#include <iomanip>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <thread>

/**
 * @brief 裁判数据解析节点
 * 
 * 该节点直接从串口读取 72 字节的 uproto 协议帧，
 * 并解析出其中集成的裁判系统数据，发布到 /gimbal_data 话题。
 * 
 * 协议格式 (72字节):
 * [0:1]   AA 55 (Header)
 * [2:3]   01 00 (MUX/Other)
 * [4:5]   25 00 (Length?)
 * ...
 * [60:67] 裁判数据 (8字节):
 *         [0] game_progress
 *         [1:2] stage_remain_time (LE)
 *         [3] game_result
 *         [4:5] current_hp (LE)
 *         [6:7] shooter_42mm_heat (LE)
 * [68:71] CRC32
 */

class RefereeSerialBridgeNode : public rclcpp::Node
{
public:
    RefereeSerialBridgeNode() : Node("referee_serial_bridge")
    {
        // 声明参数
        this->declare_parameter("device", "/dev/ttyACM0");
        this->declare_parameter("baud_rate", 115200);

        std::string device = this->get_parameter("device").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();

        // 创建发布者
        gimbal_data_pub_ = this->create_publisher<sentry_interfaces::msg::GimbalData>(
            "/gimbal_data", 10);

        // 初始化串口
        if (initSerial(device, baud_rate)) {
            RCLCPP_INFO(this->get_logger(), "Serial port %s opened.", device.c_str());
            receive_thread_ = std::thread(&RefereeSerialBridgeNode::receiveLoop, this);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s.", device.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "Referee Serial Bridge Node started.");
    }

    ~RefereeSerialBridgeNode()
    {
        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }
        if (serial_fd_ != -1) {
            close(serial_fd_);
        }
    }

private:
    int serial_fd_ = -1;
    std::thread receive_thread_;

    bool initSerial(const std::string & device, int baud_rate)
    {
        serial_fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) return false;

        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) return false;

        speed_t speed;
        switch (baud_rate) {
            case 115200: speed = B115200; break;
            case 921600: speed = B921600; break;
            default: speed = B115200; break;
        }

        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 1;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) return false;
        return true;
    }

    uint32_t verify_crc32(const uint8_t * data, uint32_t len)
    {
        uint32_t crc = 0xFFFFFFFF;
        for (uint32_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320;
                else crc >>= 1;
            }
        }
        return ~crc;
    }

    void receiveLoop()
    {
        std::vector<uint8_t> buffer;
        uint8_t read_buf[1024];
        while (rclcpp::ok()) {
            int n = read(serial_fd_, read_buf, sizeof(read_buf));
            if (n > 0) {
                buffer.insert(buffer.end(), read_buf, read_buf + n);
                while (buffer.size() >= 72) {
                    bool found = false;
                    for (size_t i = 0; i <= buffer.size() - 72; ++i) {
                        if (buffer[i] == 0xAA && buffer[i+1] == 0x55) {
                            // 检查校验和 (最后4字节)
                            uint32_t received_crc = buffer[i+68] | (buffer[i+69] << 8) | 
                                                   (buffer[i+70] << 16) | (buffer[i+71] << 24);
                            uint32_t calculated_crc = verify_crc32(&buffer[i], 68);
                            
                            // 暂时注释掉 CRC 校验，直接解析数据
                            // if (received_crc == calculated_crc) {
                                // 提取裁判数据 (偏移 60 字节)
                                parseRefereeData(&buffer[i+60]);
                                buffer.erase(buffer.begin(), buffer.begin() + i + 72);
                                found = true;
                                break;
                            // } else {
                            //     // 校验和错误，跳过这个包头
                            //     buffer.erase(buffer.begin(), buffer.begin() + i + 2);
                            //     found = true;
                            //     break;
                            // }
                        }
                    }
                    if (!found) {
                        if (buffer.size() > 1024) buffer.erase(buffer.begin(), buffer.end() - 72);
                        break;
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    
    void parseRefereeData(const uint8_t * data)
    {
        auto gimbal_data_msg = sentry_interfaces::msg::GimbalData();
        gimbal_data_msg.header.stamp = this->now();
        gimbal_data_msg.game_progress = data[0];
        gimbal_data_msg.stage_remain_time = data[1] | (data[2] << 8);
        gimbal_data_msg.game_result = data[3];
        gimbal_data_msg.current_hp = data[4] | (data[5] << 8);
        gimbal_data_msg.shooter_42mm_heat = data[6] | (data[7] << 8);
        
        gimbal_data_pub_->publish(gimbal_data_msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Parsed Referee: Progress=%u, Time=%u, Result=%u, HP=%u, Heat=%u",
            gimbal_data_msg.game_progress, gimbal_data_msg.stage_remain_time,
            gimbal_data_msg.game_result, gimbal_data_msg.current_hp, 
            gimbal_data_msg.shooter_42mm_heat);
    }

    rclcpp::Publisher<sentry_interfaces::msg::GimbalData>::SharedPtr gimbal_data_pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RefereeSerialBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
