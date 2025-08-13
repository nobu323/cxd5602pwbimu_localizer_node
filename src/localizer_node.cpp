#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <libserial/SerialPort.h>

#include <string>
#include <vector>
#include <sstream>
#include <memory>
#include <stdexcept>
#include <iostream>
#include <cstdint>

// Data structure for binary communication
// This must match the struct in the Spresense code
struct IMUData {
  uint32_t timestamp;
  float temp;
  float angular_velocity_x;
  float angular_velocity_y;
  float angular_velocity_z;
  float linear_acceleration_x;
  float linear_acceleration_y;
  float linear_acceleration_z;
  float raw_linear_acceleration_x;
  float raw_linear_acceleration_y;
  float raw_linear_acceleration_z;
  float quat_w;
  float quat_x;
  float quat_y;
  float quat_z;
  float velocity_x;
  float velocity_y;
  float velocity_z;
  float position_x;
  float position_y;
  float position_z;
};

const uint8_t HEADER[] = {0x5A, 0xA5};

// Helper function to calculate checksum
uint8_t calculate_checksum(const uint8_t* data, size_t len) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}


// Helper function to convert integer baud rate to LibSerial BaudRate enum
LibSerial::BaudRate get_libserial_baudrate(int baudrate) {
    switch (baudrate) {
        case 110: return LibSerial::BaudRate::BAUD_110;
        case 300: return LibSerial::BaudRate::BAUD_300;
        case 600: return LibSerial::BaudRate::BAUD_600;
        case 1200: return LibSerial::BaudRate::BAUD_1200;
        case 2400: return LibSerial::BaudRate::BAUD_2400;
        case 4800: return LibSerial::BaudRate::BAUD_4800;
        case 9600: return LibSerial::BaudRate::BAUD_9600;
        case 19200: return LibSerial::BaudRate::BAUD_19200;
        case 38400: return LibSerial::BaudRate::BAUD_38400;
        case 57600: return LibSerial::BaudRate::BAUD_57600;
        case 115200: return LibSerial::BaudRate::BAUD_115200;
        case 230400: return LibSerial::BaudRate::BAUD_230400;
        case 460800: return LibSerial::BaudRate::BAUD_460800;
        case 500000: return LibSerial::BaudRate::BAUD_500000;
        case 576000: return LibSerial::BaudRate::BAUD_576000;
        case 921600: return LibSerial::BaudRate::BAUD_921600;
        case 1000000: return LibSerial::BaudRate::BAUD_1000000;
        case 1152000: return LibSerial::BaudRate::BAUD_1152000;
        case 1500000: return LibSerial::BaudRate::BAUD_1500000;
        case 2000000: return LibSerial::BaudRate::BAUD_2000000;
        default: throw std::invalid_argument("Invalid baud rate");
    }
}

class LocalizerNode : public rclcpp::Node {
public:
    LocalizerNode() : Node("cxd5602pwbimu_localizer_node") {
        // Declare and get parameters
        auto serial_port_param = this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        auto baud_rate_param = this->declare_parameter<int>("baud_rate", 1152000);
        tf_parent_frame_ = this->declare_parameter<std::string>("tf_parent_frame", "world");
        tf_child_frame_ = this->declare_parameter<std::string>("tf_child_frame", "sensor");

        RCLCPP_INFO(this->get_logger(), "Using serial port: %s at %ld baud", serial_port_param.c_str(), baud_rate_param);
        RCLCPP_INFO(this->get_logger(), "TF frames: parent='%s', child='%s'", tf_parent_frame_.c_str(), tf_child_frame_.c_str());

        // Create publishers
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        imu_raw_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("pose", 10);

        // Create TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        try {
            serial_port_.Open(serial_port_param);
            serial_port_.SetDTR(true);
            serial_port_.SetBaudRate(get_libserial_baudrate(baud_rate_param));
            serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
            serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
            serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
            serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        } catch (const LibSerial::OpenFailed& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
            rclcpp::shutdown();
            return;
        } catch (const std::invalid_argument& e) {
            RCLCPP_ERROR(this->get_logger(), "Invalid baud rate specified: %ld", baud_rate_param);
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Serial port opened successfully. Waiting for data...");

        // Create timer for reading data
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&LocalizerNode::read_serial_data, this));
    }

    ~LocalizerNode() {
        if (serial_port_.IsOpen()) {
            serial_port_.Close();
            RCLCPP_INFO(this->get_logger(), "Serial port closed.");
        }
    }

private:
    enum class ReadState {
        WAITING_FOR_HEADER_1,
        WAITING_FOR_HEADER_2,
        READING_DATA
    };

    void read_serial_data() {
        if (!serial_port_.IsOpen()) {
            return;
        }

        std::vector<uint8_t> byte_buffer;
        try {
            serial_port_.Read(byte_buffer, 256, 5);
        } catch (const LibSerial::ReadTimeout&) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No data from serial port (timeout). Is device connected and sending data?");
            return;
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error reading from serial port: %s", e.what());
            return;
        }

        if (!byte_buffer.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "Read %zu bytes from serial port.", byte_buffer.size());
            for (uint8_t byte : byte_buffer) {
                process_byte(byte);
            }
        }
    }

    void process_byte(uint8_t byte) {
        switch (read_state_) {
            case ReadState::WAITING_FOR_HEADER_1:
                if (byte == HEADER[0]) {
                    read_state_ = ReadState::WAITING_FOR_HEADER_2;
                }
                break;

            case ReadState::WAITING_FOR_HEADER_2:
                if (byte == HEADER[1]) {
                    read_state_ = ReadState::READING_DATA;
                    data_buffer_.clear();
                } else {
                    read_state_ = ReadState::WAITING_FOR_HEADER_1;
                }
                break;

            case ReadState::READING_DATA:
                data_buffer_.push_back(byte);
                if (data_buffer_.size() == sizeof(IMUData) + 1) { // +1 for checksum
                    IMUData received_data;
                    uint8_t received_checksum = data_buffer_.back();
                    data_buffer_.pop_back();
                    
                    std::copy(data_buffer_.begin(), data_buffer_.end(), reinterpret_cast<uint8_t*>(&received_data));

                    uint8_t calculated_checksum = calculate_checksum(reinterpret_cast<const uint8_t*>(&received_data), sizeof(IMUData));

                    if (calculated_checksum == received_checksum) {
                        process_imu_data(received_data);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Checksum mismatch! Calculated: 0x%02X, Received: 0x%02X. Discarding packet.", calculated_checksum, received_checksum);
                    }
                    
                    read_state_ = ReadState::WAITING_FOR_HEADER_1;
                }
                break;
        }
    }

    void process_imu_data(const IMUData& data) {
        auto now = this->get_clock()->now();

        // Create and publish IMU message
        auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
        imu_msg->header.stamp = now;
        imu_msg->header.frame_id = tf_child_frame_;
        imu_msg->angular_velocity.x = data.angular_velocity_x;
        imu_msg->angular_velocity.y = data.angular_velocity_y;
        imu_msg->angular_velocity.z = data.angular_velocity_z;
        imu_msg->linear_acceleration.x = data.linear_acceleration_x;
        imu_msg->linear_acceleration.y = data.linear_acceleration_y;
        imu_msg->linear_acceleration.z = data.linear_acceleration_z;
        imu_msg->orientation.w = data.quat_w;
        imu_msg->orientation.x = data.quat_x;
        imu_msg->orientation.y = data.quat_y;
        imu_msg->orientation.z = data.quat_z;
        imu_msg->orientation_covariance[0] = -1;
        imu_msg->angular_velocity_covariance[0] = -1;
        imu_msg->linear_acceleration_covariance[0] = -1;
        imu_pub_->publish(std::move(imu_msg));

        // Create and publish raw IMU message (uncompensated)
        auto imu_raw_msg = std::make_unique<sensor_msgs::msg::Imu>();
        imu_raw_msg->header.stamp = now;
        imu_raw_msg->header.frame_id = tf_child_frame_;
        imu_raw_msg->angular_velocity.x = data.angular_velocity_x;
        imu_raw_msg->angular_velocity.y = data.angular_velocity_y;
        imu_raw_msg->angular_velocity.z = data.angular_velocity_z;
        imu_raw_msg->linear_acceleration.x = data.raw_linear_acceleration_x;
        imu_raw_msg->linear_acceleration.y = data.raw_linear_acceleration_y;
        imu_raw_msg->linear_acceleration.z = data.raw_linear_acceleration_z;
        imu_raw_msg->orientation.w = data.quat_w;
        imu_raw_msg->orientation.x = data.quat_x;
        imu_raw_msg->orientation.y = data.quat_y;
        imu_raw_msg->orientation.z = data.quat_z;
        imu_raw_msg->orientation_covariance[0] = -1;
        imu_raw_msg->angular_velocity_covariance[0] = -1;
        imu_raw_msg->linear_acceleration_covariance[0] = -1;
        imu_raw_pub_->publish(std::move(imu_raw_msg));

        // Create and publish Pose message
        auto pose_msg = std::make_unique<geometry_msgs::msg::Pose>();
        pose_msg->position.x = data.position_x;
        pose_msg->position.y = data.position_y;
        pose_msg->position.z = data.position_z;
        pose_msg->orientation.w = data.quat_w;
        pose_msg->orientation.x = data.quat_x;
        pose_msg->orientation.y = data.quat_y;
        pose_msg->orientation.z = data.quat_z;
        pose_pub_->publish(std::move(pose_msg));

        // Create and broadcast Transform
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = tf_parent_frame_;
        t.child_frame_id = tf_child_frame_;
        t.transform.translation.x = data.position_x;
        t.transform.translation.y = data.position_y;
        t.transform.translation.z = data.position_z;
        t.transform.rotation.w = data.quat_w;
        t.transform.rotation.x = data.quat_x;
        t.transform.rotation.y = data.quat_y;
        t.transform.rotation.z = data.quat_z;
        tf_broadcaster_->sendTransform(t);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    std::string tf_parent_frame_;
    std::string tf_child_frame_;

    LibSerial::SerialPort serial_port_;
    ReadState read_state_ = ReadState::WAITING_FOR_HEADER_1;
    std::vector<uint8_t> data_buffer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalizerNode>();
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
