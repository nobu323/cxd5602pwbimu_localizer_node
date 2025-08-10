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

// Helper function to convert hex string to float
float hex_to_float(const std::string& hex_str) {
    uint32_t int_val;
    std::stringstream ss;
    ss << std::hex << hex_str;
    ss >> int_val;
    return *reinterpret_cast<float*>(&int_val);
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
        auto baud_rate_param = this->declare_parameter<int>("baud_rate", 115200);
        tf_parent_frame_ = this->declare_parameter<std::string>("tf_parent_frame", "world");
        tf_child_frame_ = this->declare_parameter<std::string>("tf_child_frame", "sensor");

        RCLCPP_INFO(this->get_logger(), "Using serial port: %s at %ld baud", serial_port_param.c_str(), baud_rate_param);
        RCLCPP_INFO(this->get_logger(), "TF frames: parent='%s', child='%s'", tf_parent_frame_.c_str(), tf_child_frame_.c_str());

        // Create publishers
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("pose", 10);

        // Create TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        try {
            serial_port_.Open(serial_port_param);
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

        RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");

        // Create timer for reading data
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2),
            std::bind(&LocalizerNode::read_serial_data, this));
    }

    ~LocalizerNode() {
        if (serial_port_.IsOpen()) {
            serial_port_.Close();
            RCLCPP_INFO(this->get_logger(), "Serial port closed.");
        }
    }

private:
    void read_serial_data() {
        if (!serial_port_.IsOpen()) {
            return;
        }

        std::string line;
        try {
            // Read until the newline character, with a timeout of 10ms
            serial_port_.ReadLine(line, '\n', 10);
        } catch (const LibSerial::ReadTimeout&) {
            // This is expected if no data is available
            return;
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error reading from serial port: %s", e.what());
            return;
        }

        if (line.empty()) {
            return;
        }

        std::stringstream ss(line);
        std::string part;
        std::vector<std::string> parts;
        while(std::getline(ss, part, ',')) {
            if (!part.empty()) {
                parts.push_back(part);
            }
        }

        if (parts.size() != 18) {
            RCLCPP_WARN(this->get_logger(), "Unexpected data length: %zu, content: %s", parts.size(), line.c_str());
            return;
        }

        try {
            // Angular velocity and linear acceleration
            auto angular_velocity_x = hex_to_float(parts[2]);
            auto angular_velocity_y = hex_to_float(parts[3]);
            auto angular_velocity_z = hex_to_float(parts[4]);
            auto linear_acceleration_x = hex_to_float(parts[5]);
            auto linear_acceleration_y = hex_to_float(parts[6]);
            auto linear_acceleration_z = hex_to_float(parts[7]);

            // Quaternion
            auto quat_w = hex_to_float(parts[8]);
            auto quat_x = hex_to_float(parts[9]);
            auto quat_y = hex_to_float(parts[10]);
            auto quat_z = hex_to_float(parts[11]);

            // Position
            auto pos_x = hex_to_float(parts[15]);
            auto pos_y = hex_to_float(parts[16]);
            auto pos_z = hex_to_float(parts[17]);

            auto now = this->get_clock()->now();

            // Create and publish IMU message
            auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
            imu_msg->header.stamp = now;
            imu_msg->header.frame_id = "imu_link";
            imu_msg->angular_velocity.x = angular_velocity_x;
            imu_msg->angular_velocity.y = angular_velocity_y;
            imu_msg->angular_velocity.z = angular_velocity_z;
            imu_msg->linear_acceleration.x = linear_acceleration_x;
            imu_msg->linear_acceleration.y = linear_acceleration_y;
            imu_msg->linear_acceleration.z = linear_acceleration_z;
            imu_msg->orientation.w = quat_w;
            imu_msg->orientation.x = quat_x;
            imu_msg->orientation.y = quat_y;
            imu_msg->orientation.z = quat_z;
            imu_msg->orientation_covariance[0] = -1;
            imu_msg->angular_velocity_covariance[0] = -1;
            imu_msg->linear_acceleration_covariance[0] = -1;
            imu_pub_->publish(std::move(imu_msg));

            // Create and publish Pose message
            auto pose_msg = std::make_unique<geometry_msgs::msg::Pose>();
            pose_msg->position.x = pos_x;
            pose_msg->position.y = pos_y;
            pose_msg->position.z = pos_z;
            pose_msg->orientation.w = quat_w;
            pose_msg->orientation.x = quat_x;
            pose_msg->orientation.y = quat_y;
            pose_msg->orientation.z = quat_z;
            pose_pub_->publish(std::move(pose_msg));

            // Create and broadcast Transform
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now;
            t.header.frame_id = tf_parent_frame_;
            t.child_frame_id = tf_child_frame_;
            t.transform.translation.x = pos_x;
            t.transform.translation.y = pos_y;
            t.transform.translation.z = pos_z;
            t.transform.rotation.w = quat_w;
            t.transform.rotation.x = quat_x;
            t.transform.rotation.y = quat_y;
            t.transform.rotation.z = quat_z;
            tf_broadcaster_->sendTransform(t);

        } catch (const std::invalid_argument& e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing hex string: %s", e.what());
        } catch (const std::out_of_range& e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing hex string (out of range): %s", e.what());
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    std::string tf_parent_frame_;
    std::string tf_child_frame_;

    LibSerial::SerialPort serial_port_;
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
