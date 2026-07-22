#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <sstream>
#include <vector>
#include <iostream>
#include <cmath>

// Linux Socket Headers
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

// ROS2 Headers
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "sensor_msgs/msg/joint_state.hpp" // Added for LBR Arm Control

using namespace std::chrono_literals;

class KukaUdpBridge : public rclcpp::Node {
public:
    KukaUdpBridge() : Node("kuka_udp_bridge"), tx_counter_(0) {
        // 1. Declare ROS2 Parameters
        this->declare_parameter<std::string>("robot_ip", "172.31.1.10");
        this->declare_parameter<int>("robot_port", 30300);
        this->declare_parameter<int>("client_port", 30333);

        robot_ip_ = this->get_parameter("robot_ip").as_string();
        robot_port_ = this->get_parameter("robot_port").as_int();
        client_port_ = this->get_parameter("client_port").as_int();

        RCLCPP_INFO(this->get_logger(), "Configured to target Robot at %s:%d", robot_ip_.c_str(), robot_port_);
        RCLCPP_INFO(this->get_logger(), "Listening locally for telemetry on port %d", client_port_);

        // 2. Setup UDP Sockets
        setup_sockets();

        // 3. ROS2 Publisher & Subscribers
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&KukaUdpBridge::goal_pose_callback, this, std::placeholders::_1));
            
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&KukaUdpBridge::cmd_vel_callback, this, std::placeholders::_1));

        // NEW: Subscribe to Joint State messages for 7-DOF LBR Arm Control
        arm_cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/arm_cmd", 10, std::bind(&KukaUdpBridge::arm_cmd_callback, this, std::placeholders::_1));

        // TF Broadcaster for robot visualization in RViz
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 4. Start Background UDP Receive Thread
        rx_thread_active_ = true;
        rx_thread_ = std::thread(&KukaUdpBridge::receive_thread_loop, this);

        // 5. Send Initial Handshake & Activation Packet
        send_to_robot("App_Start", "true");
    }

    ~KukaUdpBridge() {
        // Cleanup
        send_to_robot("Set_Shutdown", "true");
        rx_thread_active_ = false;
        if (rx_thread_.joinable()) {
            rx_thread_.join();
        }
        if (sock_fd_ >= 0) {
            close(sock_fd_);
        }
    }

private:
    void setup_sockets() {
        sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_fd_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to create socket!");
            throw std::runtime_error("Socket creation failed");
        }

        struct sockaddr_in local_addr;
        memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = INADDR_ANY;
        local_addr.sin_port = htons(client_port_);

        if (bind(sock_fd_, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to bind socket to local port %d!", client_port_);
            throw std::runtime_error("Socket bind failed");
        }

        memset(&robot_addr_, 0, sizeof(robot_addr_));
        robot_addr_.sin_family = AF_INET;
        robot_addr_.sin_port = htons(robot_port_);
        inet_pton(AF_INET, robot_ip_.c_str(), &robot_addr_.sin_addr);
    }

    void send_to_robot(const std::string& command, const std::string& value) {
        tx_counter_++;
        auto now = std::chrono::system_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

        std::stringstream ss;
        ss << ms << ";" << tx_counter_ << ";" << command << ";" << value;
        std::string payload = ss.str();

        sendto(sock_fd_, payload.c_str(), payload.length(), 0,
               (struct sockaddr *)&robot_addr_, sizeof(robot_addr_));
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::stringstream ss;
        ss << msg->linear.x << "," << msg->linear.y << "," << msg->angular.z;
        send_to_robot("Set_Vel", ss.str());
    }

    void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        double x_mm = msg->pose.position.x * 1000.0;
        double y_mm = msg->pose.position.y * 1000.0;

        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        double alpha_deg = yaw * (180.0 / M_PI);

        std::stringstream ss;
        ss << x_mm << "," << y_mm << "," << alpha_deg;

        send_to_robot("Set_Pose", ss.str());
    }

    // NEW: Callback to convert incoming ROS2 JointStates (radians) to KUKA degrees payload
    void arm_cmd_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->position.size() < 7) {
            RCLCPP_WARN(this->get_logger(), "Received JointState with %size positions. Required: 7", msg->position.size());
            return;
        }

        std::stringstream ss;
        for (size_t i = 0; i < 7; ++i) {
            // Convert ROS2 radians to KUKA Java degrees
            double deg = msg->position[i] * (180.0 / M_PI);
            ss << deg;
            if (i < 6) ss << ",";
        }

        send_to_robot("Set_Arm", ss.str());
    }

    void receive_thread_loop() {
        char rx_buf[1024];
        struct sockaddr_in sender_addr;
        socklen_t sender_len = sizeof(sender_addr);

        fcntl(sock_fd_, F_SETFL, O_NONBLOCK);

        RCLCPP_INFO(this->get_logger(), "UDP Receive Thread spawned. Listening for status messages...");

        while (rx_thread_active_ && rclcpp::ok()) {
            memset(rx_buf, 0, sizeof(rx_buf));
            int bytes_received = recvfrom(sock_fd_, rx_buf, sizeof(rx_buf) - 1, 0,
                                          (struct sockaddr *)&sender_addr, &sender_len);

            if (bytes_received > 0) {
                rx_buf[bytes_received] = '\0';
                std::string msg(rx_buf);
                parse_and_publish_telemetry(msg);
            } else {
                std::this_thread::sleep_for(2ms);
            }
        }
    }

    void parse_and_publish_telemetry(const std::string& msg) {
        std::vector<std::string> parts;
        std::stringstream ss(msg);
        std::string item;
        while (std::getline(ss, item, ';')) {
            parts.push_back(item);
        }

        if (parts.size() < 4) return;

        try {
            std::vector<double> pose;
            std::stringstream pose_ss(parts[3]);
            std::string val;
            while (std::getline(pose_ss, val, ',')) {
                pose.push_back(std::stod(val));
            }

            if (pose.size() < 3) return;

            double x_m = pose[0] / 1000.0;
            double y_m = pose[1] / 1000.0;
            double yaw_rad = MathToRadians(pose[2]);

            auto odom_msg = nav_msgs::msg::Odometry();
            odom_msg.header.stamp = this->now();
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_footprint";

            odom_msg.pose.pose.position.x = x_m;
            odom_msg.pose.pose.position.y = y_m;
            odom_msg.pose.pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, yaw_rad);
            odom_msg.pose.pose.orientation.x = q.x();
            odom_msg.pose.pose.orientation.y = q.y();
            odom_msg.pose.pose.orientation.z = q.z();
            odom_msg.pose.pose.orientation.w = q.w();

            odom_pub_->publish(odom_msg);

            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header = odom_msg.header;
            tf_msg.child_frame_id = odom_msg.child_frame_id;
            tf_msg.transform.translation.x = x_m;
            tf_msg.transform.translation.y = y_m;
            tf_msg.transform.translation.z = 0.0;
            tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

            tf_broadcaster_->sendTransform(tf_msg);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse telemetry: %s", e.what());
        }
    }

    double MathToRadians(double degrees) {
        return degrees * (M_PI / 180.0);
    }

    // Networking
    int sock_fd_ = -1;
    struct sockaddr_in robot_addr_;
    std::string robot_ip_;
    int robot_port_;
    int client_port_;
    long tx_counter_;

    // ROS2 Elements
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_cmd_sub_; // NEW
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Multithreading
    std::thread rx_thread_;
    std::atomic<bool> rx_thread_active_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KukaUdpBridge>());
    rclcpp::shutdown();
    return 0;
}