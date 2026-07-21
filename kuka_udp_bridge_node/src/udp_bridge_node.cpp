#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <sstream>
#include <vector>
#include <iostream>

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

using namespace std::chrono_literals;

class KukaUdpBridge : public rclcpp::Node {
public:
    KukaUdpBridge() : Node("kuka_udp_bridge"), tx_counter_(0) {
        // 1. Declare ROS2 Parameters (Allows changing IP without recompiling!)
        this->declare_parameter<std::string>("robot_ip", "172.31.1.147");
        this->declare_parameter<int>("robot_port", 30300);
        this->declare_parameter<int>("client_port", 30333);

        robot_ip_ = this->get_parameter("robot_ip").as_string();
        robot_port_ = this->get_parameter("robot_port").as_int();
        client_port_ = this->get_parameter("client_port").as_int();

        RCLCPP_INFO(this->get_logger(), "Configured to target Robot at %s:%d", robot_ip_.c_str(), robot_port_);
        RCLCPP_INFO(this->get_logger(), "Listening locally for telemetry on port %d", client_port_);

        // 2. Setup UDP Sockets
        setup_sockets();

        // 3. ROS2 Publisher & Subscriber
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&KukaUdpBridge::goal_pose_callback, this, std::placeholders::_1));
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&KukaUdpBridge::cmd_vel_callback, this, std::placeholders::_1));

        // TF Broadcaster for robot visualization in RViz
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 4. Start the Background UDP Receive Thread
        rx_thread_active_ = true;
        rx_thread_ = std::thread(&KukaUdpBridge::receive_thread_loop, this);

        // 5. Send Initial Handshake & Activation Packet to Robot
        // This registers our dynamic IP/Port with the robot and starts the loop!
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
        // Create UDP Socket
        sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_fd_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to create socket!");
            throw std::runtime_error("Socket creation failed");
        }

        // Bind locally to client_port (30333) so we receive incoming telemetry packets
        struct sockaddr_in local_addr;
        memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = INADDR_ANY; // Listen on any interface
        local_addr.sin_port = htons(client_port_);

        if (bind(sock_fd_, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to bind socket to local port %d!", client_port_);
            throw std::runtime_error("Socket bind failed");
        }

        // Configure Destination Address (The Robot)
        memset(&robot_addr_, 0, sizeof(robot_addr_));
        robot_addr_.sin_family = AF_INET;
        robot_addr_.sin_port = htons(robot_port_);
        inet_pton(AF_INET, robot_ip_.c_str(), &robot_addr_.sin_addr);
    }

    void send_to_robot(const std::string& command, const std::string& value) {
        tx_counter_++;
        auto now = std::chrono::system_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

        // Format packet: Timestamp;Counter;Command;Value
        std::stringstream ss;
        ss << ms << ";" << tx_counter_ << ";" << command << ";" << value;
        std::string payload = ss.str();

        sendto(sock_fd_, payload.c_str(), payload.length(), 0,
               (struct sockaddr *)&robot_addr_, sizeof(robot_addr_));
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Parse Twist to vx, vy, omega (Translating m/s to KUKA's expected m/s and rad/s)
        std::stringstream ss;
        ss << msg->linear.x << "," << msg->linear.y << "," << msg->angular.z;
        
        // Dispatch over UDP
        send_to_robot("Set_Vel", ss.str());
    }

    void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // Convert Pose to KUKA's expected format: X,Y,Alpha (Alpha in degrees)
        double x_mm = msg->pose.position.x * 1000.0; // Convert meters to millimeters
        double y_mm = msg->pose.position.y * 1000.0;

        // Extract yaw from quaternion
        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        double alpha_deg = yaw * (180.0 / M_PI); // Convert radians to degrees

        std::stringstream ss;
        ss << x_mm << "," << y_mm << "," << alpha_deg;

        send_to_robot("Set_Pose", ss.str());
    }

    void receive_thread_loop() {
        char rx_buf[1024];
        struct sockaddr_in sender_addr;
        socklen_t sender_len = sizeof(sender_addr);

        // Set socket to non-blocking so the thread can shut down cleanly if we close the node
        fcntl(sock_fd_, F_SETFL, O_NONBLOCK);

        RCLCPP_INFO(this->get_logger(), "UDP Receive Thread spawned. Listening for status messages...");
        
        while (rx_thread_active_ && rclcpp::ok()) {
            memset(rx_buf, 0, sizeof(rx_buf));
            int bytes_received = recvfrom(sock_fd_, rx_buf, sizeof(rx_buf) - 1, 0,
                                          (struct sockaddr *)&sender_addr, &sender_len);

            if (bytes_received > 0) {
                rx_buf[bytes_received] = '\0';
                std::string msg(rx_buf);
                // --- Continuous Telemetry Stream Logging ---
            // Logs once per second (1000ms) without slowing down packet processing
                RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000, 
                "[KUKA RX Stream] Raw telemetry: %s",
                msg.c_str()
                );
                parse_and_publish_telemetry(msg);
            } else {
                // Sleep slightly to prevent high CPU load on empty non-blocking loops
                std::this_thread::sleep_for(2ms);
            }
        }
    }

    void parse_and_publish_telemetry(const std::string& msg) {
        // Incoming Format: Timestamp;ErrorCode;Counter;X,Y,Alpha
        std::vector<std::string> parts;
        std::stringstream ss(msg);
        std::string item;
        while (std::getline(ss, item, ';')) {
            parts.push_back(item);
        }

        if (parts.size() < 4) return;

        try {
            // Parse coordinate payload (X, Y, Alpha)
            std::vector<double> pose;
            std::stringstream pose_ss(parts[3]);
            std::string val;
            while (std::getline(pose_ss, val, ',')) {
                pose.push_back(std::stod(val));
            }

            if (pose.size() < 3) return;

            // KUKA coordinates are usually tracked in millimeters (from your math accumulator)
            // Convert to meters for standard ROS2 conventions
            double x_m = pose[0] / 1000.0;
            double y_m = pose[1] / 1000.0;
            double yaw_rad = MathToRadians(pose[2]); // Convert alpha degrees to rads

            // Publish Odometry Message
            auto odom_msg = nav_msgs::msg::Odometry();
            odom_msg.header.stamp = this->now();
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_footprint";

            // Pose Position
            odom_msg.pose.pose.position.x = x_m;
            odom_msg.pose.pose.position.y = y_m;
            odom_msg.pose.pose.position.z = 0.0;

            // Pose Orientation (Euler Yaw -> Quaternion)
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw_rad);
            odom_msg.pose.pose.orientation.x = q.x();
            odom_msg.pose.pose.orientation.y = q.y();
            odom_msg.pose.pose.orientation.z = q.z();
            odom_msg.pose.pose.orientation.w = q.w();

            odom_pub_->publish(odom_msg);

            // Broadcast TF Transform (Required for RViz mapping and visualization)
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
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;

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