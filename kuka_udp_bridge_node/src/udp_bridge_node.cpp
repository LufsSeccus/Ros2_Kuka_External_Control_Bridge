#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <sstream>
#include <vector>
#include <iostream>
#include <fstream>   // Added for file I/O
#include <iomanip>   // Added for time formatting
#include <algorithm> // Added for std::replace

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
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"

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

        RCLCPP_INFO(this->get_logger(), "[KUKA UDP Bridge] Target Robot: %s:%d", robot_ip_.c_str(), robot_port_);
        RCLCPP_INFO(this->get_logger(), "[KUKA UDP Bridge] Local Listening Port: %d", client_port_);

        // 2. Setup UDP Sockets
        setup_sockets();

        // 3. ROS2 Publishers
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        // 4. ROS2 Subscriptions
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&KukaUdpBridge::cmd_vel_callback, this, std::placeholders::_1));

        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&KukaUdpBridge::goal_pose_callback, this, std::placeholders::_1));

        arm_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/arm_cmd_joints", 10, std::bind(&KukaUdpBridge::arm_joint_callback, this, std::placeholders::_1));

        arm_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/arm_goal_pose", 10, std::bind(&KukaUdpBridge::arm_pose_callback, this, std::placeholders::_1));

        // TF Broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 5. Start Background UDP RX Thread
        rx_thread_active_ = true;
        rx_thread_ = std::thread(&KukaUdpBridge::receive_thread_loop, this);

        // 6. Send Initial Handshake Packet
        send_to_robot("App_Start", "true");
        RCLCPP_INFO(this->get_logger(), "[KUKA UDP Bridge] Sent App_Start handshake to robot.");
    }

    ~KukaUdpBridge() {
        RCLCPP_WARN(this->get_logger(), "[KUKA UDP Bridge] Shutting down bridge node...");
        send_to_robot("Set_Shutdown", "true");
        rx_thread_active_ = false;

        if (rx_thread_.joinable()) {
            rx_thread_.join();
        }
        if (sock_fd_ >= 0) {
            close(sock_fd_);
        }
        // Safely close the logger file on shutdown
        if (telemetry_log_file_.is_open()) {
            telemetry_log_file_.close();
            RCLCPP_INFO(this->get_logger(), "[KUKA UDP Bridge] Telemetry log file saved and closed.");
        }
    }

private:
    void setup_sockets() {
        sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_fd_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to create UDP socket!");
            throw std::runtime_error("Socket creation failed");
        }

        struct sockaddr_in local_addr;
        memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = INADDR_ANY;
        local_addr.sin_port = htons(client_port_);

        if (bind(sock_fd_, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to bind UDP socket to port %d!", client_port_);
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

    // --- Command Callbacks ---

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::stringstream ss;
        ss << msg->linear.x << "," << msg->linear.y << "," << msg->angular.z;
        send_to_robot("Set_Vel", ss.str());

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 500,
            "[TX KMP Vel] vx: %.2f m/s | vy: %.2f m/s | omega: %.2f rad/s",
            msg->linear.x, msg->linear.y, msg->angular.z);
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

        RCLCPP_INFO(
            this->get_logger(),
            "[TX KMP Pose] Target -> X: %.1f mm | Y: %.1f mm | Alpha: %.2f deg",
            x_mm, y_mm, alpha_deg);
    }

    void arm_joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->position.size() < 7) {
            RCLCPP_WARN(this->get_logger(), "[TX Arm Joint] Expected 7 joint positions, got %ld", msg->position.size());
            return;
        }

        std::stringstream ss;
        for (size_t i = 0; i < 7; ++i) {
            double deg = msg->position[i] * (180.0 / M_PI); // rad to deg
            ss << deg;
            if (i < 6) ss << ",";
        }

        send_to_robot("Set_Arm_Joint", ss.str());

        RCLCPP_INFO(
            this->get_logger(),
            "[TX Arm Joint] J1: %.1f° J2: %.1f° J3: %.1f° J4: %.1f° J5: %.1f° J6: %.1f° J7: %.1f°",
            msg->position[0] * (180.0 / M_PI), msg->position[1] * (180.0 / M_PI),
            msg->position[2] * (180.0 / M_PI), msg->position[3] * (180.0 / M_PI),
            msg->position[4] * (180.0 / M_PI), msg->position[5] * (180.0 / M_PI),
            msg->position[6] * (180.0 / M_PI));
    }

    void arm_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        double x_m = msg->pose.position.x;
        double y_m = msg->pose.position.y;
        double z_m = msg->pose.position.z;

        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        double roll_rad, pitch_rad, yaw_rad;
        tf2::Matrix3x3(q).getRPY(roll_rad, pitch_rad, yaw_rad);

        std::stringstream ss;
        ss << x_m << "," << y_m << "," << z_m << ","
           << yaw_rad << "," << pitch_rad << "," << roll_rad;

        send_to_robot("Set_Arm_End_Effector", ss.str());

        RCLCPP_INFO(
            this->get_logger(),
            "[TX Arm EE] Target -> Pos(m): [%.3f, %.3f, %.3f] | RPY(deg): [%.1f, %.1f, %.1f]",
            x_m, y_m, z_m,
            roll_rad * (180.0 / M_PI), pitch_rad * (180.0 / M_PI), yaw_rad * (180.0 / M_PI));
    }

    std::string get_log_filename() {
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::tm parts;
        localtime_r(&now_c, &parts); // Safe parsing for Linux environments

        std::ostringstream oss;
        // Format: logger_bridge_file_HH-MM_DD-MM-YYYY.csv
        oss << "logger_bridge_file_" 
            << std::put_time(&parts, "%H-%M_%d-%m-%Y") 
            << ".csv";
        return oss.str();
    }

    // --- Telemetry RX Processing Loop ---

    void receive_thread_loop() {
        char rx_buf[1024];
        struct sockaddr_in sender_addr;
        socklen_t sender_len = sizeof(sender_addr);

        fcntl(sock_fd_, F_SETFL, O_NONBLOCK);
        RCLCPP_INFO(this->get_logger(), "[KUKA UDP RX] Thread spawned. Listening for status payloads...");

        while (rx_thread_active_ && rclcpp::ok()) {
            memset(rx_buf, 0, sizeof(rx_buf));
            int bytes_received = recvfrom(sock_fd_, rx_buf, sizeof(rx_buf) - 1, 0,
                                          (struct sockaddr *)&sender_addr, &sender_len);

            if (bytes_received > 0) {
                rx_buf[bytes_received] = '\0';
                std::string msg(rx_buf);
                // --- 1. INITIALIZE LOGGER FILE ON FIRST MESSAGE ---
                if (!is_logging_started_) {
                    std::string filename = get_log_filename();
                    telemetry_log_file_.open(filename, std::ios::out | std::ios::app);
                    
                    if (telemetry_log_file_.is_open()) {
                        // Write the CSV Table Header
                        telemetry_log_file_ << "Timestamp,ErrorCode,Counter,"
                                            << "KMP_X,KMP_Y,KMP_Alpha,"
                                            << "Arm_J1,Arm_J2,Arm_J3,Arm_J4,Arm_J5,Arm_J6,Arm_J7\n";
                        
                        is_logging_started_ = true;
                        RCLCPP_INFO(this->get_logger(), "[KUKA UDP Bridge] Started telemetry logging: %s", filename.c_str());
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "[KUKA UDP Bridge] Failed to create telemetry log file!");
                    }
                }

                // --- 2. APPEND TELEMETRY TO LOGGER ---
                if (is_logging_started_ && telemetry_log_file_.is_open()) {
                    std::string csv_line = msg;
                    // Replace the Java ';' delimiters with ',' for the CSV table
                    std::replace(csv_line.begin(), csv_line.end(), ';', ',');
                    
                    telemetry_log_file_ << csv_line << "\n";
                    telemetry_log_file_.flush(); // Ensure it writes to disk immediately
                }

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "[KUKA Telemetry RX] %s", msg.c_str());

                parse_and_publish_telemetry(msg);
            } else {
                std::this_thread::sleep_for(2ms);
            }
        }
    }

    void parse_and_publish_telemetry(const std::string& msg) {
        // Expected Format: Timestamp;ErrorCode;Counter;BasePose(X,Y,Alpha);ArmPose(J1..J7)
        std::vector<std::string> parts;
        std::stringstream ss(msg);
        std::string item;
        while (std::getline(ss, item, ';')) {
            parts.push_back(item);
        }

        if (parts.size() < 4) return;

        try {
            // 1. Base Pose Processing (X, Y, Alpha in mm and degrees)
            std::vector<double> base_pose;
            std::stringstream base_ss(parts[3]);
            std::string val;
            while (std::getline(base_ss, val, ',')) {
                base_pose.push_back(std::stod(val));
            }

            if (base_pose.size() >= 3) {
                double x_m = base_pose[0] / 1000.0;
                double y_m = base_pose[1] / 1000.0;
                double yaw_rad = base_pose[2] * (M_PI / 180.0);

                // Publish Odometry
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

                // Broadcast TF
                geometry_msgs::msg::TransformStamped tf_msg;
                tf_msg.header = odom_msg.header;
                tf_msg.child_frame_id = odom_msg.child_frame_id;
                tf_msg.transform.translation.x = x_m;
                tf_msg.transform.translation.y = y_m;
                tf_msg.transform.translation.z = 0.0;
                tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

                tf_broadcaster_->sendTransform(tf_msg);
            }

            // 2. LBR Joint State Processing (If Arm telemetry present)
            if (parts.size() >= 5) {
                std::vector<double> arm_joints_deg;
                std::stringstream arm_ss(parts[4]);
                while (std::getline(arm_ss, val, ',')) {
                    arm_joints_deg.push_back(std::stod(val));
                }

                if (arm_joints_deg.size() >= 7) {
                    auto joint_msg = sensor_msgs::msg::JointState();
                    joint_msg.header.stamp = this->now();
                    joint_msg.name = {
                        "lbr_joint_1", "lbr_joint_2", "lbr_joint_3",
                        "lbr_joint_4", "lbr_joint_5", "lbr_joint_6", "lbr_joint_7"
                    };

                    for (int i = 0; i < 7; ++i) {
                        joint_msg.position.push_back(arm_joints_deg[i] * (M_PI / 180.0)); // deg to rad
                    }

                    joint_pub_->publish(joint_msg);
                }
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "[KUKA Telemetry Parse Error] %s", e.what());
        }
    }

    // Networking Data
    int sock_fd_ = -1;
    struct sockaddr_in robot_addr_;
    std::string robot_ip_;
    int robot_port_;
    int client_port_;
    long tx_counter_;

    // File Logging Data
    std::ofstream telemetry_log_file_;
    bool is_logging_started_ = false;

    // ROS2 Interfaces
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_joint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arm_pose_sub_;

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