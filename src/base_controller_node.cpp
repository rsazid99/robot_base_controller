/*
 * @file            src/base_controller_node.cpp
 * @description
 * @author          rsazid99 <rsazid99@gmail.com>
 * @createTime      2026-11-15 16:32:35
 * @lastModified    2026-11-15 16:32:35
 * Copyright (c) 2026 Sazid Rahman Simanto
 * SPDX-License-Identifier: MIT
*/
/**
 * base_controller_node.cpp
 *
 * Drives the four DDSM115 wheels over RS485 and publishes wheel odometry.
 *
 * The odometry published here is pure dead reckoning; odom_fusion_node consumes
 * it together with the IMU and owns the odom -> base_link transform. Set
 * publish_tf to true only when running this node standalone.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "robot_base_controller/ddsm115_protocol.hpp"
#include "robot_base_controller/serial_port.hpp"

using robot_base::SerialPort;

static constexpr double PI = 3.14159265358979323846;
static constexpr double CMD_DEADBAND = 1e-4;

class BaseControllerNode : public rclcpp::Node
{
public:
    BaseControllerNode()
    : Node("base_controller"), running_(false)
    {
        device_paths_ = declare_parameter<std::vector<std::string>>(
            "device_paths", {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"});

        wheel_radius_ = declare_parameter<double>("wheel_radius", 0.05);
        wheel_track_ = declare_parameter<double>("wheel_track", 0.385);
        wheel_base_ = declare_parameter<double>("wheel_base", 0.30);

        feedback_rate_hz_ = declare_parameter<double>("feedback_rate_hz", 10.0);
        cmd_vel_timeout_ = declare_parameter<double>("cmd_vel_timeout", 0.5);
        inter_cmd_gap_ms_ = declare_parameter<int>("inter_cmd_gap_ms", 0);
        accel_rpm_per_sec_ = declare_parameter<double>("accel_rpm_per_sec", 200.0);
        max_rpm_ = declare_parameter<double>("max_rpm", 200.0);

        wheel_ids_ = declare_parameter<std::vector<int64_t>>("wheel_ids", {1, 2, 3, 4});
        wheel_directions_ = declare_parameter<std::vector<std::string>>(
            "wheel_directions", {"forward", "backward", "forward", "backward"});

        odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
        publish_tf_ = declare_parameter<bool>("publish_tf", false);

        if (wheel_ids_.size() != 4 || wheel_directions_.size() != 4 || device_paths_.size() != 4) {
            throw std::runtime_error(
                "wheel_ids, wheel_directions, and device_paths must each contain exactly 4 entries");
        }

        for (const auto & dir : wheel_directions_) {
            if (dir != "forward" && dir != "backward") {
                throw std::runtime_error(
                    "wheel_directions entries must be \"forward\" or \"backward\"");
            }
        }

        if (wheel_radius_ <= 0.0 || wheel_track_ <= 0.0 || feedback_rate_hz_ <= 0.0) {
            throw std::runtime_error(
                "wheel_radius, wheel_track, and feedback_rate_hz must be positive");
        }

        if (max_rpm_ <= 0.0) {
            throw std::runtime_error("max_rpm must be positive");
        }

        if (inter_cmd_gap_ms_ < 0) {
            throw std::runtime_error("inter_cmd_gap_ms must be non-negative");
        }

        for (const auto id : wheel_ids_) {
            if (id < 1 || id > 255) {
                throw std::runtime_error("wheel_ids entries must be in the range [1, 255]");
            }
        }

        for (size_t i = 0; i < serial_ports_.size(); i++) {
            try {
                serial_ports_[i].open(device_paths_[i]);
            } catch (const std::exception & e) {
                RCLCPP_FATAL(
                    get_logger(), "Failed to open wheel %zu serial port %s: %s",
                    i, device_paths_[i].c_str(), e.what());
                throw;
            }

            RCLCPP_INFO(
                get_logger(), "Opened wheel %zu serial port: %s", i, device_paths_[i].c_str());

            if (robot_base::enableRs485Mode(serial_ports_[i].fd())) {
                RCLCPP_INFO(get_logger(), "RS485 kernel mode enabled on %s",
                    device_paths_[i].c_str());
            } else {
                RCLCPP_INFO(
                    get_logger(),
                    "RS485 kernel ioctl not supported on %s; assuming the adapter handles direction",
                    device_paths_[i].c_str());
            }
        }

        for (size_t i = 0; i < wheel_ids_.size(); i++) {
            auto frame = ddsm115::buildDriveModeFrame(
                static_cast<uint8_t>(wheel_ids_[i]), ddsm115::DriveMode::VELOCITY);

            serial_ports_[i].write(frame.data(), frame.size());
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            serial_ports_[i].flushInput();

            RCLCPP_INFO(
                get_logger(), "Motor %ld on %s set to velocity mode",
                static_cast<long>(wheel_ids_[i]), device_paths_[i].c_str());
        }

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", rclcpp::QoS(10),
            std::bind(&BaseControllerNode::cmdVelCallback, this, std::placeholders::_1));

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("wheel_odom", rclcpp::QoS(10));
        rpm_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("motor_rpms", rclcpp::QoS(10));
        current_pub_ =
            create_publisher<std_msgs::msg::Float32MultiArray>("motor_currents", rclcpp::QoS(10));

        if (publish_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        motor_rpms_.fill(0);
        motor_currents_.fill(0.0f);
        current_cmd_rpms_.fill(0.0);

        last_cmd_time_ = now();
        last_odom_time_ = now();
        last_tx_time_ = std::chrono::steady_clock::now();

        running_ = true;

        tx_thread_ = std::thread(&BaseControllerNode::txThreadFunc, this);
        feedback_thread_ = std::thread(&BaseControllerNode::feedbackThreadFunc, this);

        RCLCPP_INFO(get_logger(), "Base controller ready.");
    }

    ~BaseControllerNode() override
    {
        running_ = false;
        cmd_cv_.notify_all();

        if (tx_thread_.joinable()) {
            tx_thread_.join();
        }

        if (feedback_thread_.joinable()) {
            feedback_thread_.join();
        }

        try {
            sendStopNow();
        } catch (const std::exception & e) {
            RCLCPP_WARN(get_logger(), "Failed to stop motors cleanly during shutdown: %s", e.what());
        }

        std::lock_guard<std::mutex> lock(serial_mutex_);
        for (auto & port : serial_ports_) {
            port.close();
        }
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double linear = msg->linear.x;
        double angular = msg->angular.z;

        if (std::abs(linear) < CMD_DEADBAND) {
            linear = 0.0;
        }

        if (std::abs(angular) < CMD_DEADBAND) {
            angular = 0.0;
        }

        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            cmd_linear_ = linear;
            cmd_angular_ = angular;
            last_cmd_time_ = now();
        }

        cmd_cv_.notify_one();
    }

    void txThreadFunc()
    {
        bool watchdog_warned = false;

        while (running_ && rclcpp::ok()) {
            double linear = 0.0;
            double angular = 0.0;
            bool watchdog_stop = false;

            {
                std::unique_lock<std::mutex> lock(cmd_mutex_);

                cmd_cv_.wait_for(lock, std::chrono::milliseconds(50));

                if (!running_ || !rclcpp::ok()) {
                    break;
                }

                linear = cmd_linear_;
                angular = cmd_angular_;

                if (cmd_vel_timeout_ > 0.0) {
                    const double elapsed = (now() - last_cmd_time_).seconds();
                    if (elapsed > cmd_vel_timeout_) {
                        linear = 0.0;
                        angular = 0.0;
                        watchdog_stop = true;
                    }
                }
            }

            // Normal command, zero command, and watchdog timeout all use the same
            // ramp, so key release and signal loss decelerate smoothly.
            try {
                sendRpms(linear, angular);
            } catch (const std::exception & e) {
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 2000, "Motor write failed: %s", e.what());
            }

            if (watchdog_stop) {
                if (!watchdog_warned) {
                    RCLCPP_WARN(get_logger(), "cmd_vel watchdog: ramping motors to stop");
                    watchdog_warned = true;
                }
            } else {
                watchdog_warned = false;
            }
        }
    }

    void sendStopNow()
    {
        std::lock_guard<std::mutex> serial_lock(serial_mutex_);

        for (size_t i = 0; i < wheel_ids_.size(); i++) {
            if (!serial_ports_[i].isOpen()) {
                continue;
            }

            current_cmd_rpms_[i] = 0.0;

            auto frame = ddsm115::buildRpmFrame(static_cast<uint8_t>(wheel_ids_[i]), 0);
            serial_ports_[i].write(frame.data(), frame.size());

            std::this_thread::sleep_for(
                std::chrono::milliseconds(std::max(inter_cmd_gap_ms_, 1)));
        }

        last_tx_time_ = std::chrono::steady_clock::now();
    }

    void sendRpms(double linear, double angular)
    {
        const double conv = 60.0 / (2.0 * PI * wheel_radius_);
        const double left_vel = linear - angular * wheel_track_ / 2.0;
        const double right_vel = linear + angular * wheel_track_ / 2.0;

        std::array<double, 4> target{};

        for (size_t i = 0; i < wheel_ids_.size(); i++) {
            // Index order is front-left, front-right, rear-left, rear-right.
            double vel = (i == 1 || i == 3) ? right_vel : left_vel;

            if (wheel_directions_[i] == "backward") {
                vel = -vel;
            }

            target[i] = std::clamp(vel * conv, -max_rpm_, max_rpm_);
        }

        const auto now_time = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now_time - last_tx_time_).count();
        last_tx_time_ = now_time;

        dt = std::clamp(dt, 0.001, 0.2);

        const double max_step = accel_rpm_per_sec_ * dt;

        for (size_t i = 0; i < 4; i++) {
            const double diff = target[i] - current_cmd_rpms_[i];

            if (std::abs(diff) <= max_step) {
                current_cmd_rpms_[i] = target[i];
            } else {
                current_cmd_rpms_[i] += (diff > 0.0) ? max_step : -max_step;
            }
        }

        std::lock_guard<std::mutex> serial_lock(serial_mutex_);

        for (size_t i = 0; i < wheel_ids_.size(); i++) {
            const auto rpm_val = static_cast<int16_t>(std::lround(current_cmd_rpms_[i]));

            auto frame = ddsm115::buildRpmFrame(static_cast<uint8_t>(wheel_ids_[i]), rpm_val);

            // Do not flush the input here; flushing would delete pending feedback.
            serial_ports_[i].write(frame.data(), frame.size());

            std::this_thread::sleep_for(
                std::chrono::milliseconds(std::max(inter_cmd_gap_ms_, 1)));
        }
    }

    void feedbackThreadFunc()
    {
        const auto period = std::chrono::duration<double>(1.0 / feedback_rate_hz_);
        auto next_wake = std::chrono::steady_clock::now() + period;

        while (running_ && rclcpp::ok()) {
            try {
                std::lock_guard<std::mutex> serial_lock(serial_mutex_);

                for (size_t i = 0; i < wheel_ids_.size(); i++) {
                    pollFeedback(static_cast<uint8_t>(wheel_ids_[i]), static_cast<int>(i));
                }
            } catch (const std::exception & e) {
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 2000, "Feedback poll failed: %s", e.what());
            }

            publishOdometry();
            publishMotorFeedback();

            std::this_thread::sleep_until(next_wake);
            next_wake += period;

            const auto now_time = std::chrono::steady_clock::now();
            if (next_wake < now_time) {
                next_wake = now_time + period;
            }
        }
    }

    void pollFeedback(uint8_t motor_id, int idx)
    {
        auto req = ddsm115::buildFeedbackRequestFrame(motor_id);
        auto & port = serial_ports_[static_cast<size_t>(idx)];

        // Flush only right before a fresh request.
        port.flushInput();
        port.write(req.data(), req.size());

        // Give the USB-RS485 adapter and motor time to turn the line around.
        std::this_thread::sleep_for(std::chrono::milliseconds(3));

        ddsm115::Frame reply{};
        size_t filled = 0;

        // A motor with a wrong or duplicate ID replies with a different first
        // byte; without keeping those bytes the log cannot distinguish a
        // misconfigured ID from a dead wire.
        std::array<uint8_t, 16> stray{};
        size_t stray_count = 0;

        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(150);

        while (filled < ddsm115::FRAME_SIZE && std::chrono::steady_clock::now() < deadline) {
            uint8_t byte = 0;

            if (port.read(&byte, 1) == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            if (filled == 0 && byte != motor_id) {
                if (stray_count < stray.size()) {
                    stray[stray_count] = byte;
                }
                stray_count++;
                continue;
            }

            reply[filled++] = byte;
        }

        if (filled < ddsm115::FRAME_SIZE) {
            const auto warn_now = std::chrono::steady_clock::now();
            auto & last_warn = last_feedback_warn_[static_cast<size_t>(idx)];

            if (warn_now - last_warn < std::chrono::seconds(2)) {
                return;
            }

            last_warn = warn_now;

            std::ostringstream oss;

            for (size_t j = 0; j < filled; j++) {
                oss << "0x" << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                    << static_cast<int>(reply[j]) << " ";
            }

            std::ostringstream stray_oss;

            if (stray_count > 0) {
                stray_oss << "; discarded " << std::dec << stray_count << " stray byte(s): ";

                for (size_t j = 0; j < std::min(stray_count, stray.size()); j++) {
                    stray_oss << "0x" << std::hex << std::uppercase << std::setw(2)
                              << std::setfill('0') << static_cast<int>(stray[j]) << " ";
                }

                if (stray_count > stray.size()) {
                    stray_oss << "... ";
                }

                stray_oss << "(wrong motor ID on this channel?)";
            }

            RCLCPP_WARN(
                get_logger(),
                "Motor %d: feedback timeout, got %zu/%zu bytes: %s%s",
                motor_id, filled, ddsm115::FRAME_SIZE,
                oss.str().c_str(), stray_oss.str().c_str());

            return;
        }

        auto fb = ddsm115::parseFeedback(reply, motor_id);

        if (!fb.valid) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Motor %d: invalid feedback (CRC or mode mismatch)", motor_id);
            return;
        }

        if (fb.error) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Motor %d error flags: 0x%02X (sensor=%d overcurrent=%d phase=%d stall=%d)",
                motor_id, fb.error,
                fb.error & 0x01, (fb.error >> 1) & 0x01,
                (fb.error >> 2) & 0x01, (fb.error >> 3) & 0x01);
        }

        int16_t rpm = fb.rpm;

        if (wheel_directions_[static_cast<size_t>(idx)] == "backward") {
            rpm = static_cast<int16_t>(-rpm);
        }

        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            motor_rpms_[idx] = rpm;
            motor_currents_[idx] = fb.current;
        }
    }

    void publishOdometry()
    {
        std::array<int16_t, 4> rpms;

        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            rpms = motor_rpms_;
        }

        const rclcpp::Time now_time = now();
        const double dt = (now_time - last_odom_time_).seconds();
        last_odom_time_ = now_time;

        if (dt <= 0.0 || dt > 1.0) {
            return;
        }

        const double conv = (2.0 * PI * wheel_radius_) / 60.0;

        const double v_left = (rpms[0] + rpms[2]) / 2.0 * conv;
        const double v_right = (rpms[1] + rpms[3]) / 2.0 * conv;

        const double linear = (v_left + v_right) / 2.0;
        const double angular = (v_right - v_left) / wheel_track_;

        x_ += linear * std::cos(theta_) * dt;
        y_ += linear * std::sin(theta_) * dt;
        theta_ += angular * dt;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_);
        const auto q_msg = tf2::toMsg(q);

        if (tf_broadcaster_) {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = now_time;
            tf_msg.header.frame_id = odom_frame_;
            tf_msg.child_frame_id = base_frame_;
            tf_msg.transform.translation.x = x_;
            tf_msg.transform.translation.y = y_;
            tf_msg.transform.translation.z = 0.0;
            tf_msg.transform.rotation = q_msg;

            tf_broadcaster_->sendTransform(tf_msg);
        }

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now_time;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation = q_msg;

        odom.twist.twist.linear.x = linear;
        odom.twist.twist.angular.z = angular;

        // z, roll and pitch are not estimated. They are reported with a small
        // variance rather than the conventional 1e6 "unobserved" marker because
        // a wheeled ground vehicle genuinely is near zero in all three, and
        // because RViz draws covariance to scale: 1e6 becomes a 1000 m blob.
        constexpr double planar = 0.01;   // 1 sigma = 0.1 m / 0.1 rad

        odom.pose.covariance = {
            0.01, 0,    0,      0,      0,      0,
            0,    0.01, 0,      0,      0,      0,
            0,    0,    planar, 0,      0,      0,
            0,    0,    0,      planar, 0,      0,
            0,    0,    0,      0,      planar, 0,
            0,    0,    0,      0,      0,      0.05
        };

        odom.twist.covariance = odom.pose.covariance;

        odom_pub_->publish(odom);
    }

    void publishMotorFeedback()
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);

        std_msgs::msg::Float32MultiArray rpm_msg;
        std_msgs::msg::Float32MultiArray cur_msg;

        rpm_msg.data.reserve(4);
        cur_msg.data.reserve(4);

        for (size_t i = 0; i < 4; i++) {
            rpm_msg.data.push_back(static_cast<float>(motor_rpms_[i]));
            cur_msg.data.push_back(motor_currents_[i]);
        }

        rpm_pub_->publish(rpm_msg);
        current_pub_->publish(cur_msg);
    }

    std::vector<std::string> device_paths_;
    std::vector<int64_t> wheel_ids_;
    std::vector<std::string> wheel_directions_;

    double wheel_radius_ = 0.05;
    double wheel_track_ = 0.385;
    double wheel_base_ = 0.30;
    double feedback_rate_hz_ = 10.0;
    double cmd_vel_timeout_ = 0.5;
    int inter_cmd_gap_ms_ = 0;
    double accel_rpm_per_sec_ = 200.0;
    double max_rpm_ = 200.0;

    std::string odom_frame_ = "odom";
    std::string base_frame_ = "base_link";
    bool publish_tf_ = false;

    std::array<SerialPort, 4> serial_ports_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr rpm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr current_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::array<double, 4> current_cmd_rpms_{};
    std::chrono::steady_clock::time_point last_tx_time_{};

    std::mutex cmd_mutex_;
    std::condition_variable cmd_cv_;
    double cmd_linear_ = 0.0;
    double cmd_angular_ = 0.0;
    rclcpp::Time last_cmd_time_;

    std::mutex feedback_mutex_;
    std::array<int16_t, 4> motor_rpms_{};
    std::array<float, 4> motor_currents_{};
    std::array<std::chrono::steady_clock::time_point, 4> last_feedback_warn_{};

    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;
    rclcpp::Time last_odom_time_;

    std::atomic<bool> running_;
    std::thread tx_thread_;
    std::thread feedback_thread_;

    std::mutex serial_mutex_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<BaseControllerNode>();
        rclcpp::spin(node);
    } catch (const std::exception & e) {
        std::cerr << "base_controller failed: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
