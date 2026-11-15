/*
 * @file            src/flysky_ibus_node.cpp
 * @description
 * @author          rsazid99 <rsazid99@gmail.com>
 * @createTime      2026-08-08 03:27:33
 * @lastModified    2026-08-08 03:27:33
 * Copyright (c) 2026 Sazid Rahman Simanto
 * SPDX-License-Identifier: MIT
*/
/**
 * flysky_ibus_node.cpp
 *
 * Reads a FlySky i-BUS receiver over serial and publishes cmd_vel, so the rover
 * can be driven from the radio. Port of the flysky_bridge Python node.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "robot_base_controller/ibus.hpp"
#include "robot_base_controller/serial_port.hpp"

class FlyskyIbusNode : public rclcpp::Node
{
public:
    FlyskyIbusNode()
    : Node("flysky_ibus_bridge")
    {
        port_ = declare_parameter<std::string>("port", "/dev/ttyACM6");
        baud_ = declare_parameter<int>("baud", 115200);

        // Point this at something other than cmd_vel (e.g. cmd_vel_manual) to
        // run the radio alongside an autonomous planner behind a twist mux,
        // instead of both fighting over the same topic.
        cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");

        // Channel mapping, zero based:
        //   CH1 right stick left/right -> angular
        //   CH2 right stick up/down    -> linear
        //   CH5 VRA                    -> arm switch
        linear_channel_ = declare_parameter<int>("linear_channel", 1);
        angular_channel_ = declare_parameter<int>("angular_channel", 0);
        arm_channel_ = declare_parameter<int>("arm_channel", 4);

        max_linear_ = declare_parameter<double>("max_linear", 0.4);
        max_angular_ = declare_parameter<double>("max_angular", 0.20);

        invert_linear_ = declare_parameter<bool>("invert_linear", false);
        invert_angular_ = declare_parameter<bool>("invert_angular", true);

        use_arm_channel_ = declare_parameter<bool>("use_arm_channel", true);
        arm_threshold_ = declare_parameter<int>("arm_threshold", 1700);

        // Set speed_scale_channel to -1 to disable knob based speed scaling.
        speed_scale_channel_ = declare_parameter<int>("speed_scale_channel", -1);
        min_speed_scale_ = declare_parameter<double>("min_speed_scale", 0.25);
        max_speed_scale_ = declare_parameter<double>("max_speed_scale", 1.0);

        // The base controller also ramps motor RPM, so keep these high to avoid
        // stacking two slow ramps on top of each other.
        linear_accel_ = declare_parameter<double>("linear_accel", 2.0);
        angular_accel_ = declare_parameter<double>("angular_accel", 2.0);

        deadband_ = declare_parameter<int>("deadband", 40);
        timeout_sec_ = declare_parameter<double>("timeout_sec", 0.3);
        publish_hz_ = declare_parameter<double>("publish_hz", 100.0);

        debug_channels_ = declare_parameter<bool>("debug_channels", false);

        if (linear_channel_ < 0 || linear_channel_ >= static_cast<int>(ibus::NUM_CHANNELS) ||
            angular_channel_ < 0 || angular_channel_ >= static_cast<int>(ibus::NUM_CHANNELS))
        {
            throw std::runtime_error("linear_channel and angular_channel must be in [0, 13]");
        }

        if (use_arm_channel_ &&
            (arm_channel_ < 0 || arm_channel_ >= static_cast<int>(ibus::NUM_CHANNELS)))
        {
            throw std::runtime_error("arm_channel must be in [0, 13] when use_arm_channel is true");
        }

        serial_.open(port_, robot_base::baudToSpeed(baud_));
        serial_.flushInput();

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, rclcpp::QoS(10));

        last_rx_time_ = now();
        last_timer_time_ = now();

        publish_hz_ = std::clamp(publish_hz_, 10.0, 200.0);

        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / publish_hz_)),
            std::bind(&FlyskyIbusNode::timerCallback, this));

        RCLCPP_INFO(
            get_logger(), "FlySky i-BUS bridge opened %s at %d, publishing %s",
            port_.c_str(), baud_, cmd_pub_->get_topic_name());
        RCLCPP_INFO(
            get_logger(), "Channel mapping: linear=CH%d, angular=CH%d, arm=CH%d",
            linear_channel_ + 1, angular_channel_ + 1, arm_channel_ + 1);
    }

    ~FlyskyIbusNode() override
    {
        try {
            publishCmd(0.0, 0.0);
        } catch (...) {
            // Nothing useful to do while shutting down.
        }
    }

private:
    std::optional<ibus::Channels> readLatestFrame()
    {
        uint8_t chunk[512];

        try {
            size_t waiting = serial_.available();

            if (waiting == 0) {
                waiting = sizeof(chunk);
            }

            while (waiting > 0) {
                const size_t want = std::min(waiting, sizeof(chunk));
                const size_t got = serial_.read(chunk, want);

                if (got == 0) {
                    break;
                }

                buffer_.insert(buffer_.end(), chunk, chunk + got);
                waiting -= got;
            }
        } catch (const std::exception & e) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "Serial read error: %s", e.what());
            return std::nullopt;
        }

        return ibus::extractLatest(buffer_);
    }

    void publishCmd(double linear, double angular)
    {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = linear;
        msg.angular.z = angular;
        cmd_pub_->publish(msg);
    }

    void smoothPublish(double target_linear, double target_angular)
    {
        const rclcpp::Time now_time = now();
        double dt = (now_time - last_timer_time_).seconds();
        last_timer_time_ = now_time;

        if (dt <= 0.0 || dt > 0.1) {
            dt = 0.01;
        }

        current_linear_ = approach(current_linear_, target_linear, linear_accel_ * dt);
        current_angular_ = approach(current_angular_, target_angular, angular_accel_ * dt);

        if (std::abs(current_linear_) < 1e-4) {
            current_linear_ = 0.0;
        }

        if (std::abs(current_angular_) < 1e-4) {
            current_angular_ = 0.0;
        }

        publishCmd(current_linear_, current_angular_);
    }

    static double approach(double current, double target, double max_delta)
    {
        if (target > current + max_delta) {
            return current + max_delta;
        }

        if (target < current - max_delta) {
            return current - max_delta;
        }

        return target;
    }

    void timerCallback()
    {
        const auto channels = readLatestFrame();
        const rclcpp::Time now_time = now();

        if (!channels) {
            const double elapsed = (now_time - last_rx_time_).seconds();

            if (elapsed > timeout_sec_) {
                // Signal lost. Command a stop immediately; the base controller
                // still ramps the motors down smoothly.
                if (!signal_lost_) {
                    RCLCPP_WARN(
                        get_logger(), "No i-BUS frames for %.2f s, commanding stop", elapsed);
                    signal_lost_ = true;
                }

                current_linear_ = 0.0;
                current_angular_ = 0.0;
                publishCmd(0.0, 0.0);
            }

            return;
        }

        if (signal_lost_) {
            RCLCPP_INFO(get_logger(), "i-BUS signal recovered");
            signal_lost_ = false;
        }

        last_rx_time_ = now_time;

        const auto & ch = *channels;

        double speed_scale = 1.0;
        if (speed_scale_channel_ >= 0 &&
            speed_scale_channel_ < static_cast<int>(ibus::NUM_CHANNELS))
        {
            speed_scale = ibus::mapKnob(
                ch[static_cast<size_t>(speed_scale_channel_)], min_speed_scale_, max_speed_scale_);
        }

        double linear = ibus::mapChannel(
            ch[static_cast<size_t>(linear_channel_)], max_linear_ * speed_scale, 1500, deadband_);

        double angular = ibus::mapChannel(
            ch[static_cast<size_t>(angular_channel_)], max_angular_ * speed_scale, 1500, deadband_);

        if (invert_linear_) {
            linear = -linear;
        }

        if (invert_angular_) {
            angular = -angular;
        }

        bool armed = true;
        if (use_arm_channel_) {
            armed = ch[static_cast<size_t>(arm_channel_)] > arm_threshold_;
        }

        const double target_linear = armed ? linear : 0.0;
        const double target_angular = armed ? angular : 0.0;

        if (armed != was_armed_) {
            RCLCPP_INFO(get_logger(), armed ? "Radio ARMED" : "Radio DISARMED");
            was_armed_ = armed;
        }

        if (debug_channels_) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "CH1=%u CH2=%u CH3=%u CH4=%u CH5=%u CH6=%u armed=%d target=(%.3f, %.3f)",
                ch[0], ch[1], ch[2], ch[3], ch[4], ch[5],
                static_cast<int>(armed), target_linear, target_angular);
        }

        smoothPublish(target_linear, target_angular);
    }

    std::string port_ = "/dev/ttyACM6";
    int baud_ = 115200;
    std::string cmd_vel_topic_ = "cmd_vel";

    int linear_channel_ = 1;
    int angular_channel_ = 0;
    int arm_channel_ = 4;

    double max_linear_ = 0.4;
    double max_angular_ = 0.20;

    bool invert_linear_ = false;
    bool invert_angular_ = true;

    bool use_arm_channel_ = true;
    int arm_threshold_ = 1700;

    int speed_scale_channel_ = -1;
    double min_speed_scale_ = 0.25;
    double max_speed_scale_ = 1.0;

    double linear_accel_ = 2.0;
    double angular_accel_ = 2.0;

    int deadband_ = 40;
    double timeout_sec_ = 0.3;
    double publish_hz_ = 100.0;
    bool debug_channels_ = false;

    robot_base::SerialPort serial_;
    std::vector<uint8_t> buffer_;

    double current_linear_ = 0.0;
    double current_angular_ = 0.0;
    bool signal_lost_ = false;
    bool was_armed_ = false;

    rclcpp::Time last_rx_time_;
    rclcpp::Time last_timer_time_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<FlyskyIbusNode>();
        rclcpp::spin(node);
    } catch (const std::exception & e) {
        std::cerr << "flysky_ibus_bridge failed: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
