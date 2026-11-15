/*
 * @file            src/bno085_imu_node.cpp
 * @description
 * @author          rsazid99 <rsazid99@gmail.com>
 * @createTime      2026-08-08 03:27:33
 * @lastModified    2026-08-08 03:27:33
 * Copyright (c) 2026 Sazid Rahman Simanto
 * SPDX-License-Identifier: MIT
*/
/**
 * bno085_imu_node.cpp
 *
 * Streams the BNO085 over I2C and publishes sensor_msgs/Imu and
 * sensor_msgs/MagneticField. This is the C++ equivalent of the adafruit_bno08x
 * Python sample: it enables the accelerometer, calibrated gyroscope,
 * magnetometer, and rotation vector reports and reads them continuously.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "robot_base_controller/bno085.hpp"

using namespace std::chrono_literals;

class Bno085ImuNode : public rclcpp::Node
{
public:
    Bno085ImuNode()
    : Node("bno085_imu")
    {
        i2c_bus_ = declare_parameter<int>("i2c_bus", 7);
        i2c_address_ = declare_parameter<int>("i2c_address", 0x4A);
        max_i2c_transfer_ = declare_parameter<int>("max_i2c_transfer", 64);

        frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
        report_rate_hz_ = declare_parameter<double>("report_rate_hz", 100.0);
        poll_rate_hz_ = declare_parameter<double>("poll_rate_hz", 400.0);

        // The rotation vector fuses the magnetometer and gives an absolute
        // heading. The game rotation vector ignores the magnetometer, which is
        // more stable near the motors but drifts slowly in yaw.
        use_game_rotation_vector_ = declare_parameter<bool>("use_game_rotation_vector", false);
        publish_magnetometer_ = declare_parameter<bool>("publish_magnetometer", true);
        soft_reset_ = declare_parameter<bool>("soft_reset", true);

        orientation_stddev_ = declare_parameter<double>("orientation_stddev", 0.02);
        angular_velocity_stddev_ = declare_parameter<double>("angular_velocity_stddev", 0.01);
        linear_acceleration_stddev_ = declare_parameter<double>("linear_acceleration_stddev", 0.05);
        magnetic_field_stddev_ = declare_parameter<double>("magnetic_field_stddev", 5.0e-6);

        if (report_rate_hz_ <= 0.0 || poll_rate_hz_ <= 0.0) {
            throw std::runtime_error("report_rate_hz and poll_rate_hz must be positive");
        }

        if (i2c_address_ < 0x03 || i2c_address_ > 0x77) {
            throw std::runtime_error("i2c_address must be a valid 7 bit I2C address");
        }

        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());

        if (publish_magnetometer_) {
            mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(
                "imu/mag", rclcpp::SensorDataQoS());
        }

        connect();

        const auto poll_period =
            std::chrono::duration<double>(1.0 / std::max(poll_rate_hz_, 1.0));

        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(poll_period),
            std::bind(&Bno085ImuNode::pollOnce, this));

        last_report_time_ = now();
    }

private:
    void connect()
    {
        imu_.open(i2c_bus_, static_cast<uint8_t>(i2c_address_),
            static_cast<size_t>(std::max(max_i2c_transfer_, 16)));

        RCLCPP_INFO(
            get_logger(), "Opened /dev/i2c-%d, BNO085 at address 0x%02X",
            i2c_bus_, i2c_address_);

        if (soft_reset_ && !imu_.softReset()) {
            RCLCPP_WARN(
                get_logger(), "Soft reset did not produce SHTP traffic (%s); continuing",
                imu_.lastError().c_str());
        }

        std::string info;
        if (imu_.readProductId(info)) {
            RCLCPP_INFO(get_logger(), "BNO085 %s", info.c_str());
        } else {
            RCLCPP_WARN(
                get_logger(), "Product ID query failed: %s", imu_.lastError().c_str());
        }

        enableReports();
    }

    void enableReports()
    {
        const auto interval_us =
            static_cast<uint32_t>(std::lround(1.0e6 / report_rate_hz_));

        const uint8_t rotation_report = use_game_rotation_vector_
            ? bno085::REPORT_GAME_ROTATION_VECTOR
            : bno085::REPORT_ROTATION_VECTOR;

        const std::pair<uint8_t, const char *> reports[] = {
            {bno085::REPORT_ACCELEROMETER, "accelerometer"},
            {bno085::REPORT_GYROSCOPE, "gyroscope"},
            {rotation_report, use_game_rotation_vector_
                ? "game rotation vector" : "rotation vector"},
        };

        for (const auto & [id, name] : reports) {
            if (!imu_.enableReport(id, interval_us)) {
                throw std::runtime_error(
                    std::string("failed to enable ") + name + ": " + imu_.lastError());
            }

            RCLCPP_INFO(get_logger(), "Enabled %s at %.0f Hz", name, report_rate_hz_);
        }

        if (publish_magnetometer_) {
            if (imu_.enableReport(bno085::REPORT_MAGNETOMETER, interval_us)) {
                RCLCPP_INFO(get_logger(), "Enabled magnetometer at %.0f Hz", report_rate_hz_);
            } else {
                RCLCPP_WARN(
                    get_logger(), "Failed to enable magnetometer: %s", imu_.lastError().c_str());
            }
        }
    }

    void pollOnce()
    {
        unsigned updated = bno085::UPDATED_NONE;

        try {
            updated = imu_.service(sample_);
        } catch (const std::exception & e) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 2000, "I2C service failed: %s", e.what());
            return;
        }

        const rclcpp::Time stamp = now();

        if (updated == bno085::UPDATED_NONE) {
            const double silent = (stamp - last_report_time_).seconds();

            if (silent > 2.0) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "No IMU reports for %.1f s (last error: %s). Re-enabling reports.",
                    silent, imu_.lastError().c_str());

                last_report_time_ = stamp;

                try {
                    enableReports();
                } catch (const std::exception & e) {
                    RCLCPP_ERROR_THROTTLE(
                        get_logger(), *get_clock(), 5000,
                        "Failed to re-enable reports: %s", e.what());
                }
            }

            return;
        }

        last_report_time_ = stamp;

        // The rotation vector is the slowest changing anchor of a report batch,
        // so one Imu message is emitted per orientation update.
        if (updated & bno085::UPDATED_QUAT) {
            publishImu(stamp);
        }

        if ((updated & bno085::UPDATED_MAG) && mag_pub_) {
            publishMag(stamp);
        }
    }

    void publishImu(const rclcpp::Time & stamp)
    {
        sensor_msgs::msg::Imu msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id_;

        msg.orientation.x = sample_.qi;
        msg.orientation.y = sample_.qj;
        msg.orientation.z = sample_.qk;
        msg.orientation.w = sample_.qr;

        msg.angular_velocity.x = sample_.gx;
        msg.angular_velocity.y = sample_.gy;
        msg.angular_velocity.z = sample_.gz;

        msg.linear_acceleration.x = sample_.ax;
        msg.linear_acceleration.y = sample_.ay;
        msg.linear_acceleration.z = sample_.az;

        // Use the sensor reported orientation accuracy when it exceeds the
        // configured floor, so a disturbed magnetometer widens the covariance.
        const double orientation_sigma =
            std::max(orientation_stddev_, sample_.quat_accuracy_rad);

        fillDiagonal(msg.orientation_covariance, orientation_sigma);
        fillDiagonal(msg.angular_velocity_covariance, angular_velocity_stddev_);
        fillDiagonal(msg.linear_acceleration_covariance, linear_acceleration_stddev_);

        imu_pub_->publish(msg);
    }

    void publishMag(const rclcpp::Time & stamp)
    {
        sensor_msgs::msg::MagneticField msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id_;

        // The sensor reports microtesla; the message is in tesla.
        msg.magnetic_field.x = sample_.mx * 1.0e-6;
        msg.magnetic_field.y = sample_.my * 1.0e-6;
        msg.magnetic_field.z = sample_.mz * 1.0e-6;

        fillDiagonal(msg.magnetic_field_covariance, magnetic_field_stddev_);

        mag_pub_->publish(msg);
    }

    template<typename Covariance>
    static void fillDiagonal(Covariance & cov, double stddev)
    {
        cov.fill(0.0);
        const double var = stddev * stddev;
        cov[0] = var;
        cov[4] = var;
        cov[8] = var;
    }

    int i2c_bus_ = 7;
    int i2c_address_ = 0x4A;
    int max_i2c_transfer_ = 64;

    std::string frame_id_ = "imu_link";
    double report_rate_hz_ = 100.0;
    double poll_rate_hz_ = 400.0;
    bool use_game_rotation_vector_ = false;
    bool publish_magnetometer_ = true;
    bool soft_reset_ = true;

    double orientation_stddev_ = 0.02;
    double angular_velocity_stddev_ = 0.01;
    double linear_acceleration_stddev_ = 0.05;
    double magnetic_field_stddev_ = 5.0e-6;

    bno085::Bno085I2c imu_;
    bno085::ImuSample sample_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_report_time_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<Bno085ImuNode>();
        rclcpp::spin(node);
    } catch (const std::exception & e) {
        std::cerr << "bno085_imu failed: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
