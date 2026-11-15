/*
 * @file            src/odom_fusion_node.cpp
 * @description
 * @author          rsazid99 <rsazid99@gmail.com>
 * @createTime      2026-08-08 04:15:46
 * @lastModified    2026-08-08 04:15:46
 * Copyright (c) 2026 Sazid Rahman Simanto
 * SPDX-License-Identifier: MIT
*/
/**
 * odom_fusion_node.cpp
 *
 * Fuses DDSM115 wheel odometry with the BNO085 IMU in an extended Kalman filter
 * and publishes the resulting odometry plus the odom -> base_link transform.
 *
 * State:      x = [px, py, yaw, v, omega]
 * Prediction: unicycle model with constant velocity and constant yaw rate
 * Updates:    wheel linear speed, optional wheel yaw rate,
 *             IMU yaw rate (gyro z), IMU absolute yaw (rotation vector)
 *
 * On a skid steer 4WD chassis the wheels slip badly in turns, so the wheel yaw
 * rate is disabled by default and heading comes from the IMU, while the wheels
 * supply the forward speed the IMU cannot observe.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

namespace
{

constexpr int N = 5;   // state size

/// Variance reported for the dimensions this filter does not estimate: z, roll,
/// pitch, and lateral velocity.
///
/// The usual convention is a huge number (1e6) meaning "unobserved". That is
/// misleading here: the rover is a wheeled ground vehicle, so z, roll and pitch
/// are not unknown, they are known to be near zero. This value says exactly
/// that, with enough slack for uneven ground. It also keeps RViz's covariance
/// visualisation readable, since RViz draws the ellipsoid to scale and a
/// variance of 1e6 becomes a 1000 m blob that swallows the whole scene.
constexpr double PLANAR_VARIANCE = 0.01;   // 1 sigma = 0.1 m / 0.1 rad

constexpr int IX = 0;
constexpr int IY = 1;
constexpr int IYAW = 2;
constexpr int IV = 3;
constexpr int IW = 4;

using StateVector = Eigen::Matrix<double, N, 1>;
using StateMatrix = Eigen::Matrix<double, N, N>;

/// Wraps an angle to (-pi, pi].
double normalizeAngle(double angle)
{
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);

    if (angle <= 0.0) {
        angle += 2.0 * M_PI;
    }

    return angle - M_PI;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
    tf2::Quaternion tq(q.x, q.y, q.z, q.w);

    if (tq.length2() < 1e-9) {
        return 0.0;
    }

    tq.normalize();

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);

    return yaw;
}

}  // namespace

class OdomFusionNode : public rclcpp::Node
{
public:
    OdomFusionNode()
    : Node("odom_fusion")
    {
        odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
        publish_tf_ = declare_parameter<bool>("publish_tf", true);
        publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);

        use_imu_yaw_ = declare_parameter<bool>("use_imu_yaw", true);
        use_imu_gyro_ = declare_parameter<bool>("use_imu_gyro", true);
        use_wheel_angular_ = declare_parameter<bool>("use_wheel_angular", false);

        // Set both of these when the IMU is mounted upside down.
        invert_imu_yaw_ = declare_parameter<bool>("invert_imu_yaw", false);
        imu_yaw_offset_ = declare_parameter<double>("imu_yaw_offset_deg", 0.0) * M_PI / 180.0;
        zero_yaw_on_start_ = declare_parameter<bool>("zero_yaw_on_start", true);

        wheel_linear_stddev_ = declare_parameter<double>("wheel_linear_stddev", 0.05);
        wheel_angular_stddev_ = declare_parameter<double>("wheel_angular_stddev", 0.30);
        imu_gyro_stddev_ = declare_parameter<double>("imu_gyro_stddev", 0.02);
        imu_yaw_stddev_ = declare_parameter<double>("imu_yaw_stddev", 0.05);

        proc_pos_stddev_ = declare_parameter<double>("process_position_stddev", 0.02);
        proc_yaw_stddev_ = declare_parameter<double>("process_yaw_stddev", 0.01);
        proc_vel_stddev_ = declare_parameter<double>("process_velocity_stddev", 0.5);
        proc_omega_stddev_ = declare_parameter<double>("process_omega_stddev", 1.0);

        input_timeout_ = declare_parameter<double>("input_timeout", 1.0);
        stale_velocity_stddev_ = declare_parameter<double>("stale_velocity_stddev", 0.5);

        if (publish_rate_hz_ <= 0.0) {
            throw std::runtime_error("publish_rate_hz must be positive");
        }

        if (!use_imu_yaw_ && !use_imu_gyro_ && !use_wheel_angular_) {
            throw std::runtime_error(
                "no heading source enabled: set at least one of use_imu_yaw, "
                "use_imu_gyro, use_wheel_angular");
        }

        resetState();

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(20));

        wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "wheel_odom", rclcpp::QoS(20),
            std::bind(&OdomFusionNode::wheelOdomCallback, this, std::placeholders::_1));

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", rclcpp::SensorDataQoS(),
            std::bind(&OdomFusionNode::imuCallback, this, std::placeholders::_1));

        if (publish_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        reset_service_ = create_service<std_srvs::srv::Empty>(
            "reset_odometry",
            std::bind(
                &OdomFusionNode::resetService, this,
                std::placeholders::_1, std::placeholders::_2));

        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / publish_rate_hz_)),
            std::bind(&OdomFusionNode::publishOdometry, this));

        RCLCPP_INFO(
            get_logger(),
            "Odometry fusion ready (imu_yaw=%d imu_gyro=%d wheel_angular=%d)",
            static_cast<int>(use_imu_yaw_), static_cast<int>(use_imu_gyro_),
            static_cast<int>(use_wheel_angular_));
    }

private:
    void resetState()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        x_.setZero();
        P_.setIdentity();
        P_ *= 1e-3;

        last_predict_time_ = now();
        last_wheel_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        last_imu_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        yaw_offset_ = 0.0;
        yaw_offset_set_ = false;
    }

    void resetService(
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        resetState();
        RCLCPP_INFO(get_logger(), "Odometry reset to origin");
    }

    /// Uses the message stamp when it is set, otherwise the node clock.
    rclcpp::Time stampOf(const std_msgs::msg::Header & header) const
    {
        const rclcpp::Time stamp(header.stamp, RCL_ROS_TIME);
        return (stamp.nanoseconds() == 0) ? now() : stamp;
    }

    /// Advances the filter to `stamp`. Caller must hold mutex_.
    void predictTo(const rclcpp::Time & stamp)
    {
        double dt = (stamp - last_predict_time_).seconds();

        if (dt <= 0.0) {
            return;   // out of order or duplicate stamp
        }

        if (dt > 1.0) {
            // A long gap makes the linearisation meaningless; step the clock
            // forward and inflate the covariance instead of integrating.
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Prediction gap of %.2f s, skipping integration", dt);

            last_predict_time_ = stamp;
            P_ += StateMatrix::Identity() * 1.0;
            return;
        }

        last_predict_time_ = stamp;

        const double yaw = x_(IYAW);
        const double v = x_(IV);
        const double w = x_(IW);

        const double sin_yaw = std::sin(yaw);
        const double cos_yaw = std::cos(yaw);

        x_(IX) += v * cos_yaw * dt;
        x_(IY) += v * sin_yaw * dt;
        x_(IYAW) = normalizeAngle(yaw + w * dt);

        StateMatrix F = StateMatrix::Identity();
        F(IX, IYAW) = -v * sin_yaw * dt;
        F(IX, IV) = cos_yaw * dt;
        F(IY, IYAW) = v * cos_yaw * dt;
        F(IY, IV) = sin_yaw * dt;
        F(IYAW, IW) = dt;

        StateVector q;
        q << proc_pos_stddev_ * proc_pos_stddev_,
             proc_pos_stddev_ * proc_pos_stddev_,
             proc_yaw_stddev_ * proc_yaw_stddev_,
             proc_vel_stddev_ * proc_vel_stddev_,
             proc_omega_stddev_ * proc_omega_stddev_;

        const StateVector q_dt = q * dt;

        P_ = F * P_ * F.transpose();
        P_ += StateMatrix(q_dt.asDiagonal());
    }

    /// Scalar Kalman update on a single state component. Caller must hold mutex_.
    void updateScalar(int index, double measurement, double variance, bool is_angle)
    {
        Eigen::Matrix<double, 1, N> H = Eigen::Matrix<double, 1, N>::Zero();
        H(0, index) = 1.0;

        double innovation = measurement - x_(index);

        if (is_angle) {
            innovation = normalizeAngle(innovation);
        }

        const double S = (H * P_ * H.transpose())(0, 0) + variance;

        if (!(S > 0.0) || !std::isfinite(innovation)) {
            return;
        }

        const StateVector K = P_ * H.transpose() / S;

        x_ += K * innovation;
        x_(IYAW) = normalizeAngle(x_(IYAW));

        // Joseph form keeps P symmetric and positive definite under repeated
        // updates from sensors arriving at very different rates.
        const StateMatrix I_KH = StateMatrix::Identity() - K * H;
        P_ = I_KH * P_ * I_KH.transpose() + K * variance * K.transpose();
    }

    void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const rclcpp::Time stamp = stampOf(msg->header);

        std::lock_guard<std::mutex> lock(mutex_);

        predictTo(stamp);
        last_wheel_time_ = stamp;

        updateScalar(
            IV, msg->twist.twist.linear.x,
            wheel_linear_stddev_ * wheel_linear_stddev_, false);

        if (use_wheel_angular_) {
            updateScalar(
                IW, msg->twist.twist.angular.z,
                wheel_angular_stddev_ * wheel_angular_stddev_, false);
        }
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        const rclcpp::Time stamp = stampOf(msg->header);

        const double sign = invert_imu_yaw_ ? -1.0 : 1.0;
        const double gyro_z = sign * msg->angular_velocity.z;

        double yaw_meas = 0.0;
        bool yaw_valid = false;

        if (use_imu_yaw_) {
            // A zeroed orientation quaternion means the message carries no
            // orientation estimate.
            const auto & q = msg->orientation;
            const double norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;

            if (norm2 > 1e-6) {
                yaw_meas = sign * yawFromQuaternion(q) + imu_yaw_offset_;
                yaw_valid = true;
            }
        }

        std::lock_guard<std::mutex> lock(mutex_);

        predictTo(stamp);
        last_imu_time_ = stamp;

        if (use_imu_gyro_) {
            updateScalar(IW, gyro_z, imu_gyro_stddev_ * imu_gyro_stddev_, false);
        }

        if (yaw_valid) {
            if (!yaw_offset_set_) {
                // Start the odom frame aligned with the robot's initial heading
                // rather than with magnetic north.
                yaw_offset_ = zero_yaw_on_start_ ? yaw_meas : 0.0;
                yaw_offset_set_ = true;
            }

            const double yaw = normalizeAngle(yaw_meas - yaw_offset_);

            // Trust the sensor's own accuracy estimate when it is worse than
            // the configured floor.
            double variance = imu_yaw_stddev_ * imu_yaw_stddev_;
            const double reported = msg->orientation_covariance[8];

            if (std::isfinite(reported) && reported > variance) {
                variance = reported;
            }

            updateScalar(IYAW, yaw, variance, true);
        }
    }

    void publishOdometry()
    {
        StateVector x;
        StateMatrix P;
        rclcpp::Time stamp = now();

        {
            std::lock_guard<std::mutex> lock(mutex_);

            predictTo(stamp);

            if (input_timeout_ > 0.0) {
                // Without a live velocity source the constant velocity model
                // would coast forever, so a dead input decays the estimate to
                // zero rather than letting the pose run away.
                const double stale_var = stale_velocity_stddev_ * stale_velocity_stddev_;

                if (isStale("wheel_odom", last_wheel_time_, stamp)) {
                    updateScalar(IV, 0.0, stale_var, false);
                }

                const bool imu_used = use_imu_yaw_ || use_imu_gyro_;

                if (imu_used && !use_wheel_angular_ &&
                    isStale("imu/data", last_imu_time_, stamp))
                {
                    updateScalar(IW, 0.0, stale_var, false);
                }
            }

            x = x_;
            P = P_;
        }

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, x(IYAW));
        const auto q_msg = tf2::toMsg(q);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;

        odom.pose.pose.position.x = x(IX);
        odom.pose.pose.position.y = x(IY);
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = q_msg;

        odom.twist.twist.linear.x = x(IV);
        odom.twist.twist.angular.z = x(IW);

        // Row major 6x6 over [x, y, z, roll, pitch, yaw].
        odom.pose.covariance.fill(0.0);
        odom.pose.covariance[0] = P(IX, IX);
        odom.pose.covariance[1] = P(IX, IY);
        odom.pose.covariance[5] = P(IX, IYAW);
        odom.pose.covariance[6] = P(IY, IX);
        odom.pose.covariance[7] = P(IY, IY);
        odom.pose.covariance[11] = P(IY, IYAW);
        odom.pose.covariance[30] = P(IYAW, IX);
        odom.pose.covariance[31] = P(IYAW, IY);
        odom.pose.covariance[35] = P(IYAW, IYAW);
        odom.pose.covariance[14] = PLANAR_VARIANCE;   // z
        odom.pose.covariance[21] = PLANAR_VARIANCE;   // roll
        odom.pose.covariance[28] = PLANAR_VARIANCE;   // pitch

        odom.twist.covariance.fill(0.0);
        odom.twist.covariance[0] = P(IV, IV);
        odom.twist.covariance[5] = P(IV, IW);
        odom.twist.covariance[30] = P(IW, IV);
        odom.twist.covariance[35] = P(IW, IW);
        odom.twist.covariance[7] = PLANAR_VARIANCE;    // y, no lateral slip modelled
        odom.twist.covariance[14] = PLANAR_VARIANCE;   // z
        odom.twist.covariance[21] = PLANAR_VARIANCE;   // roll
        odom.twist.covariance[28] = PLANAR_VARIANCE;   // pitch

        odom_pub_->publish(odom);

        if (tf_broadcaster_) {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = stamp;
            tf_msg.header.frame_id = odom_frame_;
            tf_msg.child_frame_id = base_frame_;
            tf_msg.transform.translation.x = x(IX);
            tf_msg.transform.translation.y = x(IY);
            tf_msg.transform.translation.z = 0.0;
            tf_msg.transform.rotation = q_msg;

            tf_broadcaster_->sendTransform(tf_msg);
        }
    }

    /// Reports whether an input has gone quiet, warning at a throttled rate.
    bool isStale(const char * topic, const rclcpp::Time & last, const rclcpp::Time & now_time)
    {
        if (last.nanoseconds() == 0) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000, "Still waiting for %s", topic);
            return true;
        }

        const double age = (now_time - last).seconds();

        if (age <= input_timeout_) {
            return false;
        }

        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "%s is stale (%.1f s old); decaying the estimate toward zero", topic, age);

        return true;
    }

    std::string odom_frame_ = "odom";
    std::string base_frame_ = "base_link";
    bool publish_tf_ = true;
    double publish_rate_hz_ = 50.0;

    bool use_imu_yaw_ = true;
    bool use_imu_gyro_ = true;
    bool use_wheel_angular_ = false;
    bool invert_imu_yaw_ = false;
    double imu_yaw_offset_ = 0.0;
    bool zero_yaw_on_start_ = true;

    double wheel_linear_stddev_ = 0.05;
    double wheel_angular_stddev_ = 0.30;
    double imu_gyro_stddev_ = 0.02;
    double imu_yaw_stddev_ = 0.05;

    double proc_pos_stddev_ = 0.02;
    double proc_yaw_stddev_ = 0.01;
    double proc_vel_stddev_ = 0.5;
    double proc_omega_stddev_ = 1.0;

    double input_timeout_ = 1.0;
    double stale_velocity_stddev_ = 0.5;

    std::mutex mutex_;
    StateVector x_ = StateVector::Zero();
    StateMatrix P_ = StateMatrix::Identity();
    rclcpp::Time last_predict_time_;
    rclcpp::Time last_wheel_time_;
    rclcpp::Time last_imu_time_;
    double yaw_offset_ = 0.0;
    bool yaw_offset_set_ = false;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<OdomFusionNode>();
        rclcpp::spin(node);
    } catch (const std::exception & e) {
        std::cerr << "odom_fusion failed: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
