/*
 * Copyright (c) 2020. Mohit Deshpande.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Odometry.h>

#include <motion_controller_msgs/WheelEncoders.h>
#include <robot_config/robot_config.h>

static constexpr auto WHEEL_ENCODER_TOPIC = "/wheels/encoders";

motion_controller_msgs::WheelEncodersConstPtr encoders_;

void ReceiveEncoders(const motion_controller_msgs::WheelEncodersConstPtr& encoders) {
    encoders_ = encoders;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry_publisher_node");
    ros::NodeHandle nh;

    ros::Subscriber encoders_sub = nh.subscribe(WHEEL_ENCODER_TOPIC, 1, &ReceiveEncoders);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);

    double x{}, y{}, theta{};

    ros::Time last_update = ros::Time::now();
    ros::Rate rate(40);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();

        if (!encoders_) continue;

        double dt = (encoders_->header.stamp - last_update).toSec();
        // wheel velocities in [rad/s]
        double w_l = encoders_->left / dt / robot_config::ENCODERS_PER_REV * 2 * M_PI;
        double w_r = encoders_->right / dt / robot_config::ENCODERS_PER_REV * 2 * M_PI;
        double v = robot_config::WHEEL_RADIUS / 2. * (w_l + w_r);
        double w = robot_config::WHEEL_RADIUS / robot_config::WHEEL_BASE * (w_r - w_l);

        x += v * dt * std::cos(theta);
        y += v * dt * std::sin(theta);
        theta += w * dt;

        nav_msgs::Odometry odom{};
        odom.header.frame_id = "odom";
        odom.header.stamp = encoders_->header.stamp;
        odom.child_frame_id = "base_link";

        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.orientation = tf2::toMsg(q);

        odom.twist.twist.linear.x = v;
        odom.twist.twist.angular.z = w;

        odom_pub.publish(odom);
        last_update = encoders_->header.stamp;
        encoders_ = nullptr;
    }

    return 0;
}
