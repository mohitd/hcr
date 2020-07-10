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
#include <geometry_msgs/TwistStamped.h>

#include "joystick/joystick.h"

// [m/s]
static constexpr float V_MAX = 1.0;
// [rad/s]
static constexpr float W_MAX = M_PI;

// [Hz]
static constexpr auto SAMPLE_RATE = 200;

int main(int argc, char** argv) {
    ros::init(argc, argv, "joystick_node");
    ros::NodeHandle nh;

    joystick::JoystickInterface joystick;
    joystick::JoystickValues joystick_values{};
    geometry_msgs::TwistStamped velocity{};

    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);
    ros::Rate sample_rate(SAMPLE_RATE);
    while (ros::ok()) {
        if (joystick.Sample(joystick_values)) {
            float normalized_linear
                = static_cast<float>(joystick_values.axis_1.y) / joystick::JoystickValues::MAX_AXIS_VALUE;
            float normalized_angular
                = static_cast<float>(joystick_values.axis_1.x) / joystick::JoystickValues::MAX_AXIS_VALUE;
            velocity.twist.linear.x = normalized_linear * V_MAX;
            velocity.twist.angular.z = normalized_angular * W_MAX;
            velocity.header.stamp = ros::Time::now();
            velocity_pub.publish(velocity);
        }
        ros::spinOnce();
        sample_rate.sleep();
    }

    return 0;
}

