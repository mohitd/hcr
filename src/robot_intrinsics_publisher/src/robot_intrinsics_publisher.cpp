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
#include <tf2_ros/static_transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_intrinsics_publisher");

    tf2_ros::StaticTransformBroadcaster transform_broadcaster;
    geometry_msgs::TransformStamped base_link_to_camera;
    base_link_to_camera.header.stamp = ros::Time::now();
    base_link_to_camera.header.frame_id = "base_link";
    base_link_to_camera.child_frame_id = "camera";
    base_link_to_camera.transform.translation.x = 0.06;
    base_link_to_camera.transform.translation.y = 0.0;
    base_link_to_camera.transform.translation.z = 0.22;
    base_link_to_camera.transform.rotation.w = 1;

    geometry_msgs::TransformStamped base_link_to_laser;
    base_link_to_laser.header.stamp = ros::Time::now();
    base_link_to_laser.header.frame_id = "base_link";
    base_link_to_laser.child_frame_id = "laser";
    base_link_to_laser.transform.translation.x = -0.175;
    base_link_to_laser.transform.translation.y = 0.0;
    base_link_to_laser.transform.translation.z = 0.27;
    base_link_to_laser.transform.rotation.w = 1;

    transform_broadcaster.sendTransform({base_link_to_camera, base_link_to_laser});
    ros::spin();
    return 0;
}
