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

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <laser_geometry/laser_geometry.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <motion_controller_msgs/WheelEncoders.h>

#include "ekf/ekf.h"
#include "ekf/scan_matcher.h"

namespace ekf {

class EkfNode {
public:
    EkfNode();
    void Run();
private:
    void ReceiveOdometry(const motion_controller_msgs::WheelEncodersConstPtr& odometry);
    void ReceiveImu(const sensor_msgs::ImuConstPtr& imu);
    void ReceiveScan(const sensor_msgs::LaserScanConstPtr& scan);

    ekf::Ekf ekf_{};
    Eigen::MatrixXd imu_H_;
    Eigen::MatrixXd imu_R_;
    Eigen::MatrixXd odom_H_;
    Eigen::MatrixXd odom_R_;
    Eigen::MatrixXd scan_H_;
    Eigen::MatrixXd scan_R_;

    bool first_odom_msg = true;
    bool first_imu_msg = true;
    bool first_scan_msg = true;
    ros::Time last_odom_stamp_;
    ros::Time last_imu_stamp_;

    laser_geometry::LaserProjection laser_projector_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud_;
    ekf::Ekf prev_scan_ekf_{};

    tf2_ros::Buffer tf_buffer_{};
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    ros::Publisher pose_pub_;
    ros::Subscriber odometry_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber scan_sub_;
};

}
