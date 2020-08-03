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

#include "ekf/ekf_node.h"

#include <chrono>

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <std_msgs/Float32.h>

#include "ekf/ekf.h"
#include "ekf/utils.h"
#include "ekf/scan_matcher.h"

#include <Eigen/Eigen>

#include <robot_config/robot_config.h>

static constexpr auto kOdometryTopic = "/wheels/encoders";
static constexpr auto kImuTopic = "/camera/imu";
static constexpr auto kScanTopic = "/scan";

static constexpr auto UPDATE_RATE = 40;
static constexpr auto STATE_SIZE = 5;
// motors can't go this fast and no one is going to move it this fast
// still WIP on why the MCU occasionally produces really high counts
static constexpr auto V_MAX_SANITY = 2.0;
static constexpr auto W_MAX_SANITY = M_2_PI;

namespace ekf {

EkfNode::EkfNode()
        : tf_listener_(tf_buffer_),
        prev_cloud_(new pcl::PointCloud<pcl::PointXYZ>) {
    ros::NodeHandle nh;

    odometry_sub_ = nh.subscribe(kOdometryTopic, 1, &EkfNode::ReceiveOdometry, this);
    imu_sub_ = nh.subscribe(kImuTopic, 1, &EkfNode::ReceiveImu, this);
    scan_sub_ = nh.subscribe(kScanTopic, 1, &EkfNode::ReceiveScan, this);

    imu_H_ = Eigen::MatrixXd::Zero(1, 5);
    imu_H_(0, 4) = 1;
    imu_R_ = Eigen::MatrixXd::Identity(1, 1) * 0.1;

    odom_H_ = Eigen::MatrixXd::Zero(1, 5);
    odom_H_(0, 3) = 1;
    odom_R_ = Eigen::MatrixXd::Identity(1, 1) * 0.1;

    scan_H_ = Eigen::MatrixXd::Zero(2, 5);
    scan_H_(0, 0) = 1;
    scan_H_(1, 1) = 1;
    scan_R_ = Eigen::MatrixXd::Identity(2, 2) * 0.1;

    // [x, y, theta, v, w]
    Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(STATE_SIZE);
    Eigen::MatrixXd initial_covariance = Eigen::MatrixXd::Identity(initial_state.size(), initial_state.size());
    ekf_.state = initial_state;
    ekf_.covariance = initial_covariance;

    // publishers
    pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose/odom", 1);
    last_imu_stamp_ = ros::Time::now();
    last_odom_stamp_ = ros::Time::now();
}

void EkfNode::ReceiveImu(const sensor_msgs::ImuConstPtr& imu) {
    if (first_imu_msg) {
        first_imu_msg = false;
        last_imu_stamp_ = imu->header.stamp;
        return;
    }

    geometry_msgs::TransformStamped camera_link;
    try {
        camera_link = tf_buffer_.lookupTransform("camera_link", imu->header.frame_id, imu->header.stamp);
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM(e.what());
        return;
    }

    geometry_msgs::Vector3 transformed_angular_velocity;
    tf2::doTransform(imu->angular_velocity, transformed_angular_velocity, camera_link);

    double dt = (imu->header.stamp - last_imu_stamp_).toSec();
    auto w = transformed_angular_velocity.z;

    // sanity checking on velocity
    if (w > W_MAX_SANITY) {
        ROS_WARN_STREAM("Impossible angular velocity!: " << w << "rad/s");
        return;
    }

    Eigen::VectorXd z(1);
    z(0) = w;

    if (!Update(ekf_, z, imu_H_, imu_R_)) {
        ROS_ERROR("Not updating state! S is singular!");
        return;
    }
    Predict(ekf_, dt);

    last_imu_stamp_ = imu->header.stamp;
}

void EkfNode::ReceiveOdometry(const motion_controller_msgs::WheelEncodersConstPtr& odometry) {
    if (first_odom_msg) {
        first_odom_msg = false;
        last_odom_stamp_ = odometry->header.stamp;
        return;
    }

    double dt = (odometry->header.stamp - last_odom_stamp_).toSec();
    double counts = 0.5 * (odometry->left + odometry->right);
    double v = counts * robot_config::DISTANCE_PER_COUNT / dt;

    // sanity checking on velociy
    if (v > V_MAX_SANITY) {
        ROS_WARN_STREAM("Impossible velocity!: " << v << "m/s (l="
            << odometry->left << ", r=" << odometry->right << ")");
        return;
    }

    Eigen::VectorXd z(1);
    z(0) = v;

    if (!Update(ekf_, z, odom_H_, odom_R_)) {
        ROS_ERROR("Not updating state! S is singular!");
        return;
    }

    Predict(ekf_, dt);

    last_odom_stamp_ = odometry->header.stamp;
}

void EkfNode::ReceiveScan(const sensor_msgs::LaserScanConstPtr& scan) {
    if (first_scan_msg) {
        sensor_msgs::PointCloud2 cloud;
        laser_projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, tf_buffer_);
        pcl::moveFromROSMsg(cloud, *prev_cloud_);
        prev_scan_ekf_ = ekf_;
        first_scan_msg = false;
        return;
    }

    sensor_msgs::PointCloud2 cloud;
    laser_projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, tf_buffer_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::moveFromROSMsg(cloud, *current_cloud);

    Eigen::Affine3d transform(ScanMatch({}, prev_cloud_, current_cloud));

    Eigen::Vector2d z = Eigen::Vector2d::Zero();
    z(0) = transform.translation()(0) + prev_scan_ekf_.state(0);
    z(1) = transform.translation()(1) + prev_scan_ekf_.state(1);

    if (!Update(ekf_, z, scan_H_, scan_R_)) {
        ROS_ERROR("Not updating state! S is singular!");
    }

    prev_cloud_ = current_cloud;
    prev_scan_ekf_ = ekf_;
}

void EkfNode::Run() {
    ros::Rate rate(UPDATE_RATE);
    while (ros::ok()) {
        geometry_msgs::PoseWithCovarianceStamped pose;
        pose.header.frame_id = "odom";
        pose.header.stamp = ros::Time::now();
        pose.pose.pose.position.x = ekf_.state(0);
        pose.pose.pose.position.y = ekf_.state(1);
        pose.pose.pose.position.z = 0;
        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, ekf_.state(2));
        pose.pose.pose.orientation = tf2::toMsg(orientation);
        pose.pose.covariance = toRos(ekf_.covariance);
        ROS_INFO_STREAM("x=" << pose.pose.pose.position.x
            << ", y=" << pose.pose.pose.position.y
            << ", theta=" << tf2::getYaw(pose.pose.pose.orientation));
        pose_pub_.publish(pose);

        geometry_msgs::TransformStamped transform;
        transform.transform.translation.x = pose.pose.pose.position.x;
        transform.transform.translation.y = pose.pose.pose.position.y;
        transform.transform.translation.z = pose.pose.pose.position.z;
        transform.transform.rotation = tf2::toMsg(orientation);
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";
        tf_broadcaster_.sendTransform(transform);

        rate.sleep();
        ros::spinOnce();
    }
}

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ekf");
    ros::NodeHandle nh;

    ekf::EkfNode slam;
    slam.Run();

    return 0;
}

