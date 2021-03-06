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

#include <memory>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <pcl_ros/point_cloud.h>
#include <octomap/octomap.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Eigen>

namespace obstacle_detection {

pcl::PointCloud<pcl::PointXYZ>::Ptr ProjectDepthImage(
        const cv::Mat& depth_image,
        const cv::Mat& intrinsics,
        const Eigen::Affine3f& sensor_to_robot_transform);

std::unique_ptr<octomap::Pointcloud> DepthImageToPointCloud(
        const cv::Mat& depth_image,
        const cv::Mat& intrinsics,
        const Eigen::Affine3f& sensor_to_robot_transform);

bool LookupTransform(
        const std::string &from,
        const std::string &to,
        const ros::Time &timestamp,
        const tf2_ros::Buffer &tf_buffer,
        Eigen::Affine3f& transform);

}
