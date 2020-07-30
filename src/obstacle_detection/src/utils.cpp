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

#include "obstacle_detection/utils.h"

#include <eigen_conversions/eigen_msg.h>
#include <tf2_eigen/tf2_eigen.h>

namespace obstacle_detection {

pcl::PointCloud<pcl::PointXYZ>::Ptr ProjectDepthImage(
        const cv::Mat& depth_image,
        const cv::Mat& intrinsics,
        const Eigen::Affine3f& sensor_to_robot_transform) {
    const int rows = depth_image.rows;
    const int cols = depth_image.cols;

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ default_point;
    default_point.x = std::numeric_limits<float>::quiet_NaN();
    default_point.y = std::numeric_limits<float>::quiet_NaN();
    default_point.z = std::numeric_limits<float>::quiet_NaN();
    point_cloud->points.resize(rows * cols, default_point);

    point_cloud->width = cols;
    point_cloud->height = rows;
    point_cloud->is_dense = false;

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            const uint16_t depth_val = depth_image.at<uint16_t>(r, c);

            // invalid points are set to 0
            if (depth_val == 0) {
                continue;
            }

            // this is apparently the way to convert a depth uint16_t to a floating-point value
            const float z = depth_val * 0.001f;
            const float x = (c - intrinsics.at<double>(0, 2)) * z / intrinsics.at<double>(0, 0);
            const float y = (r - intrinsics.at<double>(1, 2)) * z / intrinsics.at<double>(1, 1);

            Eigen::Vector3f point{x, y, z};
            point = sensor_to_robot_transform * point;

            const std::size_t index = r * cols + c;
            point_cloud->points[index].x = point[0];
            point_cloud->points[index].y = point[1];
            point_cloud->points[index].z = point[2];
        }
    }

    return point_cloud;
}

std::unique_ptr<octomap::Pointcloud> DepthImageToPointCloud(
        const cv::Mat& depth_image,
        const cv::Mat& intrinsics,
        const Eigen::Affine3f& sensor_to_robot_transform) {
    const int rows = depth_image.rows;
    const int cols = depth_image.cols;

    std::unique_ptr<octomap::Pointcloud> point_cloud(std::make_unique<octomap::Pointcloud>());

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            const uint16_t depth_val = depth_image.at<uint16_t>(r, c);

            // invalid points are set to 0
            if (depth_val == 0) {
                continue;
            }

            // this is apparently the way to convert a depth uint16_t to a floating-point value
            const float z = depth_val * 0.001f;
            const float x = (c - intrinsics.at<double>(0, 2)) * z / intrinsics.at<double>(0, 0);
            const float y = (r - intrinsics.at<double>(1, 2)) * z / intrinsics.at<double>(1, 1);

            Eigen::Vector3f point{x, y, z};
            point = sensor_to_robot_transform * point;

            point_cloud->push_back(point[0], point[1], point[2]);
        }
    }

    return point_cloud;
}

bool LookupTransform(
        const std::string &from,
        const std::string &to,
        const ros::Time &timestamp,
        const tf2_ros::Buffer &tf_buffer,
        Eigen::Affine3f& transform) {
    const ros::Duration waitTime(0.1);

    geometry_msgs::TransformStamped transform_stamped;
    try {
        transform_stamped = tf_buffer.lookupTransform(to, from, timestamp, waitTime);
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_STREAM("Failed transform from " << from << " to " << to);
        return false;
    }

    transform = tf2::transformToEigen(transform_stamped).cast<float>();
    return true;
}

}
