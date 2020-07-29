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

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <grid_map_core/GridMap.hpp>
#include <laser_geometry/laser_geometry.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/camera_subscriber.h>

namespace obstacle_detection {

class ObstacleDetection {
public:
    ObstacleDetection();
private:
    void ReceiveScan(const sensor_msgs::LaserScanConstPtr& scan);
    void ReceiveDepth(
            const sensor_msgs::ImageConstPtr& aligned_depth_msg,
            const sensor_msgs::CameraInfoConstPtr& camera_info_msg);

    ros::NodeHandle nh_;
    laser_geometry::LaserProjection laser_projector_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Publisher octomap_pub_;
    ros::Publisher local_map_pub_;
    ros::Subscriber scan_sub_;

    // image subscribers
    image_transport::ImageTransport it_;
    image_transport::CameraSubscriber depth_sub_;

    cv::Mat depth_camera_intrinsics_{1, 1, CV_64FC1};
    ros::Publisher point_cloud_pub_;
};

}
