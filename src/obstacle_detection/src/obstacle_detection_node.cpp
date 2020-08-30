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

#include "obstacle_detection/obstacle_detection_node.h"

#include <chrono>

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>

#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <octomap/OcTree.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>

#include "obstacle_detection/utils.h"

static constexpr auto kScanTopic = "/scan";
static constexpr auto kLocalMapTopic = "/local_map";
static constexpr auto kOctomapTopic = "/octomap";
static constexpr auto kDepthImageBaseTopic = "/camera/depth/image_rect_raw";

static constexpr auto kBaseLink = "base_link";

namespace obstacle_detection {

ObstacleDetection::ObstacleDetection() :
        nh_{},
        tf_listener_{tf_buffer_},
        octomap_pub_{nh_.advertise<octomap_msgs::Octomap>(kOctomapTopic, 1)},
        scan_sub_{nh_.subscribe(kScanTopic, 1, &ObstacleDetection::ReceiveScan, this)},
        it_{nh_},
        depth_sub_{it_.subscribeCamera(kDepthImageBaseTopic, 1, &ObstacleDetection::ReceiveDepth, this)} {
}

void ObstacleDetection::ReceiveScan(const sensor_msgs::LaserScanConstPtr& scan) {
    sensor_msgs::PointCloud2 cloud;
    laser_projector_.transformLaserScanToPointCloud(kBaseLink, *scan, cloud, tf_buffer_);

    octomap::Pointcloud octomap_cloud;
    octomap::pointCloud2ToOctomap(cloud, octomap_cloud);

    local_map_.insertPointCloud(octomap_cloud, {0, 0, 0});

    octomap_msgs::Octomap octomap_msg;
    octomap_msg.header.frame_id = "base_link";
    octomap_msg.header.stamp = scan->header.stamp;
    octomap_msgs::binaryMapToMsg(local_map_, octomap_msg);
    octomap_pub_.publish(octomap_msg);
}

void ObstacleDetection::ReceiveDepth(
        const sensor_msgs::ImageConstPtr& depth_img_msg,
        const sensor_msgs::CameraInfoConstPtr& camera_info_msg) {
    cv_bridge::CvImageConstPtr depth_cv_image = cv_bridge::toCvShare(depth_img_msg);
    const cv::Mat& depth_img = depth_cv_image->image;

    Eigen::Affine3f sensor_to_robot_affine;
    if (!LookupTransform(depth_img_msg->header.frame_id,
            kBaseLink,
            depth_img_msg->header.stamp,
            tf_buffer_,
            sensor_to_robot_affine)) {
        return;
    }

    if (depth_camera_intrinsics_.cols == 1) {
        depth_camera_intrinsics_ = cv::Mat(3, 3, CV_64FC1, (void*)camera_info_msg->K.data());
    }

    std::unique_ptr<octomap::Pointcloud> octomap_cloud = DepthImageToPointCloud(depth_img, depth_camera_intrinsics_, sensor_to_robot_affine);

    local_map_.insertPointCloud(*octomap_cloud, {0, 0, 0}, -1, false, true);

    octomap_msgs::Octomap octomap_msg;
    octomap_msg.header.frame_id = "base_link";
    octomap_msg.header.stamp = depth_img_msg->header.stamp;
    octomap_msgs::binaryMapToMsg(local_map_, octomap_msg);
    octomap_pub_.publish(octomap_msg);
}

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_detection");
    ros::NodeHandle nh;

    obstacle_detection::ObstacleDetection obstacle_detection;
    ros::spin();

    return 0;
}
