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
        scan_sub_{nh_.subscribe(kScanTopic, 1, &ObstacleDetection::ReceiveScan, this)},
        octomap_pub_{nh_.advertise<octomap_msgs::Octomap>(kOctomapTopic, 1)},
        local_map_pub_{nh_.advertise<nav_msgs::OccupancyGrid>(kLocalMapTopic, 1)},
        it_{nh_},
        depth_sub_{it_.subscribeCamera(kDepthImageBaseTopic, 1, &ObstacleDetection::ReceiveDepth, this)},
        point_cloud_pub_{nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud", 1)}{
}

void ObstacleDetection::ReceiveScan(const sensor_msgs::LaserScanConstPtr& scan) {
    sensor_msgs::PointCloud2 cloud;
    laser_projector_.transformLaserScanToPointCloud(kBaseLink, *scan, cloud, tf_buffer_);

    octomap::Pointcloud octomap_cloud;
    octomap::pointCloud2ToOctomap(cloud, octomap_cloud);

    octomap::OcTree tree(0.05);
    tree.insertPointCloud(octomap_cloud, {0, 0, 0});
    tree.updateInnerOccupancy();

    octomap_msgs::Octomap octomap_msg;
    octomap_msg.header.frame_id = "base_link";
    octomap_msg.header.stamp = scan->header.stamp;
    octomap_msgs::binaryMapToMsg(tree, octomap_msg);
    octomap_pub_.publish(octomap_msg);

}

void ObstacleDetection::ReceiveDepth(
        const sensor_msgs::ImageConstPtr& aligned_depth_msg,
        const sensor_msgs::CameraInfoConstPtr& camera_info_msg) {
    cv_bridge::CvImageConstPtr depth_cv_image = cv_bridge::toCvShare(aligned_depth_msg);
    const cv::Mat& depth_img = depth_cv_image->image;

    Eigen::Affine3f sensor_to_robot_affine;
    if (!LookupTransform(aligned_depth_msg->header.frame_id,
            kBaseLink,
            aligned_depth_msg->header.stamp,
            tf_buffer_,
            sensor_to_robot_affine)) {
        return;
    }

    if (depth_camera_intrinsics_.cols == 1) {
        depth_camera_intrinsics_ = cv::Mat(3, 3, CV_64FC1, (void*)camera_info_msg->K.data());
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = ProjectDepthImage(depth_img, depth_camera_intrinsics_, sensor_to_robot_affine);

    grid_map::GridMap local_map{};
    local_map.setGeometry({15, 15}, 0.05);
    local_map.add("occupancy");
    local_map.setFrameId("odom");
    local_map.setTimestamp(ros::Time::now().toNSec());

    for (const auto& point : point_cloud->points) {
        for (grid_map::LineIterator it(local_map, grid_map::Position(0, 0), {point.x, point.y}); !it.isPastEnd(); ++it) {
            auto it_cpy = it;
            ++it_cpy;
            if (it_cpy.isPastEnd()) {
                local_map.at("occupancy", *it) = 1.0f;
            } else {
                local_map.at("occupancy", *it) = -1.0f;
            }
        }
    }

    nav_msgs::OccupancyGrid local_map_msg;
    grid_map::GridMapRosConverter::toOccupancyGrid(local_map, "occupancy", -1, 1, local_map_msg);
    local_map_pub_.publish(local_map_msg);

    sensor_msgs::PointCloud2 ros_point_cloud;
    pcl::toROSMsg(*point_cloud, ros_point_cloud);
    ros_point_cloud.header.frame_id = kBaseLink;
    ros_point_cloud.header.stamp = aligned_depth_msg->header.stamp;
    point_cloud_pub_.publish(ros_point_cloud);
}

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_detection");
    ros::NodeHandle nh;

    obstacle_detection::ObstacleDetection obstacle_detection;
    ros::spin();

    return 0;
}
