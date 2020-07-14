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
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/OccupancyGrid.h>

constexpr auto kScanTopic = "/scan";

namespace obstacle_detection {

ObstacleDetection::ObstacleDetection() : tf_listener_(tf_buffer_) {
    ros::NodeHandle nh;
    scan_sub_ = nh.subscribe(kScanTopic, 1, &ObstacleDetection::ReceiveScan, this);
    local_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/local_map", 1);
}

void ObstacleDetection::ReceiveScan(const sensor_msgs::LaserScanConstPtr& scan) {
    grid_map::GridMap local_map{};
    local_map.setGeometry({15, 15}, 0.05);
    local_map.add("occupancy");
    local_map.setFrameId("odom");
    local_map.setTimestamp(ros::Time::now().toNSec());

    sensor_msgs::PointCloud2 cloud;
    laser_projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, tf_buffer_);
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::moveFromROSMsg(cloud, *current_cloud);

    for (const auto& point : current_cloud->points) {
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
}

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_detection");
    ros::NodeHandle nh;

    obstacle_detection::ObstacleDetection obstacle_detection;
    ros::spin();

    return 0;
}
