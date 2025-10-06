//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    pointcloud_to_laserscan_main.cc
\brief   A point cloud to laserscan convert that actually works without
         crashing or throwing buffer overflow errors.
\author  Joydeep Biswas, (C) 2020
*/
//========================================================================

#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <vector>

#include "config_reader/config_reader.h"
#ifdef ROS2
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#else
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#endif
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "ros_compat.h"
#include <cfloat>

using Eigen::Vector2f;
using Eigen::Vector3f;
#ifndef ROS2
using sensor_msgs::LaserScan;
using sensor_msgs::PointCloud2;
#endif

static inline float Sq(const float x) { return x * x; }

DECLARE_string(helpon);
DECLARE_int32(v);

#ifdef ROS2
static sensor_msgs::msg::LaserScan laser_msg_;
#else
static sensor_msgs::LaserScan laser_msg_;
#endif
float max_height_ = FLT_MAX;
float min_height_ = -FLT_MAX;
float min_sq_range_ = 0;
float max_sq_range_ = FLT_MAX;
std::string laser_topic_ = "scan";
std::string pointcloud_topic_ = "pointcloud";

static const Eigen::Affine3f frame_tf_ =
    Eigen::Translation3f(0, 0, 0.85) *
    Eigen::AngleAxisf(0.0, Vector3f::UnitX());
const std::string target_frame_("nav_base_link");

#ifdef ROS2
PublisherPtr<sensor_msgs::msg::LaserScan> scan_publisher_;
#else
PublisherPtr<sensor_msgs::LaserScan> scan_publisher_;
#endif
NodePtr node_;

#ifdef ROS2
void PointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
#else
void PointcloudCallback(const sensor_msgs::PointCloud2& msg) {
#endif
#ifdef ROS2
    if (FLAGS_v > 1) {
        printf("PointCloud2 message, t=%f\n", msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
    }
    laser_msg_.header = msg->header;
#else
    if (FLAGS_v > 1) {
        printf("PointCloud2 message, t=%f\n", msg.header.stamp.toSec());
    }
    laser_msg_.header = msg.header;
#endif
    laser_msg_.header.frame_id = target_frame_;

    for (float& r : laser_msg_.ranges) {
        r = FLT_MAX;
    }
    // Iterate through pointcloud
#ifdef ROS2
    for (sensor_msgs::PointCloud2ConstIterator<float>
             iter_x(*msg, "x"),
         iter_y(*msg, "y"), iter_z(*msg, "z");
         iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
#else
    for (sensor_msgs::PointCloud2ConstIterator<float>
             iter_x(msg, "x"),
         iter_y(msg, "y"), iter_z(msg, "z");
         iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
#endif
        if (!isfinite(*iter_x) || !isfinite(*iter_y) || !isfinite(*iter_z)) {
            continue;
        }
        const Vector3f p = frame_tf_ * Vector3f(*iter_x, *iter_y, *iter_z);

        if (p.z() > max_height_ || p.z() < min_height_) {
            continue;
        }

        const float sq_range = Sq(p.x()) + Sq(p.y());
        if (sq_range < min_sq_range_ || sq_range > max_sq_range_) continue;
        const float angle = atan2(p.y(), p.x());

        const size_t idx = static_cast<size_t>(floor(
            (angle - laser_msg_.angle_min) / laser_msg_.angle_increment));
        if (idx < laser_msg_.ranges.size() && laser_msg_.ranges[idx] > sq_range) {
            laser_msg_.ranges[idx] = sq_range;
        }
    }

    for (float& r : laser_msg_.ranges) {
        if (r < FLT_MAX) {
            r = sqrt(r);
        } else {
            r = 0;
        }
    }
#ifdef ROS2
    scan_publisher_->publish(laser_msg_);
#else
    scan_publisher_.publish(laser_msg_);
#endif
}

DEFINE_string(config, "config/highbeams.lua", "Configuration file");

void LoadConfig() {
    CONFIG_FLOAT(range_min, "pointcloud_to_laser.range_min");
    CONFIG_FLOAT(range_max, "pointcloud_to_laser.range_max");
    CONFIG_FLOAT(angle_min, "pointcloud_to_laser.angle_min");
    CONFIG_FLOAT(angle_max, "pointcloud_to_laser.angle_max");
    CONFIG_FLOAT(height_min, "pointcloud_to_laser.height_min");
    CONFIG_FLOAT(height_max, "pointcloud_to_laser.height_max");
    CONFIG_FLOAT(num_ranges, "pointcloud_to_laser.num_ranges");
    CONFIG_STRING(pointcloud_topic_, "pointcloud_to_laser.pointcloud_topic");
    CONFIG_STRING(laser_topic_, "pointcloud_to_laser.laser_topic");

    config_reader::ConfigReader reader({FLAGS_config});

    laser_msg_.angle_min = CONFIG_angle_min;
    laser_msg_.angle_max = CONFIG_angle_max;
    if (CONFIG_num_ranges <= 0) {
        LOG(FATAL) << "num_ranges must be > 0, got: " << CONFIG_num_ranges;
    }
    if (CONFIG_angle_max <= CONFIG_angle_min) {
        LOG(FATAL) << "angle_max must be > angle_min, got: " << CONFIG_angle_max << " <= " << CONFIG_angle_min;
    }
    laser_msg_.angle_increment =
        (CONFIG_angle_max - CONFIG_angle_min) / CONFIG_num_ranges;
    laser_msg_.range_min = CONFIG_range_min;
    laser_msg_.range_max = CONFIG_range_max;
    laser_msg_.ranges.resize(CONFIG_num_ranges);
    min_height_ = CONFIG_height_min;
    max_height_ = CONFIG_height_max;
    min_sq_range_ = Sq(CONFIG_range_min);
    max_sq_range_ = Sq(CONFIG_range_max);
    laser_topic_ = CONFIG_laser_topic_;
    pointcloud_topic_ = CONFIG_pointcloud_topic_;
}

int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);
    LoadConfig();

#ifdef ROS2
    ROS_INIT(argc, argv, "pointcloud_to_laserscan");
    node_ = CREATE_NODE("pointcloud_to_laserscan");

    auto pointcloud_sub = CREATE_SUBSCRIBER(node_, sensor_msgs::msg::PointCloud2,
                                            pointcloud_topic_, 1, PointcloudCallback);
    scan_publisher_ = CREATE_PUBLISHER(node_, sensor_msgs::msg::LaserScan,
                                       laser_topic_, 1);

    ROS_SPIN();
#else
    ROS_INIT(argc, argv, "pointcloud_to_laserscan");
    node_ = CREATE_NODE("pointcloud_to_laserscan");

    ros::Subscriber pointcloud_sub = CREATE_SUBSCRIBER(node_, sensor_msgs::PointCloud2, pointcloud_topic_, 1, &PointcloudCallback);
    scan_publisher_ = CREATE_PUBLISHER(node_, sensor_msgs::LaserScan, laser_topic_, 1);

    ROS_SPIN();
#endif

    return 0;
}
