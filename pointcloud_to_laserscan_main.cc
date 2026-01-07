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
#include <atomic>
#include <chrono>
#include <thread>
#include <iostream>
#include <unistd.h>

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
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2/transform_datatypes.h"
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
std::string debug_pointcloud_topic_ = "debug_pointcloud";
std::string target_frame_;
std::string source_frame_override_;

#ifdef ROS2
PublisherPtr<sensor_msgs::msg::LaserScan> scan_publisher_;
PublisherPtr<sensor_msgs::msg::PointCloud2> debug_pc_publisher_;
#else
PublisherPtr<sensor_msgs::LaserScan> scan_publisher_;
PublisherPtr<sensor_msgs::PointCloud2> debug_pc_publisher_;
#endif
std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

NodePtr node_;

Eigen::Affine3f GetTransform(const std::string& source_frame, const std::string& target_frame,
                              const Time& stamp) {
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    try {
#ifdef ROS2
        geometry_msgs::msg::TransformStamped tf_stamped =
            tf_buffer_->lookupTransform(target_frame, source_frame, stamp);
        transform = tf2::transformToEigen(tf_stamped.transform).cast<float>();
#else
        geometry_msgs::TransformStamped tf_stamped =
            tf_buffer_->lookupTransform(target_frame, source_frame, stamp);
        transform = tf2::transformToEigen(tf_stamped.transform).cast<float>();
#endif
    } catch (const tf2::TransformException& ex) {
        printf("WARN: TF lookup failed: %s. Using identity transform.\n", ex.what());
    }

    return transform;
}

// Signal handling for graceful shutdown
std::atomic<bool> shutdown_requested{false};

void signalHandler(int signum) {
    static int signal_count = 0;
    signal_count++;
    
    if (signal_count == 1) {
        printf("\nReceived signal %d (Ctrl+C), initiating shutdown...\n", signum);
        shutdown_requested.store(true);
#ifdef ROS2
        rclcpp::shutdown();
#endif
    } else if (signal_count == 2) {
        printf("\nReceived second Ctrl+C, forcing exit...\n");
        exit(1);
    } else {
        printf("\nForce killing...\n");
        exit(2);
    }
}

#ifdef ROS2
void PointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
#else
void PointcloudCallback(const sensor_msgs::PointCloud2& msg) {
#endif
    // Check shutdown flag early to stop processing during shutdown
    if (shutdown_requested.load()) {
        return;
    }
#ifdef ROS2
    if (FLAGS_v > 1) {
        printf("PointCloud2 message, t=%f\n", msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
    }
    laser_msg_.header = msg->header;
    const std::string source_frame = source_frame_override_.empty() ? 
                                      msg->header.frame_id : source_frame_override_;
    Time stamp(rclcpp::Time(msg->header.stamp));
#else
    if (FLAGS_v > 1) {
        printf("PointCloud2 message, t=%f\n", msg.header.stamp.toSec());
    }
    laser_msg_.header = msg.header;
    const std::string source_frame = source_frame_override_.empty() ? 
                                      msg.header.frame_id : source_frame_override_;
    Time stamp = msg.header.stamp;
#endif

    // Handle frame transformation
    Eigen::Affine3f frame_tf = Eigen::Affine3f::Identity();
    if (!target_frame_.empty()) {
        laser_msg_.header.frame_id = target_frame_;
        frame_tf = GetTransform(source_frame, target_frame_, stamp);
    } else {
        laser_msg_.header.frame_id = source_frame;
    }

    for (float& r : laser_msg_.ranges) {
        r = FLT_MAX;
    }
    
    // Vector to store filtered points for debug visualization
    std::vector<Vector3f> filtered_points;
    
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
        const Vector3f p = frame_tf * Vector3f(*iter_x, *iter_y, *iter_z);

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
            // Store this point for debug visualization
            filtered_points.push_back(p);
        }
    }

    for (float& r : laser_msg_.ranges) {
        if (r < FLT_MAX) {
            r = sqrt(r);
        } else {
            r = 0;
        }
    }
    
    // Create debug PointCloud2 message with filtered points
#ifdef ROS2
    sensor_msgs::msg::PointCloud2 debug_pc_msg;
#else
    sensor_msgs::PointCloud2 debug_pc_msg;
#endif
    debug_pc_msg.header = laser_msg_.header;
    debug_pc_msg.header.frame_id = laser_msg_.header.frame_id;
    debug_pc_msg.height = 1;
    debug_pc_msg.width = filtered_points.size();
    debug_pc_msg.is_bigendian = false;
    debug_pc_msg.is_dense = true;
    
    // Set point cloud fields (x, y, z, intensity)
    debug_pc_msg.fields.resize(4);
    debug_pc_msg.fields[0].name = "x";
    debug_pc_msg.fields[0].offset = 0;
#ifdef ROS2
    debug_pc_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    debug_pc_msg.fields[0].count = 1;
    debug_pc_msg.fields[1].name = "y";
    debug_pc_msg.fields[1].offset = 4;
    debug_pc_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    debug_pc_msg.fields[1].count = 1;
    debug_pc_msg.fields[2].name = "z";
    debug_pc_msg.fields[2].offset = 8;
    debug_pc_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    debug_pc_msg.fields[2].count = 1;
    debug_pc_msg.fields[3].name = "intensity";
    debug_pc_msg.fields[3].offset = 12;
    debug_pc_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    debug_pc_msg.fields[3].count = 1;
#else
    debug_pc_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    debug_pc_msg.fields[0].count = 1;
    debug_pc_msg.fields[1].name = "y";
    debug_pc_msg.fields[1].offset = 4;
    debug_pc_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    debug_pc_msg.fields[1].count = 1;
    debug_pc_msg.fields[2].name = "z";
    debug_pc_msg.fields[2].offset = 8;
    debug_pc_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    debug_pc_msg.fields[2].count = 1;
    debug_pc_msg.fields[3].name = "intensity";
    debug_pc_msg.fields[3].offset = 12;
    debug_pc_msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    debug_pc_msg.fields[3].count = 1;
#endif
    
    debug_pc_msg.point_step = 16;  // 4 * 4 bytes (x, y, z, intensity)
    debug_pc_msg.row_step = debug_pc_msg.point_step * debug_pc_msg.width;
    debug_pc_msg.data.resize(debug_pc_msg.row_step);
    
    // Fill in the point data with intensity based on distance from origin
    for (size_t i = 0; i < filtered_points.size(); ++i) {
        const Vector3f& p = filtered_points[i];
        float* data_ptr = reinterpret_cast<float*>(&debug_pc_msg.data[i * debug_pc_msg.point_step]);
        data_ptr[0] = p.x();
        data_ptr[1] = p.y();
        data_ptr[2] = p.z();
        
        // Calculate intensity based on distance from origin (2D distance in XY plane)
        float distance = sqrt(p.x() * p.x() + p.y() * p.y());
        // Normalize intensity to 0-1 range based on max range, then scale to 0-255 for better visualization
        float max_range = sqrt(max_sq_range_);
        float normalized_distance = std::min(distance / max_range, 1.0f);
        data_ptr[3] = normalized_distance * 255.0f;  // intensity field
    }
    
#ifdef ROS2
    scan_publisher_->publish(laser_msg_);
    debug_pc_publisher_->publish(debug_pc_msg);
#else
    scan_publisher_.publish(laser_msg_);
    debug_pc_publisher_.publish(debug_pc_msg);
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
    CONFIG_STRING(debug_pointcloud_topic_, "pointcloud_to_laser.debug_pointcloud_topic");
    CONFIG_STRING(target_frame_, "pointcloud_to_laser.target_frame");
    CONFIG_STRING(source_frame_override_, "pointcloud_to_laser.source_frame_override");

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
    debug_pointcloud_topic_ = CONFIG_debug_pointcloud_topic_;
    target_frame_ = CONFIG_target_frame_;
    source_frame_override_ = CONFIG_source_frame_override_;
}

int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);
    LoadConfig();

    // Install signal handler BEFORE ROS init to ensure it takes precedence
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

#ifdef ROS2
    try {
        ROS_INIT(argc, argv, "pointcloud_to_laserscan");
        node_ = CREATE_NODE("pointcloud_to_laserscan");

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        auto pointcloud_sub = CREATE_SUBSCRIBER(node_, sensor_msgs::msg::PointCloud2,
                                                pointcloud_topic_, 1, PointcloudCallback);
        scan_publisher_ = CREATE_PUBLISHER(node_, sensor_msgs::msg::LaserScan,
                                           laser_topic_, 1);
        debug_pc_publisher_ = CREATE_PUBLISHER(node_, sensor_msgs::msg::PointCloud2,
                                               debug_pointcloud_topic_, 1);

        // Spin until shutdown
        rclcpp::spin(node_);
    } catch (const std::exception& e) {
        std::cerr << "Exception in ROS2 node: " << e.what() << std::endl;
    }
    
    printf("Shutting down gracefully...\n");
    
    // CRITICAL: Set shutdown flag immediately to stop callbacks
    shutdown_requested.store(true);
    
    // Give any in-flight callbacks a moment to finish
    printf("  [1/4] Waiting for callbacks to finish...\n");
    usleep(150000);  // 150ms
    
    // Uninstall signal handlers before cleanup
    printf("  [2/4] Uninstalling signal handlers...\n");
    signal(SIGINT, SIG_DFL);
    signal(SIGTERM, SIG_DFL);
    
    // Properly destroy ROS2 objects before shutdown
    printf("  [3/4] Destroying ROS2 publisher and node...\n");
    scan_publisher_.reset();
    debug_pc_publisher_.reset();
    node_.reset();
    
    printf("  [4/4] Shutting down ROS2...\n");
    if (rclcpp::ok()) {
        ROS_SHUTDOWN();
    }
    
    printf("Cleanup complete.\n");
#else
    ROS_INIT(argc, argv, "pointcloud_to_laserscan");
    node_ = CREATE_NODE("pointcloud_to_laserscan");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    ros::Subscriber pointcloud_sub = CREATE_SUBSCRIBER(node_, sensor_msgs::PointCloud2, pointcloud_topic_, 1, &PointcloudCallback);
    scan_publisher_ = CREATE_PUBLISHER(node_, sensor_msgs::LaserScan, laser_topic_, 1);
    debug_pc_publisher_ = CREATE_PUBLISHER(node_, sensor_msgs::PointCloud2, debug_pointcloud_topic_, 1);

    // Spin until shutdown
    ros::spin();
    
    printf("Shutting down gracefully...\n");
    
    // Set shutdown flag immediately to stop callbacks
    shutdown_requested.store(true);
    
    // Give any in-flight callbacks a moment to finish
    usleep(150000);  // 150ms
    
    // Uninstall signal handlers before cleanup
    signal(SIGINT, SIG_DFL);
    signal(SIGTERM, SIG_DFL);
    
    // Clean shutdown
    scan_publisher_.reset();
    debug_pc_publisher_.reset();
    node_.reset();
    ROS_SHUTDOWN();
    
    printf("Cleanup complete.\n");
#endif

    return 0;
}
