#pragma once

// ROS1/ROS2 compatibility layer for pointcloud_to_laserscan

#ifdef ROS2
#include <rclcpp/rclcpp.hpp>
#include <memory>

// Logger
#define PCL2LASER_LOGGER rclcpp::get_logger("pointcloud_to_laserscan")

#define LOG_ERROR(...) RCLCPP_ERROR(PCL2LASER_LOGGER, __VA_ARGS__)
#define LOG_WARN(...) RCLCPP_WARN(PCL2LASER_LOGGER, __VA_ARGS__)
#define LOG_INFO(...) RCLCPP_INFO(PCL2LASER_LOGGER, __VA_ARGS__)
#define GET_TIME() rclcpp::Clock().now()

// Node abstractions
using NodePtr = std::shared_ptr<rclcpp::Node>;

// Publisher/Subscriber type abstractions
template <typename MessageType>
using PublisherPtr = typename rclcpp::Publisher<MessageType>::SharedPtr;

template <typename MessageType>
using SubscriberPtr = typename rclcpp::Subscription<MessageType>::SharedPtr;

// Time abstractions
using Time = rclcpp::Time;

// ROS system macros
#define ROS_OK() rclcpp::ok()
#define ROS_SPIN() rclcpp::spin(node_)
#define ROS_INIT(argc, argv, name) rclcpp::init(argc, argv)
#define ROS_SHUTDOWN() rclcpp::shutdown()

// Node creation macro
#define CREATE_NODE(name) rclcpp::Node::make_shared(name)

// Publisher creation macro
#define CREATE_PUBLISHER(node, msg_type, topic, queue_size) \
    node->create_publisher<msg_type>(topic, queue_size)

// Subscriber creation macro
#define CREATE_SUBSCRIBER(node, msg_type, topic, queue_size, callback) \
    node->create_subscription<msg_type>(topic, queue_size, callback)

#else
#include <ros/ros.h>

#define LOG_ERROR(...) ROS_ERROR(__VA_ARGS__)
#define LOG_WARN(...) ROS_WARN(__VA_ARGS__)
#define LOG_INFO(...) ROS_INFO(__VA_ARGS__)
#define GET_TIME() ros::Time::now()

// Node abstractions
using NodePtr = std::shared_ptr<ros::NodeHandle>;

// Publisher/Subscriber type abstractions
template <typename MessageType>
using PublisherPtr = ros::Publisher;

template <typename MessageType>
using SubscriberPtr = ros::Subscriber;

// Time abstractions
using Time = ros::Time;

// ROS system macros
#define ROS_OK() ros::ok()
#define ROS_SPIN() ros::spin()
#define ROS_INIT(argc, argv, name) ros::init(argc, argv, name)
#define ROS_SHUTDOWN() ros::shutdown()

// Node creation macro
#define CREATE_NODE(name) std::make_shared<ros::NodeHandle>()

// Publisher creation macro
#define CREATE_PUBLISHER(node, msg_type, topic, queue_size) \
    node->advertise<msg_type>(topic, queue_size)

// Subscriber creation macro
#define CREATE_SUBSCRIBER(node, msg_type, topic, queue_size, callback) \
    node->subscribe(topic, queue_size, callback)

#endif
