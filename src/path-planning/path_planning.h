// path_planning.h
//==============================================================================
// Author: Carl Larsson
// Creation date: 2024-09-19
// Last modified: 2024-09-20 by Carl Larsson
// Description: Path planning header file.
// License: See LICENSE file for license details.
//==============================================================================


#ifndef ROBOTCONTROLLERINTERFACE_PATHPLANNING_PATHPLANNING_H_
#define ROBOTCONTROLLERINTERFACE_PATHPLANNING_PATHPLANNING_H_


// Related .h files

// C++ standard library headers
#include <functional>
#include <memory>

// Other .h files
#include "rclcpp/rclcpp.hpp"
#include "dwb_core/dwb_local_planner.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Project .h files
#include "../common_types.h"


namespace robot_controller_interface
{
namespace path_planning
{

// Struct for a location in 2D space.
struct Position
{
    int x;
    int y;
    double theta;
};


// DWB ROS2 node, inherit from LifecycleNode
class DwbNode : public nav2_util::LifecycleNode
{
public:
    // Initilizations
    DwbNode() : nav2_util::LifecycleNode("dwb_controller_node"),
                tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
    {
        // Subscribe to odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SystemDefaultsQoS(),
            std::bind(&DwbNode::OdometryCallbackFunction, this, std::placeholders::_1));

        // Path publisher, can publish only one point, target position
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_plan", 10);

        // Publish velocity on cmd_vel
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

//protected:
    void OdometryCallbackFunction(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::Pose current_pose = msg->pose.pose;
        //dwb_controller_->setRobotPose(current_pose);
    }

    // Publish a single target position
    void publish_single_goal(const geometry_msgs::msg::PoseStamped &goal_pose)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "odom"; // Don't forget to change

        // Add the goal pose to the path
        path_msg.poses.push_back(goal_pose);

        // Publish the path
        path_pub_->publish(path_msg);
    }

    // The functions bellow are required by LifecycleNode
    // They periodicly publish the control velocities.
    void ControlLoop()
    {
        geometry_msgs::msg::Twist cmd_vel;
        try
        {
            //cmd_vel = dwb_controller_->computeVelocityCommands();
            //cmd_vel_pub_->publish(cmd_vel);
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute velocity: %s", e.what());
        }
    }

    // Start the periodic control loop
    void StartControlLoop()
    {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DwbNode::ControlLoop, this));
    }

private:
    // Definitions
    std::shared_ptr<dwb_core::DWBLocalPlanner> dwb_controller_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};


} // namespace path_planning
} // namespace robot_controller_interface

#endif // ROBOTCONTROLLERINTERFACE_PATHPLANNING_PATHPLANNING_H_
