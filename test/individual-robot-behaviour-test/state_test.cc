/* state_test.cc
 *==============================================================================
 * Author: Carl Larsson
 * Creation date: 2024-09-23
 * Last modified: 2024-10-08 by Carl Larsson
 * Description: Test file for state source and header files.
 * License: See LICENSE file for license details.
 *==============================================================================
 */


/* Related .h files */
#include "../../src/individual-robot-behaviour/state.h"

/* C++ standard library headers */
#include <cmath>

/* Other .h files */
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

/* Project .h files */
#include "../../src/common_types.h"


/*============================================================================*/
/* Tests for Pose class */

/*----------------------------------------------------------------------------*/

/* Test fixture for Pose class */
class PoseClassTest : public ::testing::Test {
 protected:
  /* This will run before each test (TEST_F) */
  void SetUp() override {
    /* Create new */
    robot_controller_interface::individual_robot_behaviour::Pose pose_;
  }

  /* Test object */
  robot_controller_interface::individual_robot_behaviour::Pose pose_;
};

/*----------------------------------------------------------------------------*/

/* 
 * Test case for Pose class, initial values (assuming default constructor was 
 * used) and GetX,GetY,GetTheta
 */
TEST_F(PoseClassTest, DefaultConstructor)
{
  double default_value = 0.0;


  EXPECT_FLOAT_EQ(pose_.GetX(), default_value);

  EXPECT_FLOAT_EQ(pose_.GetY(), default_value);

  EXPECT_FLOAT_EQ(pose_.GetTheta(), default_value);
}

/* 
 * Test case for Pose class, initial values (assuming parameterized
 * constructor was used)
 */
TEST_F(PoseClassTest, ParameterizedConstructor)
{
  double x = 3.3;
  double y = 5.0;
  double theta = 1.11;
  robot_controller_interface::individual_robot_behaviour::Pose 
      param_pose_(x, y, theta);


  EXPECT_FLOAT_EQ(param_pose_.GetX(), x);

  EXPECT_FLOAT_EQ(param_pose_.GetY(), y);

  EXPECT_FLOAT_EQ(param_pose_.GetTheta(), theta);
}

/* Test case for Pose class, setting values inside boundary */
TEST_F(PoseClassTest, SettingValues)
{
  double x = 2.0;
  double y = 5.2;
  double theta = 0.5;


  pose_.SetX(x);
  EXPECT_FLOAT_EQ(pose_.GetX(), x);

  pose_.SetY(y);
  EXPECT_FLOAT_EQ(pose_.GetY(), y);

  pose_.SetTheta(theta);
  EXPECT_FLOAT_EQ(pose_.GetTheta(), theta);
}

/* Test case for Pose class, setting values outside boundary */
TEST_F(PoseClassTest, BoundaryValues)
{
  /* Positive */
  double x = robot_controller_interface::PlayingField::kFrameX/2;
  pose_.SetX(x + 10);
  EXPECT_FLOAT_EQ(pose_.GetX(), x);

  double y = robot_controller_interface::PlayingField::kFrameY/2;
  pose_.SetY(y + 10);
  EXPECT_FLOAT_EQ(pose_.GetY(), y);

  /* Negative */
  x = -(robot_controller_interface::PlayingField::kFrameX/2);
  pose_.SetX(x - 10);
  EXPECT_FLOAT_EQ(pose_.GetX(), x);

  y = -(robot_controller_interface::PlayingField::kFrameY/2);
  pose_.SetY(y - 10);
  EXPECT_FLOAT_EQ(pose_.GetY(), y);

  /* Wrap */
  double theta = 24.6;
  pose_.SetTheta(theta);
  EXPECT_FLOAT_EQ(pose_.GetTheta(), atan2(sin(theta), cos(theta)));
}

/* 
 * Test case for Pose class inequality operator
 * Doesn't have to be TEST_F but I prefer all grouped together
 */
TEST_F(PoseClassTest, InequalityOperator)
{
  double val = 1.0;

  robot_controller_interface::individual_robot_behaviour::Pose a;
  a.SetX(val);
  a.SetY(val);
  a.SetTheta(val);

  robot_controller_interface::individual_robot_behaviour::Pose b;
  b.SetX(val);
  b.SetY(val);
  b.SetTheta(val);

  
  /* Exactly equal */
  EXPECT_FALSE(a != b);

  /* Difference exactly equal to tolerance */
  a.SetX(1.0 + 1e-6);
  EXPECT_FALSE(a != b);

  /* Difference slightly outside tolerance */
  a.SetX(1.0 + 1e-5);
  EXPECT_TRUE(a != b);

  /* Difference within tolerance */
  a.SetX(1.0 + 1e-7);
  EXPECT_FALSE(a != b);
}

/*============================================================================*/
/* Tests for RobotState class */

/*----------------------------------------------------------------------------*/

/* Test fixture for RobotState class */
class RobotStateClassTest : public ::testing::Test {
 protected:
  /* This will run before each test (TEST_F) */
  void SetUp() override {
    /* Create new */
    robot_controller_interface::individual_robot_behaviour::RobotState 
        robot_state_;
  }

  /* Test object */
  robot_controller_interface::individual_robot_behaviour::RobotState 
      robot_state_;
};

/*----------------------------------------------------------------------------*/

/*
 * Test case for RobotState class, initial values (assuming default constructor 
 * was used) and GetX,GetY,GetTheta,GetBall
 */
TEST_F(RobotStateClassTest, DefaultConstructor)
{
  double default_value = 0.0;


  EXPECT_FLOAT_EQ(robot_state_.GetX(), default_value);

  EXPECT_FLOAT_EQ(robot_state_.GetY(), default_value);

  EXPECT_FLOAT_EQ(robot_state_.GetTheta(), default_value);

  EXPECT_FALSE(robot_state_.GetBall());
}

/*
 * Test case for RobotState class, initial values (assuming parameterized
 * constructor was used)
 */
TEST_F(RobotStateClassTest, ParameterizedConstructor)
{
  double x = 3.3;
  double y = 5.0;
  double theta = 1.11;
  bool ball = true;
  robot_controller_interface::individual_robot_behaviour::RobotState 
      param_robot_state_(x, y, theta, ball);


  EXPECT_FLOAT_EQ(param_robot_state_.GetX(), x);

  EXPECT_FLOAT_EQ(param_robot_state_.GetY(), y);

  EXPECT_FLOAT_EQ(param_robot_state_.GetTheta(), theta);

  EXPECT_TRUE(param_robot_state_.GetBall());
}

/* Test case for Pose class, setting values inside boundary */
TEST_F(RobotStateClassTest, SettingValues)
{
  double x = 2.0;
  double y = 5.2;
  double theta = 0.5;
  bool ball = true;


  robot_state_.SetX(x);
  EXPECT_FLOAT_EQ(robot_state_.GetX(), x);

  robot_state_.SetY(y);
  EXPECT_FLOAT_EQ(robot_state_.GetY(), y);

  robot_state_.SetTheta(theta);
  EXPECT_FLOAT_EQ(robot_state_.GetTheta(), theta);

  robot_state_.SetBall(ball);
  EXPECT_TRUE(robot_state_.GetBall());
}

/*============================================================================*/
/* Tests for OdomSubscriber */

/*----------------------------------------------------------------------------*/

/* Test fixture for OdomSubscriber class test */
class OdomSubscriberTest : public ::testing::Test
{
 protected:
  /* Will be done before each test */
  void SetUp() override
  {
    /* Initialize ROS2 */
    rclcpp::init(0, nullptr);
        
    /* Create OdomSubscriber node */
    odom_subscriber_ = std::make_shared<robot_controller_interface
        ::individual_robot_behaviour::OdomSubscriber>();
        
    /*
     * Create a publisher to publish Odometry messages (since OdomCallback is 
     * private so a mock can't be sent directly)
     */
    publisher_ = odom_subscriber_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        
    /* Create a mock Odometry message */
    odom_msg_ = std::make_shared<nav_msgs::msg::Odometry>();
    /* Modify the odom_msg_ fields */
    odom_msg_->pose.pose.position.x = 1.0;
    odom_msg_->pose.pose.position.y = 2.0;
    odom_msg_->pose.pose.position.z = 3.0;
    odom_msg_->pose.pose.orientation.w = 1.0;
    odom_msg_->pose.pose.orientation.w = 2.0;
    odom_msg_->pose.pose.orientation.w = 3.0;
    odom_msg_->pose.pose.orientation.w = 4.0;
    /* 
     * We have to transform orientation to yaw since that is how we store it in 
     * current_state, so its the only way to check
     */
    theta_ = tf2::getYaw(odom_msg_->pose.pose.orientation);

    /* Clear global variable between each test */
    robot_controller_interface::individual_robot_behaviour::current_state.SetX(0.0);
    robot_controller_interface::individual_robot_behaviour::current_state.SetY(0.0);
    robot_controller_interface::individual_robot_behaviour::current_state.SetTheta(0.0);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  std::shared_ptr<robot_controller_interface::individual_robot_behaviour
      ::OdomSubscriber> odom_subscriber_;
  nav_msgs::msg::Odometry::SharedPtr odom_msg_;
  double theta_;
};

/*----------------------------------------------------------------------------*/

TEST_F(OdomSubscriberTest, CallbackMockMessage)
{
  /* Publish the test odometry message */
  publisher_->publish(*odom_msg_);

  /* Spin the node so the subscription can process the message */
  rclcpp::spin_some(odom_subscriber_);

  EXPECT_FLOAT_EQ(robot_controller_interface::individual_robot_behaviour::current_state.GetX(), 1.0);
  EXPECT_FLOAT_EQ(robot_controller_interface::individual_robot_behaviour::current_state.GetY(), 2.0);
  EXPECT_FLOAT_EQ(robot_controller_interface::individual_robot_behaviour::current_state.GetTheta(), theta_);
}

/*============================================================================*/
/* Tests for CalculateAngle */
