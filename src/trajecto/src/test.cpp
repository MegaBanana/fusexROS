
#include "robot_localization/SetPose.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <gtest/gtest.h>

#include <iostream>


void method()
{
   ros::NodeHandle nh;
   ros::Publisher imu0Pub = nh.advertise<sensor_msgs::Imu>("/  imu_input0", 5);

   sensor_msgs::Imu imu;
   imu.header.frame_id ="base_link";
   tf2::Quaternion quat;
   const double roll = M_PI/2.0;
   const double pitch = -M_PI;
   const double yaw = -M_PI/4.0;
   quat.setRPY(roll, pitch, yaw);
   imu.orientation = tf2::toMsg(quat); 
   
   imu.orientation_covariance[0] = 0.02;
   imu.orientation_covariance[4] = 0.02;
   imu.orientation_covariance[8] = 0.02;

   imu.angular_velocity_covariance[0] = 0.02;
   imu.angular_velocity_covariance[4] = 0.02;
   imu.angular_velocity_covariance[8] = 0.02;





}



int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "ukf_navigation_node-test-interfaces");
  ros::Time::init();

  // Give ukf_localization_node time to initialize
  ros::Duration(2.0).sleep();

  return RUN_ALL_TESTS();
}

