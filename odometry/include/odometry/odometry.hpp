/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/03/17
* 		- File created.
*
* Description:
*	Class to calculate the odometry of the robot given the state of the wheelunits
* 
***********************************************************************************/

#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <map>

#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

#include "opteq_wheelunits_01/wheel_unit.hpp"

#include "rose_conversions/conversions.hpp"

#include "ros_name/ros_name.hpp"

#define ROS_NAME_ODOM (ROS_NAME + "|ODOM")

// These defines should match those in drive_controller
#define PLATFORM_WHEELBASE_LENGTH   0.575 //0.58 // [m] The heart-heart distance of the wheels in the y direction
#define PLATFORM_WHEELBASE_WIDTH    0.474 //0.48 // [m] The heart-heart distance of the wheels in the x direction.
#define MAX_RADIUS 					10000 // [m]

using namespace std;

class Odometry
{
  public:
  	Odometry();
  	~Odometry();

  	bool calculateOdometry(map<string, WheelUnit>& wheelunits_map, geometry_msgs::Pose& pose, geometry_msgs::Twist& velocity);

  private:
  	ros::Time          		prev_time_;
  	bool               	    first_iteration_;

  	vector<float> 			velocity_maf_;
};



#endif // ODOMETRY_HPP