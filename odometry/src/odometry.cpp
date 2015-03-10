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

#include "odometry/odometry.hpp"

Odometry::Odometry()
	: first_iteration_(true)
{}

Odometry::~Odometry()
{}

// Called if new odometry data is received via callback
// Assumes wheelunits are ordered in the following sequence: FrontLeft, FrontRight, BackRight, BackLeft
// In the future use: http://wiki.ros.org/robot_pose_ekf ? Does not yet implement muliple sensor sources
// FOR DETAILS see the odometry calculation of minicourse 1 page 9 on the wiki
//! @todo OH: Upload minicourses to wiki
bool Odometry::calculateOdometry(map<string, WheelUnit>& wheelunits_map, geometry_msgs::Pose& pose, geometry_msgs::Twist& velocity)
{
	if(first_iteration_)
	{
		prev_time_ 			= ros::Time::now();
		first_iteration_ 	= false;
		return false;
	}

	if( wheelunits_map.count("FR") == 0 ||
		wheelunits_map.count("FL") == 0 ||
		wheelunits_map.count("BR") == 0 ||
		wheelunits_map.count("BL") == 0)
	{
		ROS_WARN_THROTTLE_NAMED(1, ROS_NAME_ODOM, "Supplied map must contain an FR, FL, BR and BL entry.");
		return false;
	}

	// Calculate the sample time 
	ros::Duration duration(ros::Time::now() - prev_time_);
	// float dT 	= duration.toSec();
	float dT 	= wheelunits_map.at("FR").getDT();

	if(dT == 0.0)
	{
		ROS_WARN_NAMED(ROS_NAME_ODOM, "Warning dT == 0.0");
		return false;
	}
	
	prev_time_ 	= ros::Time::now();

	ROS_DEBUG_NAMED(ROS_NAME_ODOM, "dT: %.4f", dT);

	float yaw = tf::getYaw(pose.orientation);

	// Calculate these values for easier reading
	float half_length 	= PLATFORM_WHEELBASE_LENGTH/2.0;
	float half_width	= PLATFORM_WHEELBASE_WIDTH/2.0;

	//! @todo OH: Does this angle depend on the driving direction? Does it get flipped with the velocity sign?
	// Calculate the average angle on both sides
	// alpha_avg is the overal average angle   
	float alpha_avg_l		= (wheelunits_map.at("FL").getMeasuredAngleRad() + wheelunits_map.at("BL").getMeasuredAngleRad())/2.0;
	float alpha_avg_r		= (wheelunits_map.at("FR").getMeasuredAngleRad() + wheelunits_map.at("BR").getMeasuredAngleRad())/2.0;
	float alpha_avg 		= (alpha_avg_l + alpha_avg_r) / 2.0;

	ROS_DEBUG_NAMED(ROS_NAME_ODOM, "alpha_avg_l: %.4f, alpha_avg_r: %.4f, |difference|: %.4f", alpha_avg_l, alpha_avg_r, fabs(alpha_avg_l - alpha_avg_r));

	// Calculate the average velocity of the front and the back side
	// Take the average speed of the front and the back sides to get the velocity in the middle, y-axis, of the robot
	float velocity_l 		= (wheelunits_map.at("FL").getMeasuredVelocityMetersPerSec() + wheelunits_map.at("BL").getMeasuredVelocityMetersPerSec()) / 2.0;
	float velocity_r 		= (wheelunits_map.at("FR").getMeasuredVelocityMetersPerSec() + wheelunits_map.at("BR").getMeasuredVelocityMetersPerSec()) / 2.0;
	float velocity_avg		= (velocity_l + velocity_r) / 2.0;

	float dist_diff_l 		= (wheelunits_map.at("FL").getMeasuredDiffMeters() + wheelunits_map.at("BL").getMeasuredDiffMeters()) / 2.0;
	float dist_diff_r 		= (wheelunits_map.at("FR").getMeasuredDiffMeters() + wheelunits_map.at("BR").getMeasuredDiffMeters()) / 2.0;
	

	// When wheels are not rotating in the same direction (point turn)
	if(rose_conversions::sgn(dist_diff_l) != rose_conversions::sgn(dist_diff_r)) 
	{
		// Flip sign left distance to have same sign as rotation distance( CW = neg, CCW = pos)
		//dist_diff_l 	= -dist_diff_l;
		ROS_DEBUG_NAMED(ROS_NAME_ODOM, "Sign flip!");
	}
	  
	float dist_diff_avg		= (dist_diff_l + dist_diff_r) / 2.0;
	

	ROS_DEBUG_NAMED(ROS_NAME_ODOM, "Platform_TF, measured, rot[%.3f, %.3f, %.3f, %.3f]rad -> alpha_avg %.3frad",
					wheelunits_map.at("FR").getMeasuredAngleRad(),
					wheelunits_map.at("FL").getMeasuredAngleRad(),
					wheelunits_map.at("BR").getMeasuredAngleRad(),
					wheelunits_map.at("BL").getMeasuredAngleRad(),
					alpha_avg);
	ROS_DEBUG_NAMED(ROS_NAME_ODOM, "Platform_TF, measured, vel[%.3f, %.3f, %.3f, %.3f]m/s -> %.3fm/s",
					wheelunits_map.at("FR").getMeasuredVelocityMetersPerSec(),
					wheelunits_map.at("FL").getMeasuredVelocityMetersPerSec(),
					wheelunits_map.at("BR").getMeasuredVelocityMetersPerSec(),
					wheelunits_map.at("BL").getMeasuredVelocityMetersPerSec(),
					velocity_avg);
	ROS_DEBUG_NAMED(ROS_NAME_ODOM, "Platform_TF, measured, vel[%.3f, %.3f, %.3f, %.3f]rad/s",
					wheelunits_map.at("FR").getMeasuredVelocityRadPerSec(),
					wheelunits_map.at("FL").getMeasuredVelocityRadPerSec(),
					wheelunits_map.at("BR").getMeasuredVelocityRadPerSec(),
					wheelunits_map.at("BL").getMeasuredVelocityRadPerSec());
	ROS_DEBUG_NAMED(ROS_NAME_ODOM, "Platform_TF, measured, vel[%d, %d, %d, %d]pulses/s",
					wheelunits_map.at("FR").getMeasuredVelocityLowLevel(),
					wheelunits_map.at("FL").getMeasuredVelocityLowLevel(),
					wheelunits_map.at("BR").getMeasuredVelocityLowLevel(),
					wheelunits_map.at("BL").getMeasuredVelocityLowLevel());
	ROS_DEBUG_NAMED(ROS_NAME_ODOM, "Platform_TF, measured, dist_diff[%.5f, %.5f, %.5f, %.5f]m -> %.5fm, %.4fs | dist_diff vel[%.5f, %.5f, %.5f, %.5f]m/s -> %.5fm/s",
					wheelunits_map.at("FR").getMeasuredDiffMeters(),
					wheelunits_map.at("FL").getMeasuredDiffMeters(),
					wheelunits_map.at("BR").getMeasuredDiffMeters(),
					wheelunits_map.at("BL").getMeasuredDiffMeters(),
					(dist_diff_avg), 
					dT,
					wheelunits_map.at("FR").getMeasuredDiffMeters()/dT,
					wheelunits_map.at("FL").getMeasuredDiffMeters()/dT,
					wheelunits_map.at("BR").getMeasuredDiffMeters()/dT,
					wheelunits_map.at("BL").getMeasuredDiffMeters()/dT,					
					(dist_diff_avg)/dT);

	// Check if the robot is strafing
	//! @todo OH: Check if we really need a certain small noise range here?
	ROS_DEBUG_NAMED(ROS_NAME_ODOM, "fabs(alpha_avg_l - alpha_avg_r) = %.5f", fabs(alpha_avg_l - alpha_avg_r));


	// Calculate variation coeff
	float average = wheelunits_map.at("FL").getMeasuredAngleRad() + wheelunits_map.at("BL").getMeasuredAngleRad() + wheelunits_map.at("FR").getMeasuredAngleRad() + wheelunits_map.at("BR").getMeasuredAngleRad();
	average /= 4.0;

	float sum2 =   pow(wheelunits_map.at("FL").getMeasuredAngleRad() - average, 2.0)
				 + pow(wheelunits_map.at("BL").getMeasuredAngleRad() - average, 2.0)
				 + pow(wheelunits_map.at("FR").getMeasuredAngleRad() - average, 2.0)
				 + pow(wheelunits_map.at("BR").getMeasuredAngleRad() - average, 2.0);

	sum2 /= 4.0;
	sum2 = sqrt(sum2);

	float variation_coeff = sum2; 	// omitted '/average' in order to prevent div by zero

	if(variation_coeff <= 0.03) 
	{	
		// Create a travled distance vector
		float dist_vector_x = dist_diff_avg;
		float dist_vector_y = 0.0;

		// Determine the strafe angle
		float strafe_angle 	= (	wheelunits_map.at("FL").getMeasuredAngleRad() + 
								wheelunits_map.at("BL").getMeasuredAngleRad() + 
								wheelunits_map.at("FR").getMeasuredAngleRad() + 
								wheelunits_map.at("BR").getMeasuredAngleRad())/4.0;

		// Rotate the vector by the strafe_angle
		rose_geometry::rotateVect(&dist_vector_x, &dist_vector_y, -strafe_angle + yaw);

		// Calculate the velocity
		float vel_vector_x = dist_vector_x/dT;
		float vel_vector_y = dist_vector_y/dT;

		ROS_DEBUG_NAMED(ROS_NAME_ODOM, "Strafing, dT: %.3f, strafe_angle: %.3f, vel. avg.: %.3f, vel_vector_xy: %.3f, %.3f dist_vector_xy: %.3f, %.3f", dT, strafe_angle, velocity_avg, vel_vector_x, vel_vector_y, dist_vector_x, dist_vector_y);

		// Calculate the new position, rotation does not change
		pose.position.x 	+= dist_vector_x;
		pose.position.y		+= dist_vector_y;
		yaw 				+= 0.0;

		// Set the current velocity, no rotation speed
		velocity.linear.x 	= vel_vector_x;
		velocity.linear.y  	= vel_vector_y;
		velocity.angular.z 	= 0.0;
	}
	// The robot is not strafing but is moving in a circle around a certain point M with a certain radius 
	else
	{	
		// Front and back need to flip sign
		alpha_avg_l		= (wheelunits_map.at("FL").getMeasuredAngleRad() - wheelunits_map.at("BL").getMeasuredAngleRad())/2.0;
		alpha_avg_r		= (wheelunits_map.at("FR").getMeasuredAngleRad() - wheelunits_map.at("BR").getMeasuredAngleRad())/2.0;
		alpha_avg 		= (alpha_avg_l + alpha_avg_r) / 2.0;

		// Calculate the average radius
		float radius_l = 0.0;
		float radius_r = 0.0;
		
		if(0.5*M_PI-alpha_avg_l == 0.5*M_PI or 0.5*M_PI-alpha_avg_l == -0.5*M_PI)
			radius_l 			= MAX_RADIUS;
		else
			radius_l			= -(-half_width + tan(0.5*M_PI-alpha_avg_l)*half_length);
		
		if(0.5*M_PI-alpha_avg_r == 0.5*M_PI or 0.5*M_PI-alpha_avg_r == -0.5*M_PI)
			radius_r 			= MAX_RADIUS;
		else
			radius_r			= -(half_width + tan(0.5*M_PI-alpha_avg_r)*half_length);

		float radius_avg		= (radius_l + radius_r)/2.0;

		// ROS_INFO_NAMED(ROS_NAME_ODOM, "radius_l: %.5f, radius_r: %.5f, radius_avg: %.5f", radius_l, radius_r, radius_avg);

		// Calculate the radius of the right (1) and the left (2) wheels
		float rr = sqrt(pow(radius_r + half_width, 2.0) + pow(half_length, 2.0));
		float rl = sqrt(pow(radius_l - half_width, 2.0) + pow(half_length, 2.0));

		if(alpha_avg_r >= 0.0)
			rr = -rr;

		if(alpha_avg_l >= 0.0)
			rl = -rl;

		// Calculate the angular rotation of the right and left side of the robot
		float Br = dist_diff_r/rr;
		float Bl = dist_diff_l/rl;

		// The rotation around the point which we are rotating around is the average of B1 and B2
		float rotation_r = (Br);   
		float rotation_l = (Bl);   

		float velocity_w_avg_r = rotation_r/dT;
		float velocity_w_avg_l = rotation_l/dT;

		// ROS_INFO_NAMED(ROS_NAME_ODOM, "rr: %.3f, rl: %.3f, Br: %.3f, Bl: %.3f, rotation_r: %.3f, , rotation_l: %.3f", rr, rl, Br, Bl, rotation_r, rotation_l);
		
		ROS_DEBUG_NAMED(ROS_NAME_ODOM, "Circular motion, radius: %.3f, rotation_r: %.2f, rotation_l: %.2f, rotation speed _r: %.3f", radius_avg, rotation_r, rotation_l, velocity_w_avg_r);

		// Calculate the new odometry position, see wiki!
		float distance_PQ_r	= 2.0*radius_r*sin(rotation_r/2.0);
		float distance_PQ_l	= 2.0*radius_l*sin(rotation_l/2.0);


		float prev_x 	= pose.position.x;
		float prev_y 	= pose.position.y;
		float prev_th 	= yaw;

		// Calculate the new position
		float x_dist 		 = (distance_PQ_r*cos(yaw + rotation_r) + distance_PQ_l*cos(yaw + rotation_l)) / 2.0;
		float y_dist 		 = (distance_PQ_r*sin(yaw + rotation_r) + distance_PQ_l*sin(yaw + rotation_l)) / 2.0;
		pose.position.x 	+= x_dist;
		pose.position.y 	+= y_dist;
		yaw 				+= (rotation_r + rotation_l) / 2.0;

		// Calculate the current velocity
		velocity.linear.x 	= (pose.position.x  - prev_x )/dT;
		velocity.linear.y  	= (pose.position.y  - prev_y )/dT;
		velocity.angular.z 	= (yaw 				- prev_th)/dT;

		
		
		// ROS_WARN_NAMED(ROS_NAME_ODOM, "distance_PQ_r: %.6f, distance_PQ_l: %.6f", distance_PQ_r, distance_PQ_l);
		//ROS_INFO_NAMED(ROS_NAME_ODOM, "x_dist: %.5f, velocity.linear.x = %.6f, pos x: %.6f, prev pos x: %.6f, dT: %.4f", x_dist, velocity.linear.x, pose.position.x, prev_x, dT);
		//ROS_INFO_NAMED(ROS_NAME_ODOM, "y_dist: %.5f, velocity.linear.y = %.6f, pos y: %.6f, prev pos y: %.6f, dT: %.4f", y_dist, velocity.linear.y, pose.position.y, prev_y, dT);
	}

	// Transform velocity from 'odom' frame to local frame (usually 'base_link')
	rose_geometry::rotateVect(&velocity.linear.x, &velocity.linear.y, -yaw);

	// Write yaw back to quaternion form
	pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw);


	return true;
}
