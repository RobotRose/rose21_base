/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/01/02
* 		- File created.
*
* Description:
*	The drive_controller_node
* 
***********************************************************************************/

#include "rose21_drive_controller/drive_controller_node.hpp"

/**\defgroup drive_controller_node Drive controller node
 * \relates DriveController
 * Main function of the drive_controller node. Create a DriveController object and calls spinning code.
 * Use keys for simple manual control:
 *  \arg \b + Increase linear y-velocity
 *  \arg \b - Decrease linear y-velocity 
 *  \arg \b * Increase w_velocity
 *  \arg \b / Decrease w_velocity 
 *  \arg \b s Stop all movement
 *  \arg \b x Stop all movement and quit
 * @param[in] Command line argurments
 * @return bool Exit state 
 */
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "drive_controller");
	ros::NodeHandle n;
	ros::NodeHandle n_p("~");
	ros::Rate r(20);

	ROS_INFO_NAMED(ROS_NAME, "Drive_controller node started");

	DriveController* drive_controller = new DriveController("drive_controller", n);

	bool test = false;
	n_p.getParam("test", test);

	float x_velocity 	= 0.0;
	float y_velocity 	= 0.0;
	float w_velocity 	= 0.0;
	float r_velocity 	= 0.0;
	float lin_vel_step 	= 0.025;
	float w_vel_step 	= 0.05;
	bool stop 			= false;
	float radius		= 0.0;
	float radius_step 	= 0.1;
	while(n.ok() && !stop)
	{
		if(rose_conversions::kbhit() and test)
		{
		    uint c = getchar();
		    ROS_INFO_NAMED(ROS_NAME, "Key pressed: %c", (char)c);
		    switch(c)
		    {
		    	case '+':	
		    		x_velocity += lin_vel_step;    		
					break;
				case '-':	
		    		x_velocity -= lin_vel_step;    		
					break;
				case ']':	
		    		y_velocity += lin_vel_step;    		
					break;
				case '[':	
		    		y_velocity -= lin_vel_step;    		
					break;
				case '*':
					w_velocity += w_vel_step;    		
					break;
				case '/':
					w_velocity -= w_vel_step;    		
					break;
				case 's':
					y_velocity 	= 0.0;
					x_velocity 	= 0.0;
					w_velocity 	= 0.0;
					break;
				case 't':
					y_velocity 	= 0.00;
					x_velocity 	= 0.10;
					w_velocity 	= -(x_velocity/0.7);
					break;
				case 'x':
					y_velocity 	= 0.0;
					x_velocity 	= 0.0;
					w_velocity 	= 0.0;
					stop 		= true;
					break;
		    }
		    drive_controller->executeMovement(x_velocity, y_velocity, w_velocity);
		}

		ros::spinOnce();
		r.sleep();
	}

	delete drive_controller;

	return 0;
}
