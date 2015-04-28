/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
* Author: Okke Hendriks
* Date  : 2014/01/02
*     - File created.
*
* Description:
* This controller receives high level velocity and direction commands.
*   These will be translated into rotation angles and velocities for the 
*   wheel_controller.
* 
***********************************************************************************/

#ifndef DRIVE_CONTROLLER_HPP
#define DRIVE_CONTROLLER_HPP

#include <iostream>
#include <stdio.h>

#include <map>

#include <ros/ros.h>

#include "rose_common/common.hpp"
#include "rose_geometry/geometry.hpp"
#include "rose_conversions/conversions.hpp"
#include "rose_transformations/transformations.hpp"

#include "server_multiple_client/server_multiple_client.hpp"
#include "tf_helper/tf_helper.hpp"
#include "operator_messaging/operator_messaging.hpp"

#include "rose_shared_variables/shared_variable.hpp"

#include "action_result_message.hpp"

#include "rose_footprint_collision_checker/footprint_collision_checker.hpp"

#include <geometry_msgs/Twist.h> 
#include <sensor_msgs/LaserScan.h> 

#include "contact_sensor_msgs/bumper.h"
#include "contact_sensor_msgs/bumpers.h"
#include "roscomm/conversion_bool.hpp"

#include "rose_base_msgs/cmd_velocityAction.h"
#include "rose_base_msgs/cmd_velocityActionGoal.h"
#include "rose_base_msgs/cmd_velocityActionResult.h"
#include "rose_base_msgs/cmd_velocityActionFeedback.h"

#include "rose_base_msgs/wheelunit_states.h"
#include "rose_base_msgs/wheelunit_statesAction.h"
#include "rose_base_msgs/wheelunit_statesActionGoal.h"
#include "rose_base_msgs/wheelunit_statesActionResult.h"
#include "rose_base_msgs/wheelunit_statesActionFeedback.h"

#include "opteq_wheelunits_01/wheel_unit.hpp"

//! @todo OH: Fix this weird stuff vv
// Those defines should match those in odometry
#define PLATFORM_WHEELBASE_LENGTH   0.575 //0.58 // [m] The heart-heart distance of the wheels in the y direction
#define PLATFORM_WHEELBASE_WIDTH    0.474 //0.48 // [m] The heart-heart distance of the wheels in the x direction

using namespace std;
using namespace rose_shared_variables;

/**
 * The drive controller receives high level movement commands and translates them into
 * individual commands for the separate wheel units.  
 */
class DriveController
{
  protected:
    typedef ServerMultipleClient<rose_base_msgs::cmd_velocityAction> SMC;

  public:
    /**
     * Constructor of the DriveController class
     * @param[in] name The name of this drive controller
     * @param[in] n The ros nodehandle
     * @return A instance of the DriveController class 
     */
    DriveController(string name, ros::NodeHandle n);
    /**
     * Deconstructor of the DriveController class
     */
    ~DriveController();

    //! @todo OH: HACK quite standard thing, nicer to be able to add this somehow as a plugin to the FootprintCollisionChecker
    void CB_laserUpdate(const sensor_msgs::LaserScan& input_scan);

    //! @todo OH: HACK quite standard thing, nicer to be able to add this somehow as a plugin to the FootprintCollisionChecker
    void CB_bumperUpdate(const contact_sensor_msgs::bumpers& bumper_states);

    //! @todo doxygen info
    void CB_success(const actionlib::SimpleClientGoalState& state, const rose_base_msgs::wheelunit_statesResultConstPtr& client_result);
    void CB_fail(const actionlib::SimpleClientGoalState& state, const rose_base_msgs::wheelunit_statesResultConstPtr& client_result);

    /**
     * Callback method for the cmd_velocity action, receving a goal with a geometry_msg::Twist containing the command velocity.
     * executeMovement() is called to set the received comand velocity.
     * It then send these calculated wheel_states to wheel_controller.
     * @param[in] goal The received command velocity
     * @param[in] smc pointer to the server multiple client
     * @return void
     */
    void CB_CommandVelocity(const rose_base_msgs::cmd_velocityGoalConstPtr& goal, SMC* smc);

    /**
     * Calls the required wheel states by calling calculateStrafeMovement(), calculateCircularMovement() or stopMovement() given the input velocities.
     * It then send these calculated wheel_states to wheel_controller by calling setWheelUnitStates().
     * @param[in] x_velocity The veloctiy over the x-axis of the platform.
     * @param[in] y_velocity The velocity over the y-axis of the platform. Positive y is the platforms natural forward direction.
     * @param[in] w_velocity The angular velocity of the platform around its centerpoint.
     * @param[out] wheelunits_ Will be set and published to the wheel_controller.
     * @return bool Return value of the function called.
     */
    bool executeMovement(float x_velocity, float y_velocity, float w_velocity);

    /**
     * Calculate the rotation rate of the platform, w_velocity, given a certain tangential velocity
     * of the of the middle point of the platform and a certain turn radius. This function 
     * calculates the radial velocity of the middle point and calls calculateCircularMovement().
     * @param[in] tangential_velocity The tangential velocity of the middle point of the platform.
     * @param[in] turn_radius The radius of the turn, measured from the middle point of the platform, which the platform has to make 
     * @return bool False when turn_radius equals zero. The return value of CalculateCircularMovement otherwise.
     */
    bool calculateRadiusMovement(float tangential_velocity, float turn_radius);

    /**
     * Calculate the required wheel angles and speeds given a certain tangential velocity
     * of the of the middle point of the platform and a certain angular rotation rate. 
     * @param[in] tangential_velocity The tangential velocity of the middle point of the platform.
     * @param[in] w_velocity The angular velocity of the platform around its centerpoint.
     * @param[out] wheelunits_ Will be set to the required wheel angles and velocities if this functions returns true.
     * @return bool false When w_velocity equals zero. 
     * @return bool true otherwise.
     */
    bool calculateCircularMovement(float tangential_velocity, float w_velocity);

    /**
     * Calculate the required wheel angles and speeds given a certain x- and y-veloctiy.
     * @param[in] x_velocity The veloctiy over the x-axis of the platform.
     * @param[in] y_velocity The velocity over the y-axis of the platform. Positive y is the platforms natural forward direction.
     * @param[out] wheelunits_ Will be set to the required wheel angles and velocities if this functions returns true.
     * @return bool true
     */
    bool calculateStrafeMovement(float x_velocity, float y_velocity);

    /**
     * Set the wheelunits to the stop state, angle and velocity equal zero.
     * Also calls setWheelUnitStates().
     * @param[out] wheelunits_ Will be set to their stop state.
     * @return void
     */
    bool stopMovement();

    float getSpeedScale( const std::vector<std::pair<WheelUnit, float>>& wheelunit_speeds );

    /**
     * Stores the wanted wheel unit states in the local wheel_units_ map.
     * @param[in] angle_right_front The angle of the right front wheel.
     * @param[in] angle_left_front  The angle of the left front wheel.
     * @param[in] angle_right_back  The angle of the right back wheel.
     * @param[in] angle_left_back   The angle of the left front wheel.
     * @param[in] speed_right_front The veloctiy of the right front wheel.
     * @param[in] speed_left_front  The veloctiy of the left front wheel.
     * @param[in] speed_right_back  The veloctiy of the right back wheel.
     * @param[in] speed_right_front The veloctiy of the left front wheel.
     * @return bool Tsrue if successfull, false otherwise.
     */
    bool setWheelUnitStates(    float angle_right_front, 
                                float angle_left_front, 
                                float angle_right_back, 
                                float angle_left_back, 
                                float speed_right_front, 
                                float speed_left_front, 
                                float speed_right_back, 
                                float speed_left_back);

    /**
     * Publishes the required wheelunit state of wheelunits_ to the topic: 'drive_controller/wheelunit_states_request'
     * @param[out] rose20_platofrm::wheelunit_states Message containing the reuired wheelunit states.
     * @return void
     */
    void requestWheelUnitStates();

    //! @todo OH: HACK
    bool checkFCC();

  private:
    //!< Name of the drive_controller
    string                  name_;
    //!< ROS Nodehandle
    ros::NodeHandle         n_;
    //!< Map containing four WheelUnit objects, named "FR", "FL", "BR", "BL" for the four seperate wheelunits of rose 2.0.
    map<string, WheelUnit>  wheelunits_;
    //!< Server Multiple client interface, has one client, the wheel_controller smc.
    SMC*                    smc_; 

    geometry_msgs::Twist    velocity_;

    //! @todo OH: HACK move this to safety stack
    //!< Footprint collision checker, returns the distance until collision given the footprint, obstacles and a cmd_vel.
    FootprintCollisionChecker   FCC_;
    ros::Subscriber             laser_scan_sub_;
    ros::Subscriber             bumper_states_sub_;

    SharedVariable<bool>        sh_bumper_pressed_;
    OperatorMessaging           operator_gui;
    
};

#endif // DRIVE_CONTROLLER_HPP


