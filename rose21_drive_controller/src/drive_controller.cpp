/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*   Author: Okke Hendriks
*   Date  : 2014/01/02
*       - File created.
*
* Description:
*   This controller receives high level velocity and direction commands.
*   These will be translated into rotation angles and velocities for the 
*   platform_controller. 
* 
***********************************************************************************/
#include "rose21_drive_controller/drive_controller.hpp"


DriveController::DriveController(string name, ros::NodeHandle n)
    : name_(name)
    , n_(n)
    , sh_bumper_pressed_(SharedVariable<bool>("bumper_pressed"))  //! @todo OH: HACK
{
    // Start the sever multiple client
    smc_ = new SMC(n_, name_, boost::bind(&DriveController::CB_CommandVelocity, this, _1, _2));
    
    smc_->addClient<rose_base_msgs::wheelunit_statesAction>("platform_controller", 
        boost::bind(&DriveController::CB_success, this, _1, _2),
        boost::bind(&DriveController::CB_fail, this, _1, _2),
        NULL, NULL);

    smc_->startServer();

    // Monitor the low-level platform controller alarm state //! @todo OH: HACK
    sh_bumper_pressed_.host();

    // Add Wheelunits to the map
    WheelUnit* wheelunit;
    wheelunit = new WheelUnit("FR", 0);
    wheelunits_.insert( std::pair<string, WheelUnit>(wheelunit->name_, *wheelunit));
    
    wheelunit = new WheelUnit("FL", 2);
    wheelunits_.insert( std::pair<string, WheelUnit>(wheelunit->name_, *wheelunit));
    
    wheelunit = new WheelUnit("BR", 4);
    wheelunits_.insert( std::pair<string, WheelUnit>(wheelunit->name_, *wheelunit));
    
    wheelunit = new WheelUnit("BL", 6);
    wheelunits_.insert( std::pair<string, WheelUnit>(wheelunit->name_, *wheelunit));
    delete wheelunit;
   
    geometry_msgs::PoseStamped frame_of_motion; 
    frame_of_motion.header.frame_id = "base_link";
    frame_of_motion.pose.orientation.w = 1.0;

    FCC_.setFrameOfMotion(frame_of_motion);
    FCC_.showCollisions();

    // laser_scan_sub_ = n_.subscribe("/scan", 1, &DriveController::CB_laserUpdate, this);
    bumper_states_sub_ = n_.subscribe("/lift_controller/bumpers2/state", 1, &DriveController::CB_bumperUpdate, this);

    stopMovement();
}

DriveController::~DriveController()
{
    stopMovement();
}

//! @todo OH: Disabled
void DriveController::CB_laserUpdate(const sensor_msgs::LaserScan& input_scan)
{
    FCC_.clearPoints();    //! @todo OH: WORKS ONLY WITH ONE THING THAT ADDS STUFF

    StampedVertices lethal_points;
    std_msgs::Header header;
    header.stamp    = ros::Time::now();
    header.frame_id = input_scan.header.frame_id;
    
    // HOKUYO scans are CCW
    float current_angle = input_scan.angle_min;
    for(const auto& ray : input_scan.ranges)
    {
        // Is this ray in the valid scan range?
        if(ray >= input_scan.range_min && ray <= input_scan.range_max)
        {
            float x = 1.0;
            float y = 0.0;
            rose_geometry::rotateVect(&x, &y, current_angle);
            rose_geometry::setVectorLengthXY(&x, &y, ray);
            lethal_points.push_back(StampedVertex (header, rose_geometry::Point(x, y, 0.0)));
        }

        // Increment time stamp with time between ray's
        header.stamp += ros::Duration(input_scan.time_increment);

        // Increment current angle
        current_angle += input_scan.angle_increment;
    }
    
    FCC_.addPoints(lethal_points);
}

void DriveController::CB_bumperUpdate(const contact_sensor_msgs::bumpers& bumpers_msg)
{
    StampedVertices lethal_points;

    bool none_pressed = true;
    for(const auto& bumper : bumpers_msg.bumpers)
    {
        // Skip this bumper if not pressed
        if( not bumper.state.data )
            continue;

        none_pressed = false;

        // Add lethal points
        for(const auto& vertex : bumper.footprint.polygon.points)
            lethal_points.push_back(StampedVertex (bumper.footprint.header, rose_geometry::Point(vertex.x, vertex.y, 0.0)));
    }   

    FCC_.clearPoints();    //! @todo OH:  WORKS ONLY WITH ONE THING THAT ADDS STUFF
    FCC_.addPoints(lethal_points);
    
    //! @todo OH: HACK
    if(none_pressed)
        sh_bumper_pressed_ = false;
    else if(sh_bumper_pressed_ == false)    // Only set to true once
    {
        if( not checkFCC() )
            stopMovement();

        sh_bumper_pressed_ = true;
    }
}


void DriveController::CB_success(const actionlib::SimpleClientGoalState& state, const rose_base_msgs::wheelunit_statesResultConstPtr& client_result)
{
    ROS_DEBUG_NAMED(ROS_NAME, "Succesfully set requested wheel unit state.");
    
    rose_base_msgs::cmd_velocityResult server_result;
    server_result.return_code = client_result->return_code;

    if(smc_->hasActiveGoal())
        smc_->sendServerResult(true, server_result);
}

void DriveController::CB_fail(const actionlib::SimpleClientGoalState& state, const rose_base_msgs::wheelunit_statesResultConstPtr& client_result)
{
    ROS_WARN_NAMED(ROS_NAME, "Unable to set requested wheel unit state.");
    rose_base_msgs::cmd_velocityResult server_result;
    server_result.return_code = client_result->return_code;

    if(smc_->hasActiveGoal())
        smc_->sendServerResult(false, server_result);
}

void DriveController::CB_CommandVelocity(const rose_base_msgs::cmd_velocityGoalConstPtr& goal, SMC* smc)
{
    ROS_DEBUG_NAMED(ROS_NAME, "Command velocity goal received");

    if( not executeMovement(goal->cmd_vel.linear.x, goal->cmd_vel.linear.y, goal->cmd_vel.angular.z) )
    {
        ROS_WARN_NAMED(ROS_NAME, "Could not set requested velocity goal, stopping.");
        stopMovement();
        smc_->abort();
    }
    // Otherwise the results depend on platform_controller thus success or failure is registred through CB_success or CB_fail
}

bool DriveController::executeMovement(float x_velocity, float y_velocity, float w_velocity)
{
    ROS_DEBUG_NAMED(ROS_NAME, "executeMovement called with [%.2fm/s, %.2fm/s, %.2frad/s]", x_velocity, y_velocity, w_velocity);
    
    bool succes = false;
    // Move along a circle if our angle speed is non zero
    if(fabs(w_velocity) > 0.01) //! @todo OH: make configurable
    {
        // Take the y-velocity as the center tangential velocity
        succes = calculateCircularMovement(x_velocity, w_velocity);   
        velocity_.linear.x   = x_velocity;  
        velocity_.linear.y   = 0.0;  
        velocity_.angular.z  = w_velocity;  
    }
    // Otherwise 
    else if(fabs(x_velocity) > 0.0 || fabs(y_velocity) > 0.0)
    {
        succes = calculateStrafeMovement(x_velocity, y_velocity);
        velocity_.linear.x   = x_velocity;  
        velocity_.linear.y   = y_velocity;  
        velocity_.angular.z  = 0.0;  
    }
    else
        succes = stopMovement();

    //! @todo OH [IMPR]: Disabled checking for laser scan collisions
    if( not checkFCC() )
    {
        ROS_WARN_THROTTLE(0.1, "Footprint collision checker expects a collision, stopping (this could be a bumper which is pressed).");
        succes = stopMovement();
    }
    
    requestWheelUnitStates();

    return succes;  //! @todo OH: Improve the results, why failed to set a certain state, enum's etc.?  
}

//! @todo OH: HACK
bool DriveController::checkFCC()
{
    ROS_DEBUG_NAMED(ROS_NAME, "Checking FCC...");
    return not FCC_.checkVelocity(velocity_, 1.0); //! @todo OH [IMPR]: Should take velocity and acceleration into account when determining if it can drive somewhere.
}

bool DriveController::calculateRadiusMovement(float w_velocity, float turn_radius)
{
    // Parameter check
    if(turn_radius == 0.0)
    {
        ROS_WARN_NAMED(ROS_NAME, "CalculateRadiusMovement: turn_radius speed cannot be zero.");
        return false;
    }

    ROS_DEBUG_NAMED(ROS_NAME, "CalculateRadiusMovement, w_velocity: %.4f, turn_radius: %.4f", w_velocity, turn_radius);
    float  tangential_velocity = w_velocity*-turn_radius;

    return calculateCircularMovement(tangential_velocity, w_velocity);
}


// Assumes w_velocity != 0.0, w_velocity is angles speed of middle point with
bool DriveController::calculateCircularMovement(float tangential_velocity, float w_velocity)
{
    // Parameter check
    if(w_velocity == 0.0)
    {
        ROS_WARN_NAMED(ROS_NAME, "CalculateCircularMovement: w_velocity speed cannot be zero.");
        return false;
    }   

    // Calculate these values for easier reading
    float half_length       = PLATFORM_WHEELBASE_LENGTH/2.0;
    float half_width        = PLATFORM_WHEELBASE_WIDTH/2.0;

    ROS_DEBUG_NAMED(ROS_NAME, "CalculateCircularMovement, tangential_velocity: %.4f, w_velocity: %.4f", tangential_velocity, w_velocity);
    float turn_radius   = tangential_velocity/w_velocity;

    ROS_DEBUG_NAMED(ROS_NAME, "turn_radius  : %.4f", turn_radius);


    float angle_left_vector     = atan((turn_radius - half_width)/ half_length);
    float angle_right_vector    = atan((turn_radius + half_width)/ half_length);

    float angle_left            = (M_PI/2.0 - angle_left_vector);
    float angle_right           = (M_PI/2.0 - angle_right_vector);

    ROS_DEBUG_NAMED(ROS_NAME, "angle_left: %.4f     | angle_right: %.4f", angle_left, angle_right);
    // ROS_DEBUG_NAMED(ROS_NAME, "angle_left_vector: %.4f   | angle_right_vector: %.4f", angle_left_vector, angle_right_vector);

    int direction = rose_conversions::sgn(tangential_velocity);
    if(rose_conversions::sgn(tangential_velocity) == 0)
        direction = -rose_conversions::sgn(w_velocity);


    float radius_left   = sqrt(pow(half_length, 2.0) + pow(turn_radius - half_width, 2.0));
    float radius_right  = sqrt(pow(half_length, 2.0) + pow(turn_radius + half_width, 2.0));

    float velocity_left     = fabs(w_velocity) * direction * radius_left;
    float velocity_right    = fabs(w_velocity) * direction * radius_right;
    ROS_DEBUG_NAMED(ROS_NAME, "Vel L: %.4f          | Vel R: %.4f", velocity_left, velocity_right);

    // Flip velocity in correct situations
    if(angle_left >= M_PI/2.0 && rose_conversions::sgn(w_velocity)*rose_conversions::sgn(tangential_velocity) > 0.0)
        velocity_left = -velocity_left;
    
    // Flip velocity in correct situations
    if(angle_right <= M_PI/2.0  && rose_conversions::sgn(w_velocity)*rose_conversions::sgn(tangential_velocity) <= 0.0)
        velocity_right = -velocity_right;

    ROS_DEBUG_NAMED(ROS_NAME, "rose_conversions::sgn(w_velocity)*rose_conversions::sgn(tangential_velocity): %d", rose_conversions::sgn(w_velocity)*rose_conversions::sgn(tangential_velocity));
    ROS_DEBUG_NAMED(ROS_NAME, "LIM angle_left: %.4f     | angle_right: %.4f", angle_left, angle_right);
    ROS_DEBUG_NAMED(ROS_NAME, "LIM Vel L: %.4f          | Vel R: %.4f", velocity_left, velocity_right);


    float angle_right_front = -angle_right;
    float angle_left_front  = -angle_left;
    float angle_right_back  = angle_right;
    float angle_left_back   = angle_left;


    if(!setWheelUnitStates( angle_right_front,
                            angle_left_front,
                            angle_right_back,
                            angle_left_back,
                            velocity_right,
                            velocity_left,
                            velocity_right,
                            velocity_left))
        return false;

    return true;
}

bool DriveController::calculateStrafeMovement(float x_velocity, float y_velocity)
{
    ROS_DEBUG_NAMED(ROS_NAME, "CalculateStrafeMovement");

    float angle;
    if(x_velocity == 0.0 && y_velocity == 0.0)
        angle = 0.0;
    else
        angle = atan2(y_velocity, x_velocity);

    float velocity  = sqrt(pow(x_velocity, 2.0) + pow(y_velocity, 2.0));

    ROS_DEBUG_NAMED(ROS_NAME, "Angle: %.4f | Vel: %.4f", angle, velocity);

    // Limit angle to -M_PI/2.0 -> M_PI/2.0, change velocity accordingly
    if(rose_geometry::wrapToHalfPi(&angle))
        velocity = -velocity;

    if(!setWheelUnitStates( -angle,
                            -angle,
                            -angle,
                            -angle,
                            velocity,
                            velocity,
                            velocity,
                            velocity))
        return false;

    return true;
}

bool DriveController::stopMovement()
{
    if( not setWheelUnitStates( 0.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0))
        return false;

    velocity_.linear.x   = 0.0;  
    velocity_.linear.y   = 0.0;  
    velocity_.angular.z  = 0.0;  

    return true;
}

bool DriveController::setWheelUnitStates(   float angle_right_front, 
                                            float angle_left_front, 
                                            float angle_right_back, 
                                            float angle_left_back, 
                                            float speed_right_front, 
                                            float speed_left_front, 
                                            float speed_right_back, 
                                            float speed_left_back)
{
    if(!wheelunits_.at("FR").setAngleRad(angle_right_front))
        return false;
    if(!wheelunits_.at("FL").setAngleRad(angle_left_front))
        return false;
    if(!wheelunits_.at("BR").setAngleRad(angle_right_back))
        return false;
    if(!wheelunits_.at("BL").setAngleRad(angle_left_back))
        return false;

    if(!wheelunits_.at("FR").setVelocityMetersPerSec(speed_right_front))
        return false;
    if(!wheelunits_.at("FL").setVelocityMetersPerSec(speed_left_front))
        return false;
    if(!wheelunits_.at("BR").setVelocityMetersPerSec(speed_right_back))
        return false;
    if(!wheelunits_.at("BL").setVelocityMetersPerSec(speed_left_back))
        return false;

    return true;
}

void DriveController::requestWheelUnitStates()
{
    // Show debug information
    ROS_DEBUG_NAMED(ROS_NAME, "Alphas       FR, FL, BR,BL   : %.4f, %.4f, %.4f, %.4f",  wheelunits_.at("FR").getSetAngleRad(), 
                                                                        wheelunits_.at("FL").getSetAngleRad(),
                                                                        wheelunits_.at("BR").getSetAngleRad(),
                                                                        wheelunits_.at("BL").getSetAngleRad());
    ROS_DEBUG_NAMED(ROS_NAME, "Velocities       FR, FL, BR,BL   : %.4f, %.4f, %.4f, %.4f",  wheelunits_.at("FR").getSetVelocityMetersPerSec(), 
                                                                        wheelunits_.at("FL").getSetVelocityMetersPerSec(),
                                                                        wheelunits_.at("BR").getSetVelocityMetersPerSec(),
                                                                        wheelunits_.at("BL").getSetVelocityMetersPerSec());

    // Fill message with lowlevel values
    rose_base_msgs::wheelunit_states wheelunit_states;    
    wheelunit_states.angle_FR       = wheelunits_.at("FR").getSetAngleLowLevel();
    wheelunit_states.angle_FL       = wheelunits_.at("FL").getSetAngleLowLevel();
    wheelunit_states.angle_BR       = wheelunits_.at("BR").getSetAngleLowLevel();
    wheelunit_states.angle_BL       = wheelunits_.at("BL").getSetAngleLowLevel();

    wheelunit_states.velocity_FR    = wheelunits_.at("FR").getSetVelocityLowLevel();
    wheelunit_states.velocity_FL    = wheelunits_.at("FL").getSetVelocityLowLevel();
    wheelunit_states.velocity_BR    = wheelunits_.at("BR").getSetVelocityLowLevel();
    wheelunit_states.velocity_BL    = wheelunits_.at("BL").getSetVelocityLowLevel();

    // Set this wheel unit state via smc 
    rose_base_msgs::wheelunit_statesGoal goal;
    goal.requested_state = wheelunit_states;
    smc_->sendGoal<rose_base_msgs::wheelunit_statesAction>(goal, "platform_controller", 1.0/25.0);
}
