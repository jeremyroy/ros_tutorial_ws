#include "linear_controller.h"

// Constructor
LinearController::LinearController(ros::NodeHandle* nh, double p_gain)
{
    // Assign class with reference of nodehandle
    m_nh = *nh;

    // Assign controller parameters
    m_p_gain = p_gain;

    // Set initial conditions
    m_setpoint = 0;

    // Set up publisher
    m_velocity_pub = m_nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // Set up subscribers
    m_setpoint_sub = m_nh.subscribe("setpoint", 10, &LinearController::setpointCallback, this);
    m_position_sub = m_nh.subscribe("odom", 1, &LinearController::positionCallback, this);

    // Set up safety timeout - will stop robot when timer elapses
    m_timer = m_nh.createTimer(ros::Duration(0.1), &LinearController::timoutCallback, this);
}


// If this timer runs out, set the robot's velocity to zero
void LinearController::timoutCallback(const ros::TimerEvent& e)
{
    // Create message
    geometry_msgs::Twist msg;
    
    // Publish empty message to stop robot
    m_velocity_pub.publish(msg);
}


// Callback function for incoming setpoint
void LinearController::setpointCallback(const std_msgs::Float64& data)
{
    // Save setpoint
    m_setpoint = data.data;
}


// Callback function for incoming robot position
void LinearController::positionCallback(const nav_msgs::Odometry& data)
{
    // Reset the timer
    m_timer.stop();
    m_timer.start();

    // Calculate control input
    double position = data.pose.pose.position.x;
    double error = m_setpoint - position;
    double input = error * m_p_gain;

    // Create message
    geometry_msgs::Twist command;
    command.linear.x = input;
    
    // Publish message
    m_velocity_pub.publish(command);
}