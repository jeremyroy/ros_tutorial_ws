#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"

// Specify the controller class' structure
class LinearController
{
public:
    LinearController(ros::NodeHandle* nh, double p_gain);
private:
    /* Member attributes */
    // ros-specific
    ros::NodeHandle m_nh;
    ros::Publisher m_velocity_pub;
    ros::Timer m_timer;
    ros::Subscriber m_setpoint_sub;
    ros::Subscriber m_position_sub;
    // controller specific
    double m_setpoint; // Setpoint for controller
    double m_p_gain;

    /* Member methods */
    void timoutCallback(const ros::TimerEvent& e);
    void setpointCallback(const std_msgs::Float64& msg);
    void positionCallback(const nav_msgs::Odometry& msg);
};