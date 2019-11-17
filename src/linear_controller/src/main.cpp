#include "linear_controller.h"

/*
 * Parameters
 */
#define P_GAIN 4

/*
 * Main function
 */
int main(int argc, char **argv)
{
    // Initialize node
    ros::init(argc, argv, "linear_controller");
    ros::NodeHandle nh;
    
    // Create object to handle node activities
    LinearController ctrl = LinearController(&nh, P_GAIN);

    // Wait for messages on topic, go to callback function when new messages arrive.
    ros::spin();

    return 0;
}
