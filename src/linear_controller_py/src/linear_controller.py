#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

##
# Parameters
##
P_GAIN = 40


####################
# Controller class #
####################
class LinearController:
    # Constructor
    def __init__(self, p_gain):

        # Assign controller parameters
        self.p_gain = p_gain

        # Set initial conditions
        self.setpoint = 0.

        # Set up publisher
        self.velocity_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # Set up subscribers
        self.setpoint_sub = rospy.Subscriber('setpoint', Float64, self.setpointCallback)
        self.position_sub = rospy.Subscriber('odom', Odometry, self.positionCallback)
        
        # Set up safety timout - will stop robot when timer elapses
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timoutCallback)


    # If this timer runs out, set the robot's velocity to zero
    def timoutCallback(self, msg):
        # Create message
        command = Twist()
        
        # Publish empty message to stop robot
        self.velocity_pub.publish(command)


    # Callback function for incoming setpoint
    def setpointCallback(self, msg):
        self.setpoint = float(msg.data)


    # Callback function for incoming robot position
    def positionCallback(self, msg):
        # Reset the timer
        self.timer.shutdown()
        self.timer.run()

        # Calculate control input
        position = float(msg.pose.pose.position.x)
        error = self.setpoint - position
        input_ = error * self.p_gain

        # Create message
        command = Twist()
        command.linear.x = input_
        
        # Publish message
        self.velocity_pub.publish(command)



#################
# Main function #
#################
def main():
    # Initialize the node
    rospy.init_node('linear_controller', anonymous=True)

    # Create the controller
    controller = LinearController(P_GAIN)
    
    # Wait for messages on topic, go to callback function when new messages arrive.
    rospy.spin()



##########################
# Entry point to program #
##########################
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass