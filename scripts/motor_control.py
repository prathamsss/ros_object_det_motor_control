#!/usr/bin/env python3

"""
ROS Node for controlling a motor.

This node publishes Twist messages to the /cmd_vel 
topic based on the motor status parameter.
"""

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class MotorControlNode:
    """
    Class for controlling a motor.

    Attributes:
        stop_motor (bool): Flag for stopping the motor.
        cmd_vel_pub (rospy.Publisher): Publisher for Twist messages.
        rate (rospy.Rate): ROS rate object for controlling loop frequency.
    """


    def __init__(self):
        """Initialize MotorControlNode object."""
        self.stop_motor = False
        self.motor_sub = rospy.Subscriber('/motor_status', Bool, self.callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(1)
        self.twist_msg = Twist()
        # rospy.set_param("motor_status",False)


    def callback(self,data):
        """Control the motor based on the motor status parameter.""" 
        
        if data.data:
            self.twist_msg.linear.x = 0.0
            self.twist_msg.linear.y = 0.0
            rospy.loginfo("MOTOR STATUS: OFF")

            self.cmd_vel_pub.publish(self.twist_msg)
        else:
            self.twist_msg.linear.x = 7.0
            self.twist_msg.linear.y = 7.0

            self.cmd_vel_pub.publish(self.twist_msg)
            rospy.loginfo("MOTOR STATUS: ON")
            
       
        # rospy.loginfo(self.twist_msg.linear)

    def run(self):
        """Main loop for controlling the motor."""
        while not rospy.is_shutdown():
            self.rate.sleep()


def main():
    """Initialize the motor control node and run it."""
    rospy.init_node('motor_control_node', anonymous=True)
    rospy.loginfo("Starting Motor Control Node")
    motor_control_node = MotorControlNode()
    motor_control_node.run()

if __name__ == '__main__':
    main()
