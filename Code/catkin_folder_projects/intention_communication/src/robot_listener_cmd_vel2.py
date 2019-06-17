#!/usr/bin/env python
"""
This module contains a component that access the
cmd_vel message, identify the motion of the robot
and generates message framework to arduino.
"""


import roslib; roslib.load_manifest('intention_communication')
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import playground_interface
import numpy as np
command_to_sent ="some"

class MotionIdentification(object):
    """
    Identify the motion of robot from the linear velocity x,y and
    angular velocity z data
    """
    def __init__(self):
        # params
        self.cmd_vel = None
        self.x_linear_velocity = 0
        self.y_linear_velocity = 0
        self.z_angular_velocity = 0
        self.motion = "None"
        self.angle = 0.0
        #framework params
        self.led_numbers_hex = ""
        self.led_colour = ""
        self.header = "<RIC##"
        self.led_frame = "LED:"
        self.ledpattern_frame = ";PAT:1020"
        self.buzzer_frame = ";BUZ:2000;50;2003"
        self.end_of_frame = "#RIC>"

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(0.5)
        # subscribers
        rospy.Subscriber("/cmd_vel", Twist, self.update_cmd_vel_message)

    def update_cmd_vel_message(self, msg):
        """
        Obtains velocity messages.
        """
        self.x_linear_velocity = msg.linear.x
        self.y_linear_velocity = msg.linear.y
        self.z_angular_velocity = msg.angular.z


    def update_parameters(self):
        """
        From cmd_vel message and return the motion direction and LED list
        """
        led_numbers = []
        angle_in_radian = np.tanh(self.y_linear_velocity/self.x_linear_velocity)
        angle_in_degree =  np.deg2rad(angle_in_radian)
        motion = "not found"
        #based on the calculated angle, decide the region of commanded Motion
        # First you may prefer to check for rotation
        if(abs(self.z_angular_velocity) >= 0.5):
            if self.z_angular_velocity > 0:
                motion = "ACW rotation"
            else:
                motion = "CW rotation"
        else:
            if (angle_in_degree>330.0 and angle_in_degree<30.0):
                led_numbers = [1, 2, 10, 9]
                motion = "straight"
            elif (angle_in_degree<=60.0 and angle_in_degree>=30.0):
                led_numbers = [1, 2, 3]
                motion = "TopLeft Diagonal"
            elif (angle_in_degree>60.0 and angle_in_degree<120.0):
                led_numbers = [1, 2, 3, 4, 5]
                motion = "Left"
            elif (angle_in_degree>=120.0 and angle_in_degree<=150.0):
                led_numbers = [3, 4, 5]
                motion = "BottomLeft Diagonal"
            elif (angle_in_degree>150.0 and angle_in_degree<210.0):
                led_numbers = [4, 5, 6, 7]
                motion = "Reverse"
            elif (angle_in_degree>=210.0 and angle_in_degree<=240.0):
                led_numbers = [6, 7, 8]
                motion = "BottomRight Diagonal"
            elif (angle_in_degree>240.0 and angle_in_degree<300.0):
                led_numbers = [6, 7, 8, 9, 10]
                motion = "Right"
            elif (angle_in_degree>=300.0 and angle_in_degree<=330.0):
                led_numbers = [ 8, 9, 10]
                motion = "TopRight Diagonal"
        return motion, led_numbers


    def set_bit(self,v, index, x):
        """Set the index:th bit of v to 1 if x is truthy, else to 0, and return the new value."""
        mask = 1 << index-1   # Compute mask, an integer with just bit 'index' set.
        v &= ~mask          # Clear the bit indicated by the mask (if x is False)
        if x:
            v |= mask         # If x was True, set the bit indicated by the mask.
        return v            # Return the result, we're done.


    def create_framework(self, led_number, led_colour):
        led_number_in_hex = 0
        for n in led_number:
            led_number_in_hex = self.set_bit(led_number_in_hex, n, 1)
        led_number_in_hex = hex(led_number_in_hex)
        led_number_in_hex = led_number_in_hex.upper()
        self.led_numbers_hex = led_number_in_hex[2:]
        self.led_colour = "00FF00"
        message_framework = self.header+self.led_frame+self.led_numbers_hex+\
        ":"+self.led_colour+self.ledpattern_frame+self.buzzer_frame+self.end_of_frame
        return message_framework

    def send_framework(self, msg_framework):
        serial_msg.send_command(msg_framework)

    def start(self):
        """
        Start sending the messages to arduino

        """
        rospy.loginfo("Ready to start...")
        while not rospy.is_shutdown():
            rospy.loginfo("getting command vel: x={}, y={}, z={}".format(self.x_linear_velocity,\
		self.y_linear_velocity, self.z_angular_velocity))
            movement, leds = self.update_parameters()
            if (self.motion != movement):
                self.motion = movement
                framework = self.create_framework(leds,"00FF00")
                rospy.loginfo("detected motion is "+ motion)
                self.send_framework(framework)
            else:
                rospy.loginfo("Data is the same")
            # send_to_arduino(self.x_linear_velocity, self.y_linear_velocity,self.z_angular_velocity)
            # self.loop_rate.sleep()


if __name__ == '__main__':
    serial_msg = playground_interface.SerialInterface(9600,1,"239A")
    serial_msg.open_port()

    rospy.init_node('cmd_vel_listener')
    identify_motion = MotionIdentification()
    identify_motion.start()
    # msg = identify_motion.create_framework([0,1,2,3],"FF0000")
    # print(msg)
