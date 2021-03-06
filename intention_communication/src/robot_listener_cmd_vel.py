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
from ropod_ros_msgs.msg import DockingCommand
from ropod_ros_msgs.msg import  DockingFeedback
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
        self.docking=None
        self.docking_state = None
        self.prev_dock_status = 0
        #framework params
        self.led_numbers_hex = ""
        self.led_colour = ""
        self.header = "<RIC##"
        self.led_frame = "LED:"
        self.ledpattern_frame = ";PAT:"
        self.ledpattern_number = "1020"
        self.buzzer_frame = ";BUZ:2000;50;2003"
        self.end_of_frame = "#RIC>"

        # node cycle rate (in hz)
        self.loop_rate = rospy.Rate(2)
        # subscribers
        rospy.Subscriber("/cmd_vel", Twist, self.update_cmd_vel_message)
        rospy.Subscriber("/ropod/ropod_low_level_control/cmd_dock",DockingCommand,self.update_dock_message)
        rospy.Subscriber("/ropod/ropod_low_level_control/dockingFeedback",DockingFeedback,self.update_dock_status)

    def get_docking_status(self):
        """
        Obtains docking status.
        """
        status = False
        if (self.docking == 1):
            status = True
        else:
            status = False
        return status

    def update_dock_message(self, msg):
        """
        Obtains docking messages.
        """
        nowisastring=format(msg)
        docking_string= nowisastring.split(' ')[1]
        rospy.loginfo("Received docking command:  "+docking_string)
        self.docking=int(docking_string)

    def update_dock_status(self, msg):
        """
        Obtains internal status while docking.
        """
        self.docking_state = msg.docking_status
        # nowisastring=format(msg)
        # docking_string= nowisastring.split('\n\r')[0]
	    # docking_string= docking_string.split(': ')[1]
        # rospy.loginfo("Received docking command:  "+docking_string)
        # self.docking_state=int(docking_string)

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
        angle_in_radian = np.arctan2((self.y_linear_velocity+.0001),(self.x_linear_velocity+.0001))
        angle_in_degree =  np.rad2deg(angle_in_radian)
        motion = "find motion dude"
        #based on the calculated angle, decide the region of commanded Motion
        # First you may prefer to check for rotation
        rospy.loginfo("angle_in_degree debug = "+str(angle_in_degree))
        if(abs(self.z_angular_velocity) >= 0.5):
            if self.z_angular_velocity > 0:
                motion = "ACW rotation"
                led_numbers = [1,2,3,4,5,6,7,8,9,10]
                self.ledpattern_number = "1009"
            else:
                motion = "CW rotation"
                led_numbers = [1,2,3,4,5,6,7,8,9,10]
                self.ledpattern_number = "1008"
        else:
            rospy.loginfo("dock_State = {}".format(self.docking_state))
            if self.docking_state in range(1,6):
                motion = "docking"
                led_numbers = [1,2,3,4,5,6,7,8,9,10]
                self.ledpattern_number = "1020"
                rospy.loginfo("inside dock_State")
            elif self.docking_state==12:
                motion = "undocking"
                led_numbers = [1,2,3,4,5,6,7,8,9,10]
                self.ledpattern_number = "1020"
                rospy.loginfo("inside dock_State")
            else:
                if(self.y_linear_velocity ==0 and self.x_linear_velocity==0):
                    if(self.prev_dock_status!=self.docking and self.docking == 1):
                        self.prev_dock_status = self.docking
                        led_numbers = [1,2,3,4,5,6,7,8,9,10]
                        self.ledpattern_number = "1020"
                        motion = "Dockin"
                    else:
                        led_numbers = []
                        motion = "None"
                elif (angle_in_degree<30 and angle_in_degree>-30.0):
                    led_numbers = [1, 2, 10, 9]
                    self.ledpattern_number = "1020"
                    motion = "straight"
                elif (angle_in_degree<=60.0 and angle_in_degree>=30.0):
                    led_numbers = [1, 2, 3]
                    self.ledpattern_number = "1020"
                    motion = "TopLeft Diagonal"
                elif (angle_in_degree>60.0 and angle_in_degree<120.0):
                    led_numbers = [1, 2, 3, 4, 5]
                    self.ledpattern_number = "1020"
                    motion = "Left"
                elif (angle_in_degree>=120.0 and angle_in_degree<=150.0):
                    led_numbers = [3, 4, 5]
                    self.ledpattern_number = "1020"
                    motion = "BottomLeft Diagonal"
                elif ((angle_in_degree>150.0 and angle_in_degree<=180.0) or (angle_in_degree<-150.0 and angle_in_degree>=-180.0) ):
                    led_numbers = [4, 5, 6, 7]
                    self.ledpattern_number = "1020"
                    motion = "Reverse"
                elif (angle_in_degree>=-150 and angle_in_degree<=-120.0):
                    led_numbers = [6, 7, 8]
                    self.ledpattern_number = "1020"
                    motion = "BottomRight Diagonal"
                elif (angle_in_degree>-120.0 and angle_in_degree<-60):
                    led_numbers = [6, 7, 8, 9, 10]
                    self.ledpattern_number = "1020"
                    motion = "Right"
                elif (angle_in_degree>=-60.0 and angle_in_degree<=-30.0):
                    led_numbers = [ 8, 9, 10]
                    self.ledpattern_number = "1020"
                    motion = "TopRight Diagonal"
        return motion, led_numbers


    def set_bit(self,v, index, x):
        """Set the index:th bit of v to 1 if x is truthy, else to 0, and return the new value."""
        rospy.loginfo(index)
        mask = 1 << index-1   # Compute mask, an integer with just bit 'index' set.

        v &= ~mask          # Clear the bit indicated by the mask (if x is False)
        if x:
            v |= mask         # If x was True, set the bit indicated by the mask.
        return v            # Return the result, we're done.


    def create_framework(self, led_number, led_colour):
        """
        from received LED colour and LED number, create string message framework to
	be sent to arduino

        """
        led_number_in_hex = 0
        rospy.loginfo("led_number debug = "+str(led_number))
        if (len(led_number)==0):
            led_number_in_hex = "0000"
            self.led_numbers_hex = led_number_in_hex
        else:
            for n in led_number:
                led_number_in_hex = self.set_bit(led_number_in_hex, n, 1)
            led_number_in_hex = hex(led_number_in_hex)
            led_number_in_hex = led_number_in_hex.upper()
            self.led_numbers_hex = led_number_in_hex[2:]
        rospy.loginfo("docking sttadfk debug = {}".format(self.get_docking_status()))
        if(self.docking_state in range(1,6)):
            self.led_colour = "FFFF00"
        elif(self.docking_state==12):
            self.led_colour = "0000FF"
        elif(self.get_docking_status()):
            self.led_colour = "FF0000"
        else:
            self.led_colour = "00FF00"
        message_framework = self.header+self.led_frame+self.led_numbers_hex+\
        ":"+self.led_colour+self.ledpattern_frame+self.ledpattern_number\
        +self.buzzer_frame+self.end_of_frame
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
            rospy.loginfo("detected motion is "+movement)
            if (self.motion != movement):
                self.motion = movement
                framework = self.create_framework(leds,"000000")
                rospy.loginfo("detected motion is "+ self.motion)
                self.send_framework(framework)
            else:
                rospy.loginfo("Data is the same")
            self.loop_rate.sleep()


if __name__ == '__main__':
    serial_msg = playground_interface.SerialInterface(9600,1,"239A")
    serial_msg.open_port()

    rospy.init_node('cmd_vel_listener')
    identify_motion = MotionIdentification()
    identify_motion.start()
