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

    def start(self):
        """
        Start sending the messages to arduino

        """
        rospy.loginfo("Ready to start...")
        while not rospy.is_shutdown():
            rospy.loginfo("inside while loop")
            rospy.loginfo("getting command vel: {}".format(self.x_linear_velocity))
            send_to_arduino(self.x_linear_velocity, self.y_linear_velocity, self.z_angular_velocity)
            self.loop_rate.sleep()
x_prev = 0
y_prev = 0
z_prev = 0
def send_to_arduino(x,y,z):
    global x_prev
    global y_prev
    global z_prev
    rospy.loginfo("Received a /cmd_vel message!")

    if x_prev==x and y_prev==y and z_prev==z:
        rospy.loginfo("Data is the same")
    else:
        if x>0 and y==0:
            rospy.loginfo("Going straigt")
            serial_msg.send_command("<RIC##LED:0387:FFFFFF;PAT:1000;BUZ:2000;50;2001#RIC>")
        elif x<0 and y==0:
            rospy.loginfo("Going reverse")
            serial_msg.send_command("<RIC##LED:00FC:00EE00;PAT:1001;BUZ:2000;50;2003#RIC>")
        elif x==0 and y==0 and z<0:
            rospy.loginfo("Turning ACW")
            serial_msg.send_command("<RIC##LED:03FF:FF1122;PAT:1009;BUZ:2000;50;2003#RIC>")
        elif x==0 and y==0 and z>0:
            rospy.loginfo("Turning CW")
            serial_msg.send_command("<RIC##LED:03FF:FF1122;PAT:1008;BUZ:2000;50;2003#RIC>")
        elif x==0 and y>0:
            rospy.loginfo("Going right")
            serial_msg.send_command("<RIC##LED:03E0:00FF00;PAT:1002;BUZ:2000;50;2002#RIC>")
        elif x==0 and y<0:
            rospy.loginfo("Going left")
            serial_msg.send_command("<RIC##LED:001F:00EEFF;PAT:1003;BUZ:2000;50;2003#RIC>")
        elif x>0 and y<0 :
            rospy.loginfo("TR_Diagonal")
            serial_msg.send_command("<RIC##LED:0380:FFFF22;PAT:1004;BUZ:2000;50;2003#RIC>")
        elif x>0 and y>0 :
            rospy.loginfo("TL Diagonal")
            serial_msg.send_command("<RIC##LED:0007:FFFF22;PAT:1005;BUZ:2000;50;2003#RIC>")
        elif x<0 and y<0 :
            rospy.loginfo("BR Diagonal")
            serial_msg.send_command("<RIC##LED:00E0:FFFF22;PAT:1006;BUZ:2000;50;2003#RIC>")
        elif x<0 and y>0:
            rospy.loginfo("BL Diagonal")
            serial_msg.send_command("<RIC##LED:001C:FFFF22;PAT:1007;BUZ:2000;50;2003#RIC>")
        else:
            rospy.loginfo("Doing something else")
            serial_msg.send_command("<RIC##LED:03FF:FF0000;PAT:1010;BUZ:2000;50;2003#RIC>")
    x_prev = x
    y_prev = y
    z_prev = z

# def listener(uno,dos,tres):
#     rospy.init_node('cmd_vel_listener')
#     rospy.Subscriber("/cmd_vel", Twist, callback)
#     # rospy.rate(100)
#     rospy.spin()

if __name__ == '__main__':
    serial_msg = playground_interface.SerialInterface(9600,1,"239A")
    serial_msg.open_port()

    rospy.init_node('cmd_vel_listener')
    identify_motion = MotionIdentification()
    identify_motion.start()
