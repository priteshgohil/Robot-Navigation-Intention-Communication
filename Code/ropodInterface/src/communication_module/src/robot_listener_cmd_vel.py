#!/usr/bin/env python
import roslib; roslib.load_manifest('intention_communication')
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import playground_interface
command_to_sent ="some"

x=0
y=0
z=0

def callback(msg):
    global x
    global y
    global z
    rospy.loginfo("Received a /cmd_vel message!")
    if msg.linear.x==x and msg.linear.y==y and msg.angular.z==z:
        rospy.loginfo("Data is the same")
    else:
        x=msg.linear.x
        y=msg.linear.y
        z=msg.angular.z
        if x>0 and y==0 and z==0:
            rospy.loginfo("Going straigt")
            serial_msg.send_command("<RIC##LED:0387:FFFFFF;PAT:1000;BUZ:2000;50;2001#RIC>")
        elif x<0 and y==0 and z>0:
            rospy.loginfo("Going reverse")
            serial_msg.send_command("<RIC##LED:00FC:00EE00;PAT:1001;BUZ:2000;50;2003#RIC>")
        elif x==0 and y==0 and z<0:
            rospy.loginfo("Turning ACW")
            serial_msg.send_command("<RIC##LED:03FF:FF1122;PAT:1009;BUZ:2000;50;2003#RIC>")
        elif x==0 and y==0 and z>0:
            rospy.loginfo("Turning CW")
            serial_msg.send_command("<RIC##LED:03FF:FF1122;PAT:1008;BUZ:2000;50;2003#RIC>")
        elif x==0 and y>0 and z==0:
            rospy.loginfo("Going right")
            serial_msg.send_command("<RIC##LED:03E0:00FF00;PAT:1002;BUZ:2000;50;2002#RIC>")
        elif x==0 and y<0 and z==0:
            rospy.loginfo("Going left")
            serial_msg.send_command("<RIC##LED:001F:00EEFF;PAT:1003;BUZ:2000;50;2003#RIC>")
        elif x>0 and y<0 and z==0:
            rospy.loginfo("TR_Diagonal")
            serial_msg.send_command("<RIC##LED:0380:FFFF22;PAT:1004;BUZ:2000;50;2003#RIC>")
        elif x>0 and y>0 and z==0:
            rospy.loginfo("TL Diagonal")
            serial_msg.send_command("<RIC##LED:0007:FFFF22;PAT:1005;BUZ:2000;50;2003#RIC>")
        elif x<0 and y<0 and z==0:
            rospy.loginfo("BR Diagonal")
            serial_msg.send_command("<RIC##LED:00E0:FFFF22;PAT:1006;BUZ:2000;50;2003#RIC>")
        elif x<0 and y>0 and z==0:
            rospy.loginfo("BL Diagonal")
            serial_msg.send_command("<RIC##LED:001C:FFFF22;PAT:1007;BUZ:2000;50;2003#RIC>")
        else:
            rospy.loginfo("Doing something else")
            serial_msg.send_command("<RIC##LED:03FF:FF0000;PAT:1010;BUZ:2000;50;2003#RIC>")


def listener(uno,dos,tres):
    rospy.init_node('cmd_vel_listener')
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    serial_msg = playground_interface.SerialInterface(9600,1,"239A")
    serial_msg.open_port()
    # rospy.loginfo(msg_l)
    msg_l=[0,0,0]
    msg_a=[0,0,0]
    listener(x,y,z)
