#!/usr/bin/env python

## Simple ropod_talker that published std_msgs/Strings messages
## to the 'circuit_playground' topic

import rospy
from std_msgs.msg import String

def ropod_talker():
    msg_publisher = rospy.Publisher('circuit_playground', String, queue_size=10)
    rospy.init_node('ropod_talker', anonymous=True)
    publish_rate = rospy.Rate(.125) # 10hz
    while not rospy.is_shutdown():
        test_framework = "<RIC##LED:0387:FFFFFF;PAT:1000;BUZ:2000;50;2001#RIC>"
        rospy.loginfo(test_framework)
        msg_publisher.publish(test_framework)
        publish_rate.sleep()

if __name__ == '__main__':
    try:
        ropod_talker()
    except rospy.ROSInterruptException:
        pass
