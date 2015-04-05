#!/usr/bin/env python 
# THIS SHEBANG IS REALLY REALLY IMPORTANT
import rospy
import numpy as np
import time
from std_msgs.msg import Int16MultiArray

if __name__ == '__main__':
    try:
        rospy.init_node('simple_publisher')
        time.sleep(1)
        pub = rospy.Publisher('/robot', Int16MultiArray, queue_size=1)
        out = Int16MultiArray()
        out.data = [0,120,120,120,120]
        pub.publish(out)
        rospy.logwarn("SENT: {0}".format(out.data))

    except rospy.ROSInterruptException:
        rospy.logwarn('ERROR!!!')
