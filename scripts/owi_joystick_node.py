#!/usr/bin/env python 
# THIS SHEBANG IS REALLY REALLY IMPORTANT
import rospy
import roscpp
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray
class MyJoystickNode(object):
    def __init__(self):
         rospy.on_shutdown(self.shutdown)
         self.pub = rospy.Publisher('robot', Int16MultiArray, queue_size=1)
         rospy.Subscriber("/joy", Joy, self.do_it)
         rospy.Subscriber("/state", Int16MultiArray, self.update_state)
         rospy.init_node('owi_joystick_node')
         self.state = [0,0,0,0,0]
         rospy.spin()

    def update_state(self,msg):
        self.state = msg.data
#        rospy.loginfo(self.state)

    def do_it(self,msg):
        print "hi"
        m1 = self.state[1]
        m2 = self.state[2]
        m3 = self.state[3]
        m4 = self.state[4]
        step = 10
        if(msg.buttons[1] == 1 ):
            m1+=step
        elif( msg.buttons[2] == 1 ):
            m1-=step
        if(msg.buttons[0] == 1 ):
            m2+=step
        elif( msg.buttons[3] ==1 ):
            m2-=step
        if(msg.axes[-1] > 0 ):
            m3+=step
        elif( msg.axes[-1] < 0 ):
            m3-=step
        if(msg.axes[-2] > 0 ):
            m4+=step
        elif( msg.axes[-2] < 0 ):
            m4-=step
            
        data = [self.state[0],
                int(np.clip(m1,0,180)),
                int(np.clip(m2,0,180)),
                int(np.clip(m3,0,180)),
                int(np.clip(m4,0,180))]
        change = any([abs(a-b)>0 for a,b in zip(data,self.state)])
        self.state = data
        if( change ):
            out = Int16MultiArray()
            rospy.loginfo("sending {0}.".format(data))
            out.data = data
            self.pub.publish(out)

    def shutdown(self):
        data = [0,0,0,0]
        out = Int16MultiArray()
        print "sending {0}.".format(data)
        out.data = data
        pub.publish(out)

 

if __name__ == '__main__':
    try:
        rospy.init_node('owi_joystick_node')
        node = MyJoystickNode()
    except rospy.ROSInterruptException:
        rospy.logwarn('ERROR!!!')
