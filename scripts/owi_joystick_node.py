import rospy
import roscpp
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray
class MyJoystickNode(object):
    def __init__(self):
         rospy.on_shutdown(self.shutdown)
         self.pub = rospy.Publisher('robot', Int16MultiArray, queue_size=1)
         rospy.Subscriber("/joy", Joy, self.do_it)
         rospy.init_node('owi_joystick_node')
         self.state = [0,0,0,0]
         rospy.spin()
        
    def do_it(self,msg):
        s = 1000
        m1 = 0
        m2 = 0
        m3 = 0
        m4 = 0
        if(msg.buttons[1] == 1 ):
            m1 = s
        elif( msg.buttons[2] == 1 ):
            m1 = -1*s
        if(msg.buttons[0] == 1 ):
            m2 = s
        elif( msg.buttons[3] == 1 ):
            m2 = -1*s
        if(msg.axes[-1] > 0 ):
            m3 = s
        elif( msg.axes[-1] < 0 ):
            m3 = -1*s
        if(msg.axes[-2] > 0 ):
            m4 = s
        elif( msg.axes[-2] < 0 ):
            m4 = -1*s
            
        data = [int(m1),int(m2),int(m3),int(m4)]
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
    # Initialize the node and name it.
    rospy.init_node('owi_joystick_node')
    node = MyJoystickNode()
#    try:
 #   except rospy.ROSInterruptException:
 #       rospy.logwarn('ERROR!!!')
