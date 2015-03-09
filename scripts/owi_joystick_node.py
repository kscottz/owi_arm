import rospy
#import roscpp
#from sensor_msgs.msg import Joy
#from std_msgs.msg import Int16MultiArray
class MyJoystickNode(object):
    def __init__(self):
        rospy.spin()
    #     rospy.on_shutdown(self.shutdown)
    #     pub = rospy.Publisher('robot', Int16MultiArray, queue_size=1)
    #     rospy.Subscriber("/joy", Joy, self.do_it)
    #     rospy.init_node('owi_joystick_node')
    #     rospy.spin()
        
    # def do_it(self,msg):
    #     rospy.logwarn("joy {0}".format(msg))
    #     #msg = Int16MultiArray()
    #     #data = [int(args.integers[0]),int(args.integers[1]),int(args.integers[2]),int(args.integers[3])]
    #     #print "sending {0}.".format(data)
    #     #msg.data = data
    #     #pub.publish(msg)

    # def shutdown(self):
    #     # set all motors to zero
    #     pass
 

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('owi_joystick_node')
    try:
        node = MyJoystickNode()
    except rospy.ROSInterruptException:
        rospy.logwarn('ERROR!!!')
