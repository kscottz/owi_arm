from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray
import rospy
import time
import numpy as np


class JointStateRepublisher(object):
    def __init__(self):
        self.m0_trim = rospy.get_param('m0_trim', 0.00)
        self.m1_trim = rospy.get_param('m1_trim', 0.00)
        self.m2_trim = rospy.get_param('m2_trim', 0.00)
        self.m3_trim = rospy.get_param('m3_trim', 0.00)
        self.wrist_trim = rospy.get_param('wrist_trim', 0.00)

        self.state_topic = rospy.get_param('~arduino_topic', '/robot')
        rospy.Subscriber(self.state_topic, Int16MultiArray, self._update_from_arduino)

        #set the joint state publish topic
        self.joint_topic = rospy.get_param('~joint_topic', 'joint_states')
        self.joint_state_pub = rospy.Publisher(self.joint_topic, JointState, queue_size = 1 )
        # m0 m1 m2 wrist m3
        self.states = [0,0,0,0,0]

        self.rate = rospy.get_param('~rate', 30)
        self._run()

    def _run(self):
        r = rospy.Rate(self.rate) # 30hz 
        while not rospy.is_shutdown():
            self._publish_joint_state()
            r.sleep()

    def _update_from_arduino(self,msg):
        self.states[0] = np.deg2rad(msg.data[0]) + np.deg2rad(self.m0_trim)
        self.states[1] = np.deg2rad(msg.data[1]) + np.deg2rad(self.m1_trim)
        self.states[2] = np.deg2rad(msg.data[2]) + np.deg2rad(self.m2_trim)
        self.states[3] = np.deg2rad(self.wrist_trim)
        self.states[4] = np.deg2rad(msg.data[3]) + np.deg2rad(self.m3_trim)

    def _publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name =  ['base_to_body', 'body_to_arm1', 'arm1_to_arm2', 'arm2_to_wrist','wrist_to_endeffector']
        msg.position = self.state
        msg.velocity = [0.00,0.00,0.00,0.00,0.00]
        msg.effort = [0.00,0.00,0.00,0.00,0.00]
        self.joint_state_pub.publish(msg)
