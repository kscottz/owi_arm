#!/usr/bin/env python
import rospy
from owi_arm.srv import *
from std_msgs.msg import Int16MultiArray
import sys, os

class ArmController:
    def __init__(self):
        self.debug = rospy.get_param('~debug', False)
        rospy.loginfo("Start Arm Controller")
        #rospy.logwarn("[Top partfinder] Waiting for ImagePtToWorld...")
        self.waypoint_service = rospy.Service('/waypoint', waypoint, self._handle_save_state_to_file)   
        self.path = rospy.get_param('/animation_path','/home/kscottz/Desktop/')
        rospy.Subscriber("/state", Int16MultiArray, self.update_state)
        self.state = [0,0,0,0,0]
        self._run()

    def update_state(self,msg):
        self.state = msg.data

    def _run(self):
        rospy.spin()

    def _handle_save_state_to_file(self,req):
        rospy.wait_for_message('/state', Int16MultiArray, timeout=10)
        fname = self.path+req.fname
        out_str = "{0} {1} {2} {3}".format(self.state[1],self.state[2],self.state[3],self.state[4])
        if( not os.path.isfile(fname) ): # open in write mode
            with open(fname, 'w') as outfile:
                outfile.write(out_str)
        else:
            with open(fname, 'a') as outfile:
                outfile.write(out_str)
        msg = "Wrote {0} to file {1}.".format(out_str,fname)
        rospy.logwarn(msg)
        response = waypointResponse()
        response.result = msg
        return response

if __name__ == '__main__':
    try:
        rospy.init_node('ArmController')
        arm_controller = ArmController()
    except rospy.ROSInterruptException:
        pass
