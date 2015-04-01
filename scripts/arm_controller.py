#!/usr/bin/env python
import rospy
import actionlib
from owi_arm.srv import *
from owi_arm.msg import *

from std_msgs.msg import Int16MultiArray
import sys, os, time

class ArmController:
    def __init__(self):
        self.debug = rospy.get_param('~debug', False)
        rospy.loginfo("Start Arm Controller")
        #rospy.logwarn("[Top partfinder] Waiting for ImagePtToWorld...")
        self.waypoint_service = rospy.Service('/waypoint', waypoint, self._handle_save_state_to_file)   
        self.path = rospy.get_param('/animation_path','/home/kscottz/Desktop/')
        rospy.Subscriber("/state", Int16MultiArray, self.update_state)
        self.pub = rospy.Publisher('/robot', Int16MultiArray, queue_size=1)
        self.state = [0,0,0,0,0]

        self.action_server_play = actionlib.SimpleActionServer('play_animation', play_animationAction, execute_cb=self._handle_play_animation, auto_start = False)

        self.action_server_play.start()


        self._run()

    def update_state(self,msg):
        self.state = msg.data

    def _run(self):
        rospy.spin()

    def _handle_save_state_to_file(self,req):
        rospy.wait_for_message('/state', Int16MultiArray, timeout=10)
        fname = self.path+req.fname
        out_str = "{0},{1},{2},{3}\n".format(self.state[1],self.state[2],self.state[3],self.state[4])
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

    def _handle_play_animation(self,goal):
        fname = self.path + goal.filename 
        sleepy_time = 0.2 #goal.step_size 
        result = play_animationResult()

        if( not os.path.isfile(fname) ): 
            result.result = "Could not find file {0}".format(fname)
            return result
        # feedback = play_animationFeedback()
        # feedback.update = "Loading file {0}.".format(fname)
        # self.action_server_play.publish_feedback(feedback)
        steps = []
        with open(fname, "rt") as f:
            for line in f:
                steps.append([int(j) for j in line.split(',')])
        #feedback.update = "Finished reading file, got {0} commands.".format(len(steps))
        #self.action_server_play.publish_feedback(feedback)
        for step in steps:
            out = Int16MultiArray()
            rospy.loginfo("sending {0}.".format(step))
            out.data = [0,step[0],step[1],step[2],step[3]]
            self.pub.publish(out)
            #feedback.update = "Going to {0}.".format(out)
            #self.action_server_play.publish_feedback(feedback)
            time.sleep(sleepy_time)
        #result.result = "hella wicked!"
        return result
            
        #if self.check_preempt():
        #    result.result = "We did nothing"
        #    return result



        

if __name__ == '__main__':
    try:
        rospy.init_node('ArmController')
        arm_controller = ArmController()
    except rospy.ROSInterruptException:
        pass
