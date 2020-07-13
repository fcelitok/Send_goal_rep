#!/usr/bin/env python

import rospy
import time
import actionlib
import math
import tf

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
 
#added
#class TfTransform():
#    def __init__(self):
#        
#        TFx = GGoal.initial_point[0]
#        TFy = GGoal.initial_point[1]
#        TFw = GGoal.initial_point[2]
#
#        br = tf.TransformBroadcaster()
#        br.sendTransform((TFx,TFy,0),
#                         tf.transformations.quaternion_from_euler(0, 0, TFw),
#                         rospy.Time.now(),"base_footprint","map")

#######       



class InitialGoalMove():

    def __init__(self):


        self.goal_point = []
        
        rospy.init_node('from_initial_to_goal_node')
        initial_point = rospy.get_param('send_goal_point/init_point_pose_param')
        self.goal_point = rospy.get_param('send_goal_point/goal_point_pose_param')

        initial_quat = Quaternion(*(quaternion_from_euler(0,0,initial_point[2]*math.pi/180,axes='sxyz')))
        self.goal_quat = Quaternion(*(quaternion_from_euler(0,0,self.goal_point[2]*math.pi/180,axes='sxyz')))

        #create initial pose publisher 
        self.pub = rospy.Publisher('/initialpose',PoseWithCovarianceStamped,queue_size=10)
        init_pose_msg = PoseWithCovarianceStamped()
        init_pose_msg.header.frame_id = 'map'
        
        init_pose_msg.pose.pose.position.x = initial_point[0]
        init_pose_msg.pose.pose.position.y = initial_point[1]
        init_pose_msg.pose.pose.orientation.w = initial_quat.w
        

        #create action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")

        #send publisher message
        self.pub.publish(init_pose_msg)
        rospy.loginfo("INITIAL POSE SEND")

        self.movebase_client()

    def movebase_client(self):
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.goal_point[0]
        goal.target_pose.pose.position.y = self.goal_point[1]
        goal.target_pose.pose.orientation.w = self.goal_quat.w

        self.client.send_goal(goal)
        result = self.client.wait_for_result()

        if not result:
            rospy.logerr("There is no result!")
            rospy.signal_shutdown("There is no result!")
            rospy.logerr("There is an error!")
        else:
            rospy.loginfo("Turtlebot reached goal pose.")
            return self.client.get_result()
            


if __name__ == '__main__':
    try:
        InitialGoalMove()
        #Tftransform()

        #rospy.init_node('movebase_client_py')
        #result = movebase_client()
        #if result:
        #    rospy.loginfo("Goal execution done!")
    
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
        pass
    
