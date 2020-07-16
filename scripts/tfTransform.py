#!/usr/bin/env python

import rospy
import time
import actionlib
import math
import tf
import tf2_ros
import geometry_msgs.msg

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseActionFeedback, MoveBaseFeedback
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
       

class Tftransform_class():

    def __init__(self):

        #taking initial and goal points from parameter server 
        self.initial_point = rospy.get_param('tfTransform/init_pose_param')
        #euler to queternion transform 
        self.initial_quat = tf.transformations.quaternion_from_euler(0,0,self.initial_point[2]*math.pi/180,axes='sxyz')
        #self.initial_quat = Quaternion(*(quaternion_from_euler(0,0,self.initial_point[2]*math.pi/180,axes='sxyz')))

        broadcaster = tf2_ros.TransformBroadcaster()
        transformStamped = geometry_msgs.msg.TransformStamped()

        transformStamped.header.stamp = rospy.Time.now()
        transformStamped.header.frame_id = "map"
        transformStamped.child_frame_id = "base_footprint"

        transformStamped.transform.translation.x = self.initial_point[0]
        transformStamped.transform.translation.y = self.initial_point[1]
        transformStamped.transform.rotation.x= self.initial_quat[0]
        transformStamped.transform.rotation.y= self.initial_quat[1]
        transformStamped.transform.rotation.w= self.initial_quat[2]
        transformStamped.transform.rotation.z = self.initial_quat[3] 

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction) #I am checking for move_base aciton is it waiting or active
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))

        rate = rospy.Rate(20) #20 Hz

        if not wait:
            while not wait:
                broadcaster.sendTransform(transformStamped)
                rate.sleep()
                wait = self.client.wait_for_server(rospy.Duration(5.0))
        
        transformStamped.transform.translation.x = feedbacklist[0]
        transformStamped.transform.translation.y = feedbacklist[1]
        transformStamped.transform.rotation.x= feedbacklist[2]
        transformStamped.transform.rotation.y= feedbacklist[3]
        transformStamped.transform.rotation.w= feedbacklist[4]
        transformStamped.transform.rotation.z = feedbacklist[5] 
        while rospy.is_shutdown():
            broadcaster.sendTransform(transformStamped)
            rate.sleep() 


def callbackfb(message):
    feedbackX = message.feedback.base_position.pose.position.x
    feedbackY = message.feedback.base_position.pose.position.y
    feedbackOX = message.feedback.base_position.pose.orientation.x
    feedbackOY = message.feedback.base_position.pose.orientation.y
    feedbackOZ = message.feedback.base_position.pose.orientation.z
    feedbackW = message.feedback.base_position.pose.orientation.w
    
    feedbacklist = [feedbackX,feedbackY,feedback0X,feedbackOY,feedbackOZ,feedbackW]

    #rospy.loginfo("X Y W  %s, %s, %s", feedbackX, feedbackY, feedbackW)

    

if __name__ == '__main__':
    
    try:
        feedbacklist = []
        rospy.init_node('tf_transform_node')
        
        Tftransform_class()
        
        rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, callbackfb)
        rospy.spin()
        
        

    
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
        pass
    





        
""" #Static TF Transform part
        br = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "map"
        static_transformStamped.child_frame_id = "base_footprint"
        
        static_transformStamped.transform.translation.x = self.initial_point[0]
        static_transformStamped.transform.translation.y = self.initial_point[1]
        static_transformStamped.transform.rotation.x= self.initial_quat[0]
        static_transformStamped.transform.rotation.y= self.initial_quat[1]
        static_transformStamped.transform.rotation.w= self.initial_quat[2]
        static_transformStamped.transform.rotation.z = self.initial_quat[3] 
        

        rate = rospy.Rate(20) # 20hz
        while not rospy.is_shutdown():   
            br.sendTransform(static_transformStamped)
            rate.sleep()

       #create action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(10.0))

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")

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

        
 """  