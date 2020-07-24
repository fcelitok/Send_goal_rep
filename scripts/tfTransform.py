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
from nav_msgs.msg import Path


class Tftransform_class():

    def __init__(self):

        # taking initial and goal points from parameter server
        self.initial_point = rospy.get_param('~init_pose_param')
        self.goal_point = rospy.get_param('~goal_point_pose_param')

        sub = rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.pathCallback)

        # euler to queternion transform
        self.initial_quat = tf.transformations.quaternion_from_euler(
            0, 0, self.initial_point[2]*math.pi/180, axes='sxyz')
        
        self.goal_quat = Quaternion(*(quaternion_from_euler(0,0,self.goal_point[2]*math.pi/180,axes='sxyz')))

        # self.initial_quat = Quaternion(*(quaternion_from_euler(0,0,self.initial_point[2]*math.pi/180,axes='sxyz')))
        self.tfTimer = rospy.Timer(rospy.Duration().from_sec(1.0/20), self.tfTimerCallback)

        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.transformStamped = geometry_msgs.msg.TransformStamped()

        
        self.transformStamped.header.frame_id = "map"
        self.transformStamped.child_frame_id = "base_footprint"

        self.transformStamped.transform.translation.x = self.initial_point[0]
        self.transformStamped.transform.translation.y = self.initial_point[1]
        self.transformStamped.transform.rotation.x = self.initial_quat[0]
        self.transformStamped.transform.rotation.y = self.initial_quat[1]
        self.transformStamped.transform.rotation.z = self.initial_quat[2]
        self.transformStamped.transform.rotation.w = self.initial_quat[3]

        # I am checking for move_base aciton is it waiting or active
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        
        self.recieve = False

    def pathCallback(self,pathMessage):
        #print("sendign path message ", pathMessage)
        print("Path message recieved")
        self.recieve = True


    def loop(self):

        rospy.loginfo("Waiting for move_base action SERVER...")
        server_is_available = self.client.wait_for_server(rospy.Duration(5.0))

        rate = rospy.Rate(20)  # 20 Hz

        if server_is_available:
            self.goal_client()   #If server is available send a goal point
            rospy.loginfo("Server available")
            while not rospy.is_shutdown() and not self.recieve:
                rospy.loginfo(self.recieve)
                rate.sleep()
                
            rospy.signal_shutdown("Global plan recieved")
            rospy.loginfo("Shutting down")
        else:
            rospy.logerr("The move base action was not available for 5 second")

    
    def tfTimerCallback(self,event):
        self.transformStamped.header.stamp = rospy.Time.now()
        self.broadcaster.sendTransform(self.transformStamped)
        #print(event) 

    
    def goal_client(self):
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.goal_point[0]
        goal.target_pose.pose.position.y = self.goal_point[1]
        goal.target_pose.pose.orientation.w = self.goal_quat.w

        self.client.send_goal(goal)
        
    

if __name__ == '__main__':
    
    try:

        rospy.init_node('tf_transform_node')
        
        node = Tftransform_class()
        node.loop()

    # rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, callbackfb)
    #    rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
        pass
    