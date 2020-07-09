#!/usr/bin/env python

import rospy
import time
import actionlib
import math

from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

class MoveBaseSequence():

    def __init__(self):

        rospy.init_node('move_base_sequence')
        points_seq = rospy.get_param('global_planning_supervisor/p_seq')
        yaweulerangles_seq = rospy.get_param('global_planning_supervisor/yea_seq')
        init_pose = rospy.get_param('global_planning_supervisor/init_pose_param') #ekleme
        quat_seq = list() #list of quaternions
        self.pose_seq = list()
        self.goal_cnt = 0

        quat_init = Quaternion(*(quaternion_from_euler(0, 0, init_pose[2]*math.pi/180, axes='sxyz'))) #ekleme

        for yawangle in yaweulerangles_seq:
            #Unpacking the quaternion list and passing it as arguments to Quaternion message constructor
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
            
        n = 3    
        
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]

        for point in points:
            self.pose_seq.append(Pose(Point(*point),quat_seq[n-3])) #creating pose list which every element include [x,y,z,quatset]
            n += 1
        
        #create initial pose publisher added
        self.pub = rospy.Publisher('/initialpose',PoseWithCovarianceStamped,queue_size=10)
        init_pose_msg = PoseWithCovarianceStamped()
        init_pose_msg.header.frame_id = 'map'
        init_pose_msg.pose.pose.position.x = init_pose[0]
        init_pose_msg.pose.pose.position.y = init_pose[1]
        init_pose_msg.pose.pose.orientation.w = quat_init.w

        

        #creat action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")

        self.pub.publish(init_pose_msg)        #Initial pose publisher
        rospy.loginfo("INITIAL POSE SEND")
        #rospy.loginfo("Sending Initial pose " + str(init_pose_msg))

        self.movebase_client()



    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]

        rospy.loginfo("Sending goal pose " + str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()
        

    def active_cb(self):
        rospy.loginfo("Goal pose " + str(self.goal_cnt+1) + " is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        #To print current pose at each feedback:
        #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
        rospy.loginfo("Feedback for goal pose " + str(self.goal_cnt+1) + " received")

    def done_cb(self, status, result):
        self.goal_cnt += 1
    # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached") 
            if self.goal_cnt< len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]

                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(next_goal, self.done_cb, self.active_cb, self.feedback_cb) 
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return

        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")


if __name__ == '__main__':
    try:
        MoveBaseSequence()
    
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
        pass
    
