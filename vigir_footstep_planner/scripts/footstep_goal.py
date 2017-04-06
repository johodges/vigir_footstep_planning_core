#!/usr/bin/env python

"""footstep_goal.py - Version 1.0 2017-04-05
Created by Jonathan Hodges
This code publishes a goal for the vigir humanoid path planning service.

Subscribers:

Publishers:
    /goal_pose: target location for robot

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details at:
http://www.gnu.org/licenses/gpl.html
"""

import rospy
import actionlib
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from geometry_msgs.msg import Quaternion
from vigir_footstep_planning_msgs.msg import StepPlan, ExecuteStepPlanAction, ExecuteStepPlanActionGoal
import numpy as np
import tf

class footstep_goal():
    def __init__(self):
        """This initializes the class.
        """
        rospy.init_node('footstep_goal', anonymous=True) # Name this node
        # Define user inputs
        self.x = 1.5
        self.y = 0.0
        self.z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = -0.39
        self.timeout = 15

        # Convert to quaternion
        self.q = tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)

        # Establish publishers and subscribers
        self.goal_pub = rospy.Publisher("/goal_pose", PoseStamped, queue_size=1)
        rospy.Subscriber("/vigir/footstep_planning/step_plan", StepPlan, self.callback, queue_size=1)
        self.execute_client = actionlib.SimpleActionClient("/vigir/footstep_manager/execute_step_plan", ExecuteStepPlanAction)

        # Build goal_pose
        self.goal = PoseStamped()
        self.goal.header.frame_id = "world"
        self.goal.header.stamp = rospy.Time.now()
        while self.goal.header.stamp.secs == 0:
            self.goal.header.stamp = rospy.Time.now()
            rospy.sleep(0.1)
        self.goal.pose.position.x = self.x
        self.goal.pose.position.y = self.y
        self.goal.pose.position.z = self.z
        self.goal.pose.orientation.x = self.q[0]
        self.goal.pose.orientation.y = self.q[1]
        self.goal.pose.orientation.z = self.q[2]
        self.goal.pose.orientation.w = self.q[3]
        for i in range(0,5):
            self.goal_pub.publish(self.goal)
            rospy.sleep(0.1)
        print self.goal
        self.current_time = 0
        self.success = False
        while self.current_time < self.timeout and not self.success:
            rospy.sleep(0.1)
            self.current_time = self.current_time+0.1
        # If a successful step plan was found, execute it. Otherwise kill the node
        if self.success:
            print "Success"
            self.path = ExecuteStepPlanActionGoal()
            self.path.goal.step_plan = self.step_plan
            print self.path.goal
            self.execute_client.send_goal(self.path.goal)
            rospy.sleep(0.5)
            rospy.signal_shutdown('Ending Node.')
            
        else:
            print "Failure"
            rospy.signal_shutdown('Ending Node.')
    def callback(self, data):
        print "Recieved step plan."
        self.success = True
        self.step_plan = data
if __name__ == '__main__':
    footstep_goal()
    rospy.spin()
