#!/usr/bin/env python3

import rospy
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
import actionlib
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
from navigation_stack.srv import NavService, NavServiceResponse
from tf.transformations import euler_from_quaternion
import math
from nav_msgs.msg import Path
import subprocess
import threading

class GotoTarget():
    def __init__(self):
        # Get robot name from environment variable
        self.robot_name = os.getenv('ROBOT_NAME', 'robot_alterego3')
        rospy.init_node('navigation', anonymous=False)
        self.service = rospy.Service('/navigation_service', NavService, self.handle_nav_service)
        
        # Inizializzazione e setup
        self.path_length = None
        rospy.on_shutdown(self.shutdown)
        self._point_list = rospy.get_param('navigation/Locations')
        self.target = None

        # Condition variable to synchronize head movement
        self.condition = threading.Condition()
        self.goal_reached = False
        self.new_location_received = False

        # Action client for move_base
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for the action server to come up")
        self.move_base.wait_for_server(rospy.Duration(5))

    def handle_nav_service(self, req):
        rospy.loginfo(f"Received request to go to: {req.location} - {req.sub_location}")
        
        self.new_location = req.location
        self.new_sub_location = req.sub_location
        self.new_location_received = True

        # Wait for going to the location
        with self.condition:
            self.condition.wait_for(lambda: self.goal_reached)
        
        success = self.goal_reached  # Imposta a False se l'azione fallisce
        self.goal_reached = False
        return NavServiceResponse(success=success)

    def navigate_to_location(self):
        goal = MoveBaseGoal()
        # print(self._point_list)

        for location_dict in self._point_list:
            if self.new_location in location_dict:
                print(self.new_location)
                sub_locations = location_dict[self.new_location]
                for sub_location_dict in sub_locations:
                    if self.new_sub_location in sub_location_dict:
                        print(self.new_sub_location)                
                        data = sub_location_dict[self.new_sub_location]
                        position = data['position']
                        orientation = data['orientation']
                        rospy.loginfo(f"Target Position: {position}")
                        rospy.loginfo(f"Target Orientation: {orientation}")
                        
                        # Clear costmaps before sending the goal
                        rospy.wait_for_service(f'/{self.robot_name}/move_base/clear_costmaps')
                        try:
                            clear_costmaps = rospy.ServiceProxy(f'/{self.robot_name}/move_base/clear_costmaps', Empty)
                            clear_costmaps()
                            rospy.loginfo("Costmaps cleared successfully.")
                        except rospy.ServiceException as e:
                            rospy.logerr(f"Service call failed: {e}")

                        # Set up the frame parameters
                        goal.target_pose.header.frame_id = 'map'
                        goal.target_pose.header.stamp = rospy.Time.now()
                        goal.target_pose.pose.position.x = position['x']
                        goal.target_pose.pose.position.y = position['y']
                        goal.target_pose.pose.position.z = position['z']
                        goal.target_pose.pose.orientation.x = orientation['x']
                        goal.target_pose.pose.orientation.y = orientation['y']
                        goal.target_pose.pose.orientation.z = orientation['z']
                        goal.target_pose.pose.orientation.w = orientation['w']

                        # Start moving
                        self.move_base.send_goal(goal)
                        state = self.move_base.get_state()
                        while state not in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.REJECTED] and not rospy.is_shutdown():
                            state = self.move_base.get_state()
                            rospy.sleep(1)
                        
                        if state == GoalStatus.SUCCEEDED:
                            self.goal_reached = True
                            with self.condition:
                                self.condition.notify_all()
                            rospy.loginfo("Goal reached successfully.")
                        else:
                            rospy.loginfo("Failed to reach the goal.")
                            self.goal_reached = False
                            with self.condition:
                                self.condition.notify_all()

    def shutdown(self):
        rospy.loginfo("Stop")

if __name__ == '__main__':
    try:
        GotoTarget = GotoTarget()

        while not rospy.is_shutdown():
            if GotoTarget.new_location_received:
                rospy.loginfo(f"Target location updated to: {GotoTarget.new_location} - {GotoTarget.new_sub_location}")
                GotoTarget.navigate_to_location()
                GotoTarget.new_location_received = False
            rospy.sleep(1)
            
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")