#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point, Quaternion
from alterego_moveit_config.srv import HappyPoseService, HappyPoseServiceResponse
from geometry_msgs.msg import Pose
import tf.transformations as tft
import os
from std_msgs.msg import Bool
import threading

class FakeHappyPoseService:
    def __init__(self):
        rospy.init_node('fake_happypose_server')
        self.robot_name = os.getenv('ROBOT_NAME', 'robot_alterego3')
        self.service = rospy.Service('/happypose_service', HappyPoseService, self.handle_happypose_service)
        self.pub_head_pose = rospy.Publisher(f'/{self.robot_name}/head_pose', Pose, queue_size=10)
        rospy.Subscriber(f'/{self.robot_name}/auto_mode_status', Bool, self.auto_mode__Callback)

        rospy.loginfo("Fake HappyPose service ready.")

        # Initialize pose and orientation
        quaternion = tft.quaternion_from_euler(-0.7, -0.4, 0.0)
        self.head_pose = Pose()
        self.head_pose.orientation.x = quaternion[0]
        self.head_pose.orientation.y = quaternion[1]
        self.head_pose.orientation.z = quaternion[2]
        self.head_pose.orientation.w = quaternion[3]

        # Flag to indicate the first publication of Button_A
        self.first_button_A_publish = True
    
        self.head_moved = False
        self.new_request = False

        # Condition variable to synchronize head movement
        self.condition = threading.Condition()

        # test pose centro
        self.pose_goal = Pose()
        self.pose_goal.position.x = 0.2597989505766868
        self.pose_goal.position.y = -0.24148760061051408
        self.pose_goal.position.z = 0.6900732726925773
        self.pose_goal.orientation.x = -0.05580263730445686
        self.pose_goal.orientation.y = 0.06700347214530794
        self.pose_goal.orientation.z = 0.7630860249644816
        self.pose_goal.orientation.w = 0.6403876317467787
        self.auto_mode_enabled = False

    def auto_mode__Callback(self, msg):
        self.auto_mode_enabled = msg.data
        if self.auto_mode_enabled:
            rospy.loginfo("Auto mode enabled")
        else:
            rospy.loginfo("Auto mode disabled")

    def handle_happypose_service(self, req):
        rospy.loginfo(f"Received request for object: {req.object}")
        self.new_request = True

        # Wait for the head to move
        with self.condition:
            self.condition.wait_for(lambda: self.head_moved)
        position = Point(self.pose_goal.position.x, self.pose_goal.position.y, self.pose_goal.position.z)
        orientation = Quaternion(self.pose_goal.orientation.x, self.pose_goal.orientation.y, self.pose_goal.orientation.z, self.pose_goal.orientation.w)
        success = True
        return HappyPoseServiceResponse(success=success, position=position, orientation=orientation)



    def move_head(self):
        self.pub_head_pose.publish(self.head_pose)
        self.head_moved = True
        # Notify that the head has moved
        with self.condition:
            self.condition.notify_all()


    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.new_request and self.auto_mode_enabled:
                self.move_head()
                # self.new_request = False
            rate.sleep()

if __name__ == "__main__":
    service = FakeHappyPoseService()
    service.run()