#!/usr/bin/env python3
import sys
import os
import rospy
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from PickObjectClass import PickObjectClass

if __name__ == '__main__':
    
    # Initialize the object
    PickObject = PickObjectClass()

    rate = rospy.Rate(10)  # 10 Hz
    try:
        while not rospy.is_shutdown():
            #1 arriva la posa dall'agent
            if PickObject.new_pose_received:
                #2 Definisco il goal pose
                PickObject.pose_goal = PickObject.create_pose(PickObject.received_position, PickObject.received_orientation)
                #3 Set di un pre grasp pose
                PickObject.pre_grasp_pose = PickObject.create_pre_grasp_pose(PickObject.pose_goal)
                #4 Set e plan del pre grasp pose
                PickObject.set_and_plan_target(PickObject.pre_grasp_pose)
                #5 Cambio stato 
                PickObject.new_pose_received = False
            
            if PickObject.new_trajectory_received:
                #6 Avvio il movimento al pre grasp pose
                PickObject.execute_trajectory()
                # Pubblica la trasformazione for debug
                pose_tf = [PickObject.pre_grasp_pose.position.x, PickObject.pre_grasp_pose.position.y, PickObject.pre_grasp_pose.position.z]
                orientation = [PickObject.pre_grasp_pose.orientation.x, PickObject.pre_grasp_pose.orientation.y, PickObject.pre_grasp_pose.orientation.z, PickObject.pre_grasp_pose.orientation.w]
                PickObject.br.sendTransform(pose_tf, orientation, rospy.Time.now(),"pre_grasp_frame", "base_link")   


 

            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted. Shutting down gracefully.")