#!/usr/bin/env python

import rospy

def shutdown_hook():

    rospy.init_node('shutdown_hook', anonymous=True)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        print("alive!")        
        rate.sleep()

def shutdown_callback():
    rospy.loginfo("It's shutdown time!")


if __name__ == '__main__':

    rospy.on_shutdown(shutdown_callback)
    shutdown_hook()
