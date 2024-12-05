#!/usr/bin/env python

from video_service.srv import CloseGStreamer, CloseGStreamerResponse
import rospy
import subprocess

def handle_close_gstreamer(req):
    command = ["killall", "-9", "gst-launch-1.0"]

    try:
        subprocess.Popen(command)
        return CloseGStreamerResponse(success=True)
    except Exception as e:
        rospy.logerr("Failed to launch GStreamer pipeline: %s", e)
        return CloseGStreamerResponse(success=False)

def close_gstreamer_server():
    rospy.init_node('close_gstreamer_server')
    s = rospy.Service('close_gstreamer', CloseGStreamer, handle_close_gstreamer)
    rospy.loginfo("Waiting to close GStreamer pipeline.")
    rospy.spin()

if __name__ == "__main__":
    close_gstreamer_server()
