#!/usr/bin/env python

from video_service.srv import LaunchGStreamer, LaunchGStreamerResponse
import rospy
import subprocess

def handle_launch_gstreamer(req):
    command = [
        "gst-launch-1.0",
        "v4l2src", "device=/dev/video0", "!",
        "videoconvert", "!",
        "video/x-raw,width=2560,height=720,framerate=60/1", "!",
        "tee", "name=t",
        "t.", "!", "queue", "!", "videocrop", "right=1280", "!", "videoconvert", "!", 
        "x264enc", "tune=zerolatency", "bitrate=5000", "speed-preset=ultrafast", "!",
        "rtph264pay", "config-interval=1", "pt=96", "!", "udpsink", "host=192.168.0.95", "port=30000",
        "t.", "!", "queue", "!", "videocrop", "left=1280", "!", "videoconvert", "!", 
        "x264enc", "tune=zerolatency", "bitrate=5000", "speed-preset=ultrafast", "!",
        "rtph264pay", "config-interval=1", "pt=96", "!", "udpsink", "host=192.168.0.95", "port=30001"
    ]

    try:
        subprocess.Popen(command)
        return LaunchGStreamerResponse(success=True)
    except Exception as e:
        rospy.logerr("Failed to launch GStreamer pipeline: %s", e)
        return LaunchGStreamerResponse(success=False)

def launch_gstreamer_server():
    rospy.init_node('launch_gstreamer_server')
    s = rospy.Service('launch_gstreamer', LaunchGStreamer, handle_launch_gstreamer)
    rospy.loginfo("Ready to launch GStreamer pipeline.")
    rospy.spin()

if __name__ == "__main__":
    launch_gstreamer_server()
