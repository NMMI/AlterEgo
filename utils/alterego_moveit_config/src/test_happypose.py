#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from happypose.toolbox.inference.example_inference_utils import (
    load_detections,
    make_poses_visualization,
)
from happypose.toolbox.datasets.scene_dataset import CameraData

class HappyPoseNode:
    def __init__(self):
        rospy.init_node("happypose_node")

        # Publisher e Subscriber
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)

        self.bridge = CvBridge()
        self.camera_intrinsics = None
        self.latest_image = None

    def camera_info_callback(self, msg):
        self.camera_intrinsics = msg.K

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"Errore durante la conversione dell'immagine: {e}")

    def run_inference(self):
        if self.latest_image is None or self.camera_intrinsics is None:
            rospy.logwarn("Dati della fotocamera non disponibili.")
            return

        # Carica i rilevamenti
        detections = load_detections("/path/to/your/example")

        # Esegui visualizzazione
        make_poses_visualization(
            rgb=self.latest_image,
            object_dataset=None,  # Inserisci il tuo dataset
            object_datas=detections,
            camera_data=CameraData.from_intrinsics(self.camera_intrinsics),
            example_dir="/path/to/your/example",
        )

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.run_inference()
            rate.sleep()

if __name__ == "__main__":
    node = HappyPoseNode()
    node.run()
