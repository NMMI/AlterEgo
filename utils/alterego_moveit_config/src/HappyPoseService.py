#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Point, Quaternion
from std_srvs.srv import Trigger, TriggerResponse
from alterego_moveit_config.srv import HappyPoseService, HappyPoseServiceResponse
from cv_bridge import CvBridge
import cv2
import numpy as np
from happypose.toolbox.inference.pose_estimator import PoseEstimator

import os

class HappyPoseNode:
    def __init__(self):
        rospy.init_node("happypose_node")

        # Nome del robot
        self.robot_name = os.getenv('ROBOT_NAME', 'robot_alterego3')

        # Configura il dataset di Happypose
        self.happypose_data_dir = os.getenv('HAPPYPOSE_DATA_DIR')
        if not self.happypose_data_dir:
            rospy.logerr("HAPPYPOSE_DATA_DIR non è configurato. Configurare la variabile d'ambiente.")
            exit(1)

        # Configura il rilevatore BOP
        self.bop_inference = BopInference(
            detector="detector-bop-ycbv-pbr--970850",
            coarse="coarse-bop-ycbv-pbr--724183",
            refiner="refiner-bop-ycbv-pbr--604090",
            data_dir=self.happypose_data_dir,
            dataset_name="ycbv"
        )

        # Subscriber per l'immagine RGB e CameraInfo
        self.image_sub = rospy.Subscriber(f"/{self.robot_name}/camera/color/image_raw", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber(f"/{self.robot_name}/camera/color/camera_info", CameraInfo, self.camera_info_callback)

        # Service per richiedere la posa
        self.service = rospy.Service('/happypose_service', HappyPoseService, self.handle_happypose_service)

        # Pubblicatore della posa stimata
        self.pose_pub = rospy.Publisher("/happypose/pose", Pose, queue_size=10)

        self.bridge = CvBridge()
        self.camera_intrinsics = None
        self.latest_image = None

    def camera_info_callback(self, msg):
        """Salva i parametri intrinseci della fotocamera."""
        self.camera_intrinsics = np.array(msg.K).reshape(3, 3)

    def image_callback(self, msg):
        """Riceve l'immagine RGB dalla fotocamera."""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"Errore nella conversione dell'immagine: {e}")

    def handle_happypose_service(self, req):
        """Gestisce le richieste di posa tramite il servizio."""
        if self.latest_image is None or self.camera_intrinsics is None:
            rospy.logwarn("Immagine o parametri intrinseci non disponibili.")
            return TriggerResponse(success=False, message="No image or intrinsics available.")

        rospy.loginfo("Calcolo della posa iniziato...")

        # Calcola la posa usando Happypose
        try:
            results = self.bop_inference.infer_image(
                image=self.latest_image,
                intrinsics=self.camera_intrinsics,
                objects=None  # Puoi specificare un filtro di oggetti YCB
            )

            # Estrai la prima posa stimata (modifica se hai più oggetti)
            if not results:
                rospy.logwarn("Nessun oggetto trovato.")
                return TriggerResponse(success=False, message="No objects detected.")

            pose_result = results[0]
            position = pose_result.translation
            quaternion = pose_result.rotation_quaternion

            # Pubblica la posa
            pose_msg = Pose(
                position=Point(*position),
                orientation=Quaternion(*quaternion)
            )
            self.pose_pub.publish(pose_msg)

            rospy.loginfo("Posa calcolata e pubblicata.")
            return TriggerResponse(success=True, message="Pose estimated successfully.")

        except Exception as e:
            rospy.logerr(f"Errore durante la stima della posa: {e}")
            return TriggerResponse(success=False, message="Pose estimation failed.")

    def run(self):
        rospy.loginfo("Nodo HappyPose avviato.")
        rospy.spin()

if __name__ == "__main__":
    node = HappyPoseNode()
    node.run()
