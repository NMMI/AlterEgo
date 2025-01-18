#!/usr/bin/env python3
import rospy
from alterego_msgs.msg import DetectedObjectInfo
from geometry_msgs.msg import Point, Quaternion
from gazebo_msgs.srv import GetModelState, GetWorldProperties
import os
import re

class fake_ObjectPosePublisher:
    def __init__(self):
        # Inizializzare il nodo ROS
        rospy.init_node('object_pose_publisher', anonymous=False)
        self.robot_name = os.getenv('ROBOT_NAME', 'robot_alterego3')
        
        # Publisher per l'oggetto
        self.pub = rospy.Publisher(f"/{self.robot_name}/object_info", DetectedObjectInfo, queue_size=10)
        
        # Attendere che il servizio sia disponibile
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        rospy.loginfo("Servizio /gazebo/get_model_state pronto")

        rospy.wait_for_service('/gazebo/get_world_properties')
        self.get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        rospy.loginfo("Servizio /gazebo/get_world_properties pronto")

        # Ottenere e stampare la lista dei modelli presenti nella scena di Gazebo
        try:
            response = self.get_world_properties()
            if response.success:
                self.model_names = response.model_names
                print("Modelli presenti nella scena di Gazebo:")
                for model_name in self.model_names:
                    print(f"- {model_name}")
            else:
                rospy.logwarn("Non è stato possibile ottenere la lista degli oggetti.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Errore nel chiamare il servizio: {e}")

    def publish_object_pose(self, object_name):
        rospy.loginfo(f"Richiesta posa per l'oggetto: {object_name}")
        
        try:
            # Richiedere la posa dell'oggetto rispetto al frame base_link del robot
            response = self.get_model_state(object_name, "ego_robot::base_link")
            if response.success:
                # Crea il messaggio di posa dell'oggetto
                object_msg = DetectedObjectInfo()
                object_msg.object_name = object_name
                object_msg.position = response.pose.position
                object_msg.orientation = response.pose.orientation

                # Pubblica la posa dell'oggetto
                self.pub.publish(object_msg)
                rospy.loginfo(f"Pubblicata la posa di {object_name} rispetto a ego_robot::base_link")
            else:
                rospy.logwarn(f"Oggetto '{object_name}' non trovato in Gazebo")
        
        except rospy.ServiceException as e:
            rospy.logerr(f"Errore nel chiamare il servizio: {e}")

    def run(self):
        while not rospy.is_shutdown():
            # Mostra i modelli disponibili e chiedi all'utente di inserire il nome
            object_name = input("Inserisci il nome dell'oggetto da pubblicare (oppure 'exit' per uscire): ").strip()
            if object_name.lower() == 'exit':
                break
            elif object_name in self.model_names:
                self.publish_object_pose(object_name)
            else:
                rospy.logwarn(f"Oggetto '{object_name}' non presente nella scena di Gazebo. Riprova.")

if __name__ == "__main__":
    try:
        object_pose_publisher = fake_ObjectPosePublisher()
        object_pose_publisher.run()
    except rospy.ROSInterruptException:
        pass
