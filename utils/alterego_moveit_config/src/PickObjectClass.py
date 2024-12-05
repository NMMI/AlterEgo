#!/usr/bin/env python3
import sys
import os
import rospy
import moveit_commander
from ego_msgs.msg import EgoArms, DetectedObjectInfo
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool
import tf.transformations as tft
import tf
from moveit_commander import PlanningSceneInterface
import re
from alterego_moveit_config.srv import PickService, PickServiceResponse
import threading
class PickObjectClass:
    def __init__(self):
        # Get robot name from environment variable
        self.robot_name = os.getenv('ROBOT_NAME', 'robot_alterego3')
        rospy.init_node('pick_object_node', anonymous=False)

        self.pick_service = rospy.Service('/pick_service', PickService, self.handle_pick_service)

        # Initialize the ROS node
        moveit_commander.roscpp_initialize(sys.argv)

        # Costruisci il topic dinamico per la point cloud
        point_cloud_topic = f"/{self.robot_name}/camera/depth/color/points"
        rospy.set_param('/move_group/sensors/point_cloud_topic', point_cloud_topic)

        # Subscribers
        self.sub_display_trajectory = rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, self.trajectory_callback)

        # Publishers
        self.pub_ref_eq_des = rospy.Publisher(f"/{self.robot_name}/right/hat_kin_des_jnt_topic", EgoArms, queue_size=10)


        # Initialize the planning group (MoveIt group name)
        self.group_name = "right_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        # Initialize the Planning Scene Interface
        self.scene = PlanningSceneInterface()
        rospy.sleep(1)

        self.move_group.set_max_velocity_scaling_factor(0.05)
        self.move_group.set_max_acceleration_scaling_factor(0.05)
        self.move_group.set_planning_time(10.0)  # Aumenta il tempo di pianificazione a 10 secondi
        self.move_group.set_goal_position_tolerance(0.1)  # Tolleranza di posizione (in metri)
        self.move_group.set_goal_orientation_tolerance(0.15)  # Tolleranza di orientamento (in radianti)


        # Flag to indicate if a new grasping pose has been received
        self.new_pose_received = False

        # Flag to indicate if a new trajectory has been received
        self.new_trajectory_received = False

        # Initialize the planned trajectory
        self.planned_trajectory = None

        # Index to track the current point in the trajectory
        self.current_trajectory_index = 0

        # Initialize the end effector pose
        self.end_effector_pose = Pose()

        # Flag to indicate if the end effector pose has been published
        self.pose_pub = False

        # Path to the YCB models
        self.ycb_models_path = "/home/grosato/ALTEREGO/catkin_ws/src/AlterEGO_v2/utils/ycb-tools/models/ycb"

        self.br = tf.TransformBroadcaster()

        # Condition variable to synchronize the request and the movement
        self.condition = threading.Condition()

        self.object_picked = False

    
    def handle_pick_service(self,req):
        # Implementa qui la logica per gestire il pick 
        rospy.loginfo(f"Received request to pick {req.object} at position ({req.position.x}, {req.position.y}, {req.position.z}) and orientation ({req.orientation.x}, {req.orientation.y}, {req.orientation.z}, {req.orientation.w})")

        self.received_object = req.object
        self.received_position = [req.position.x, req.position.y, req.position.z]
        self.received_orientation = [req.orientation.x, req.orientation.y, req.orientation.z, req.orientation.w]
        self.new_pose_received = True
        
        # Wait for picking the object
        with self.condition:
            self.condition.wait_for(lambda: self.object_picked)
        success = True  # Imposta a False se l'azione fallisce

        return PickServiceResponse(success=success)

    def create_pose(self, position, orientation):
        """Crea un oggetto Pose da posizione e orientamento"""
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
        return pose


    def create_pre_grasp_pose(self, pose_goal):
        """Crea un oggetto Pre Pose da posizione e orientamento del goal"""
        pose = Pose()
        # pose.position.x = 0.25
        # pose.position.y = -0.25
        pose.position.x = pose_goal.position.x
        pose.position.y = pose_goal.position.y
        pose.position.z = pose_goal.position.z 
        # set euler angles and trasform them to quaternion
        orientation = tft.quaternion_from_euler( -1.57, 0.0, 0.0)
        euler = tft.euler_from_quaternion(orientation)
        rospy.logwarn(f"PreGrasp: {euler}")

        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]

        return pose



    def trajectory_callback(self, msg):
        """Callback per ricevere la traiettoria pianificata"""
        rospy.loginfo("Received planned trajectory")
        self.new_trajectory_received = True
        # Verifica che il messaggio contenga almeno una traiettoria per evitare errori quando si accede alla traiettoria.
        self.planned_trajectory = msg.trajectory[0].joint_trajectory
        self.current_trajectory_index = 0


    def execute_trajectory(self):
        if self.current_trajectory_index < len(self.planned_trajectory.points):
            point = self.planned_trajectory.points[self.current_trajectory_index]
            self.ego_joint_publisher(point)
            self.current_trajectory_index += 1
            rospy.loginfo(f"Trajectory point {self.current_trajectory_index}: {point.positions}")
        else:
            self.new_trajectory_received = False
            rospy.logwarn("End of trajectory reached")

            # Notify that the object has been picked
            self.object_picked = True
            with self.condition:
                self.condition.notify_all()


    def ego_joint_publisher(self, point):
        """Pubblica i riferimenti di posizione dei giunti"""
        rospy.loginfo("Publishing planned trajectory point")
        des_joint = EgoArms()
        des_joint.q_des = point.positions
        # Questo publisher invia le posizioni dei giunti ai motori del braccio, ma i motori potrebbero non seguire esattamente i riferimenti.
        self.pub_ref_eq_des.publish(des_joint)


    def set_and_plan_target(self, pre_grasp_pose):
        rospy.loginfo("New grasping pose received")

        self.move_group.set_pose_target(pre_grasp_pose)
        #Debug
        success, plan, _, _ = self.move_group.plan()
        if success:
            rospy.loginfo("Planning successful, new goal set")
        else:
            rospy.logerr("Planning failed")
        self.new_pose_received = False
    #-----------------------------------------------------------------------------------------


    def get_full_model_path(self, object_name):
        """
        Cerca il file modello corrispondente al nome dell'oggetto, ignorando il prefisso numerico.
        :param object_name: Nome dell'oggetto senza prefisso numerico, ad esempio 'mug'.
        :return: Percorso completo del file modello, o None se non trovato.
        """
        try:
            # Itera attraverso tutte le cartelle della directory ycb-models
            for folder_name in os.listdir(self.ycb_models_path):
                # Usa una regex per trovare una corrispondenza con il nome dell'oggetto senza considerare il prefisso numerico
                if re.search(rf'\d+_{object_name}$', folder_name):
                    model_path = os.path.join(self.ycb_models_path, folder_name, "google_16k", "textured.obj")
                    if os.path.exists(model_path):
                        return model_path
            # Se non viene trovato nessun modello corrispondente
            rospy.logwarn(f"Modello per l'oggetto '{object_name}' non trovato")
            return None
        except Exception as e:
            rospy.logerr(f"Errore nella ricerca del modello per l'oggetto '{object_name}': {e}")
            return None

    def add_object_to_scene(self, object_name):
        self.model_path = self.get_full_model_path(object_name)
        self.add_ycb_object_to_scene(object_name, self.pose_goal)


    def remove_object_from_octomap(self, object_name):
        """
        Rimuove un'area dalla Octomap usando un box temporaneo con le dimensioni reali dell'oggetto.
        :param object_name: Nome dell'oggetto da rimuovere senza prefisso numerico.
        :param position: Posizione [x, y, z] del box da rimuovere.
        """
        # Trova il percorso del modello
        self.model_path = self.get_full_model_path(object_name)
        # Aggiungi un box temporaneo alla scena
        self.add_ycb_object_to_scene(object_name, self.pose_goal)
        rospy.sleep(0.5)  # Pausa per permettere l'aggiornamento della scena
        # Rimuovi il box per svuotare l'area corrispondente nella Octomap
        self.scene.remove_world_object(object_name)
        rospy.loginfo(f"Oggetto {object_name} rimosso dalla Octomap")
        # Da testare come graspare questo oggetto senza ostacoli intorno
        self.add_ycb_object_to_scene(object_name, self.pose_goal)

    
    def add_ycb_object_to_scene(self, object_name, pose_goal):
        """Aggiungi un oggetto YCB alla scena come ostacolo"""
        object_pose = PoseStamped()
        object_pose.header.frame_id = "base_link"
        object_pose.pose = pose_goal
        rospy.loginfo(f"\nOggetto {object_name} aggiunto alla scena alla posizione \n{pose_goal.position} \ncon orientamento \n{pose_goal.orientation}")
        # Assumendo che tu abbia i modelli YCB scaricati
        self.scene.add_mesh(object_name, object_pose, self.model_path)


    def shutdown(self):
        """Funzione di chiusura"""
        moveit_commander.roscpp_shutdown()








