#!/usr/bin/env python3

import rospy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
from math import sqrt, atan2, degrees, acos, cos, sin
from visualization_msgs.msg import Marker, MarkerArray

class KeyPointSender:
    def __init__(self):
        rospy.init_node('send_key_points')
        self.current_pose = None
        # what to do if shut down (e.g. ctrl + C o failure)
        rospy.on_shutdown(self.shutdown)

        self.key_points = self.load_key_points(rospy.get_param('navigation/KeyPoints'))
        self.pub = rospy.Publisher('nearest_point', String, queue_size=10)
        self.marker_pub = rospy.Publisher('keypoint_markers_array', MarkerArray, queue_size=10)
        self.threshold = 2  # Soglia di distanza in metri
        self.angle_threshold = 60.0  # Soglia di angolo in gradi (30 gradi diviso per 2)
        self.last_published_point = None  # Variabile per memorizzare l'ultimo punto pubblicato

    def load_key_points(self, key_points_param):
        key_points = []
        for location in key_points_param:
            for name, data in location.items():
                position = data['position']
                orientation = data['orientation']
                key_points.append({
                    'name': name,
                    'position': position,
                    'orientation': orientation
                })
        return key_points

    def calculate_distance(self, pose1, pose2):
        dx = pose1.position.x - pose2['x']
        dy = pose1.position.y - pose2['y']
        dz = pose1.position.z - pose2['z']
        return sqrt(dx*dx + dy*dy + dz*dz)

    def quaternion_to_yaw(self, orientation):
        """Convert a quaternion into a yaw angle (in radians)."""
        return atan2(2.0 * (orientation['w'] * orientation['z'] + orientation['x'] * orientation['y']),
                     1.0 - 2.0 * (orientation['y'] * orientation['y'] + orientation['z'] * orientation['z']))

    def calculate_angle(self, pose1, pose2, orientation2):
        # Vettore dal keypoint alla posa del robot
        dx = pose1.position.x - pose2['x']
        dy = pose1.position.y - pose2['y']
        
        # Direzione del keypoint
        keypoint_yaw = self.quaternion_to_yaw(orientation2)
        keypoint_dir_x = cos(keypoint_yaw)
        keypoint_dir_y = sin(keypoint_yaw)
        
        # Prodotto scalare
        dot_product = dx * keypoint_dir_x + dy * keypoint_dir_y
        mag_robot = sqrt(dx**2 + dy**2)
        mag_keypoint = sqrt(keypoint_dir_x**2 + keypoint_dir_y**2)
        
        # Calcolo dell'angolo
        angle = degrees(acos(dot_product / (mag_robot * mag_keypoint)))
        return angle

    def find_point_within_threshold(self):
        for key_point in self.key_points:
            distance = self.calculate_distance(self.current_pose.pose.pose, key_point['position'])
            angle = self.calculate_angle(self.current_pose.pose.pose, key_point['position'], key_point['orientation'])
            if distance <= self.threshold and angle <= self.angle_threshold:
                return key_point
        return None

    def normalize_quaternion(self, q):
        norm = sqrt(q['x']**2 + q['y']**2 + q['z']**2 + q['w']**2)
        return {
            'x': q['x'] / norm,
            'y': q['y'] / norm,
            'z': q['z'] / norm,
            'w': q['w'] / norm
        }

    def create_markers(self):
        marker_array = MarkerArray()
        for i, key_point in enumerate(self.key_points):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "keypoints"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = key_point['position']['x']
            marker.pose.position.y = key_point['position']['y']
            marker.pose.position.z = key_point['position']['z']
            normalized_orientation = self.normalize_quaternion(key_point['orientation'])
            marker.pose.orientation.x = normalized_orientation['x']
            marker.pose.orientation.y = normalized_orientation['y']
            marker.pose.orientation.z = normalized_orientation['z']
            marker.pose.orientation.w = normalized_orientation['w']
            marker.scale.x = 1.0  # Lunghezza della freccia
            marker.scale.y = 0.1  # Larghezza della freccia
            marker.scale.z = 0.1  # Altezza della freccia
            marker.color.a = 1.0  # Opacità
            marker.color.r = 1.0  # Colore rosso
            marker.color.g = 0.0  # Colore verde
            marker.color.b = 0.0  # Colore blu
            marker_array.markers.append(marker)
        return marker_array

    def run(self):
        rate = rospy.Rate(1)  # Frequenza di pubblicazione (1 Hz)
        while not rospy.is_shutdown():
            self.current_pose = rospy.wait_for_message('amcl_pose', PoseWithCovarianceStamped)
            point_within_threshold = self.find_point_within_threshold()

            if point_within_threshold and point_within_threshold != self.last_published_point:
                print("Punto di interesse entro la soglia trovato: " + point_within_threshold['name'])
                self.pub.publish(point_within_threshold['name'])
                self.last_published_point = point_within_threshold  # Aggiorna l'ultimo punto pubblicato

            # Pubblica i marker
            marker_array = self.create_markers()
            self.marker_pub.publish(marker_array)

            rate.sleep()  # Attendi per mantenere la frequenza di pubblicazione

    def shutdown(self):
        rospy.loginfo("Stop")

if __name__ == '__main__':
    try:
        sender = KeyPointSender()
        sender.run()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")