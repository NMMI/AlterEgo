#!/usr/bin/env python3
import os
import rospy
from ego_msgs.msg import EgoArms, UpperBodyState
import numpy as np
from std_msgs.msg import Bool

class JointPIDController:
    def __init__(self, kp, ki, kd):
        # Ottieni il nome del robot dalla variabile d'ambiente
        self.robot_name = os.getenv('ROBOT_NAME', 'robot_alterego3')

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = np.zeros(6)
        self.integral = np.zeros(6)
        self.last_time = rospy.Time.now()

        # Publisher per i comandi del motore
        self.command_pub = rospy.Publisher(f'/{self.robot_name}/right/kin_des_jnt_topic', EgoArms, queue_size=10)
        # Subscriber per i riferimenti e le misure
        rospy.Subscriber(f'/{self.robot_name}/right/hat_kin_des_jnt_topic', EgoArms, self.reference_callback)
        rospy.Subscriber(f'/{self.robot_name}/alterego_state/upperbody', UpperBodyState, self.measurement_callback)
        rospy.Subscriber(f'/{self.robot_name}/auto_mode_status', Bool, self.auto_mode__Callback)
        self.reference = np.zeros(6)
        self.measurement = np.zeros(6)
        self.auto_mode_enabled = False
        
    def reference_callback(self, msg):
        self.reference = np.array(msg.q_des)
        # rospy.loginfo(f"Reference received: {self.reference}")

    def measurement_callback(self, msg):
        self.measurement = np.array(msg.right_meas_arm_shaft)  # Assicurati di gestire i dati correttamente
        # rospy.loginfo(f"Measurement received: {self.measurement}")
    
    def auto_mode__Callback(self, msg):
        self.auto_mode_enabled = msg.data
        if self.auto_mode_enabled:
            rospy.loginfo("Auto mode enabled")
        else:
            rospy.loginfo("Auto mode disabled")

    def update(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        if dt <= 0:
            return

        # error = self.reference - self.measurement
        error = self.reference 
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        # Crea e pubblica il messaggio di comando
        des_joint = EgoArms()
        des_joint.q_des = output.tolist()  # Converti l'array numpy in lista per il messaggio

        # Se necessario, puoi aggiungere velocità e accelerazioni, attualmente commentate
        # des_joint.qd_des = [0.0] * 6  # Esempio di velocità desiderata
        # des_joint.qdd_des = [0.0] * 6  # Esempio di accelerazione desiderata
        if self.auto_mode_enabled:
            self.command_pub.publish(des_joint)
        # rospy.loginfo(f"Command published: {des_joint.q_des}")

        self.prev_error = error

if __name__ == '__main__':
    try:
        rospy.init_node('pid_controller_node')
        kp = rospy.get_param('/kp', 1.5)
        ki = rospy.get_param('/ki', 0.1)
        kd = rospy.get_param('/kd', 0.0001)
        rospy.loginfo(f"PID parameters: kp={kp}, ki={ki}, kd={kd}")
        pid_controller = JointPIDController(kp, ki, kd)

        rate = rospy.Rate(100)  # 10 Hz
        while not rospy.is_shutdown():
            pid_controller.update()
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down PID controller node.")