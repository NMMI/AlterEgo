import rospy
from geometry_msgs.msg import Quaternion, Pose
from std_msgs.msg import String, Bool, Float64
import json
import tf
import math
import numpy as np
import time
import random
import os

# Classe PID per il controllo proporzionale-integrale-derivativo
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp  # Guadagno proporzionale
        self.Ki = Ki  # Guadagno integrale
        self.Kd = Kd  # Guadagno derivativo
        self.setpoint = setpoint  # Valore desiderato
        self.prev_error = 0  # Errore precedente
        self.integral = 0  # Somma degli errori passati
        self.last_time = time.time()  # Tempo dell'ultima misurazione

    def compute(self, measurement):
        current_time = time.time()
        dt = current_time - self.last_time  # Intervallo di tempo
        error = self.setpoint - measurement  # Errore attuale
        self.integral += error * dt  # Calcolo della parte integrale
        derivative = (error - self.prev_error) / dt if dt > 0 else 0  # Calcolo della parte derivativa
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative  # Uscita del PID
        self.prev_error = error  # Aggiorna l'errore precedente
        self.last_time = current_time  # Aggiorna il tempo dell'ultima misurazione
        return output

# Classe FaceTracker per il tracciamento della faccia
class FaceTracker:
    def __init__(self):
        # Dimensioni dell'immagine
        self.IMG_WIDTH = 672
        self.IMG_HEIGHT = 376
        self.navigation_status = "SUCCEEDED"

        # Variabili di controllo
        self.pitch_pid = PID(Kp=0.1, Ki=0.2, Kd=0.001)  # PID per il pitch
        self.yaw_pid = PID(Kp=0.1, Ki=0.5, Kd=0.001)  # PID per lo yaw
        self.error_yaw_ = 0  # Errore sull'asse x
        self.error_pitch_ = 0  # Errore sull'asse y

        self.head_pose_target = Pose()  # Posa target della testa
        self.pitch_ = 0.0  # Pitch attuale
        self.yaw_ = 0.0  # Yaw attuale
        self.first_pitch_ = False  # Flag per il primo pitch
        self.pitch_offset_ = 0.0  # Offset del pitch

        # Filtro passa-basso
        self.alpha = 0.1  # Coefficiente di smorzamento (0 < alpha <= 1)
        self.filtered_pitch_ = 0.0  # Pitch filtrato
        self.filtered_yaw_ = 0.0  # Yaw filtrato

        # Filtro passa-basso per gli errori
        self.filtered_error_pitch_ = 0.0  # Errore di pitch filtrato
        self.filtered_error_yaw_ = 0.0  # Errore di yaw filtrato

        # Timer per il rilevamento della faccia
        self.last_detection_time = time.time()

        # Contatore per il tempo senza rilevamento della faccia
        self.no_face_counter = 0
        self.no_face_threshold = 5 * 100  # 5 secondi a 100 Hz
        self.target_yaw = 0
        self.target_pitch = 0

        # Configurazione di ROS
        rospy.init_node('face_tracker', anonymous=False)
        self.robot_name = os.getenv('ROBOT_NAME', 'robot_alterego3')
        rospy.Subscriber(f'/{self.robot_name}/detections', String, self.tracker_callback)
        rospy.Subscriber(f'/{self.robot_name}/left/meas_neck_shaft', Float64, self.left_meas_neck_shaft_callback)
        rospy.Subscriber(f'/{self.robot_name}/right/meas_neck_shaft', Float64, self.right_meas_neck_shaft_callback)
        rospy.Subscriber(f'/{self.robot_name}/auto_mode_status', Bool, self.auto_mode_status_callback)
        self.head_pose_pub = rospy.Publisher(f'/{self.robot_name}/head/head_pos', Pose, queue_size=10)

        # Inizializza il client del servizio enable_auto_mode_service

        self.rate = rospy.Rate(100)  # 100 Hz
        self.first_pilot_publish = True  # Flag per il primo valore del pilot publisher

        self.target_pitch_home = 10
        self.target_yaw_home = 0
        self.homing_status = 1
        self.auto_mode_status  = False

    # Funzione per applicare il filtro passa-basso
    def low_pass_filter(self, current_value, previous_filtered_value):
        return self.alpha * current_value + (1 - self.alpha) * previous_filtered_value

    # Callback per il sensore del collo sinistro
    def left_meas_neck_shaft_callback(self, msg):
        self.yaw_ = msg.data

    # Callback per il sensore del collo destro
    def right_meas_neck_shaft_callback(self, msg):
        self.pitch_ = -msg.data

    def auto_mode_status_callback(self, msg):
        self.auto_mode_status = msg.data

    # Calcolo dell'errore tra il centro dell'immagine e il centro del bounding box della faccia rilevata
    def compute_error(self, msg):
        center = (self.IMG_WIDTH / 2, self.IMG_HEIGHT / 2)
        x = (msg[0] + msg[2]) / 2
        y = (msg[1] + msg[3]) / 2
        center_box = (x, y)
        error_x = (center_box[0] - center[0]) / center[0]
        error_y = (center_box[1] - center[1]) / center[1]
        return error_x, error_y

    # Callback per il rilevamento della faccia
    def tracker_callback(self, msg):
        msg = json.loads(msg.data)
        if len(msg) == 0:
            # self.no_face_counter += 1
            return
        else:
            self.last_detection_time = time.time()  # Reset del timer al rilevamento della faccia

        # Ordinamento dei bounding box per area e per errore rispetto al centro dell'immagine
        # boxes = sorted(msg.items(), key=lambda item: (item[1][2]-item[1][0])*(item[1][3]-item[1][1]), reverse=True)
        boxes = sorted(msg.items(), key=lambda item: abs(self.IMG_WIDTH/2 - (item[1][0] + item[1][2])/2) + abs(self.IMG_HEIGHT/2 - (item[1][1] + item[1][3])/2), reverse=False)
        target = boxes[0]
        error_x, error_y = self.compute_error(target[1])

        # Imposta un margine di tolleranza per considerare l'errore come zero quando è piccolo
        if abs(error_x) < 0.1:
            error_x = 0
        if abs(error_y) < 0.1:
            error_y = 0
        
        self.error_yaw_ = error_x
        self.error_pitch_ = error_y
        # Se un volto viene rilevato, mantieni il target pitch a 10 gradi fino a quando non viene rilevato un nuovo errore significativo
        if self.homing_status == 2 and (abs(self.error_pitch_) > 0.1 or abs(self.error_yaw_) > 0.1):
            self.homing_status = 1

    # Ciclo principale
    def main_loop(self):
        while not rospy.is_shutdown():

            if self.auto_mode_status:
                pitch = self.pitch_ * 180 / math.pi  # Conversione del pitch in gradi
                yaw = self.yaw_ * 180 / math.pi  # Conversione dello yaw in gradi

                # Applicazione del filtro passa-basso alle misure
                self.filtered_pitch_ = self.low_pass_filter(pitch, self.filtered_pitch_)
                self.filtered_yaw_ = self.low_pass_filter(yaw, self.filtered_yaw_)
                
                # Applicazione del filtro passa-basso agli errori
                self.filtered_error_pitch_ = self.low_pass_filter(self.error_pitch_, self.filtered_error_pitch_)
                self.filtered_error_yaw_ = self.low_pass_filter(self.error_yaw_, self.filtered_error_yaw_)

                # Calcolo dell'uscita del PID
                self.pitch_pid.setpoint = -self.filtered_error_pitch_ * 180 / math.pi
                self.yaw_pid.setpoint = -self.filtered_error_yaw_ * 180 / math.pi
                self.target_pitch = self.pitch_pid.compute(self.filtered_pitch_)
                self.target_yaw = self.yaw_pid.compute(self.filtered_yaw_)

                # Saturazione del pitch tra 30 e -30 gradi
                self.target_pitch = max(min(self.target_pitch, 25), -25)

                # Saturazione dello yaw tra 30 e -30 gradi
                self.target_yaw = max(min(self.target_yaw, 30), -30)
                
                # print("---------------------")
                # print("pitch" , self.filtered_pitch_)
                # print("yaw" , self.filtered_yaw_)
                # print("target_pitch" , self.target_pitch)
                # print("target_yaw" , self.target_yaw)
                # print("error_pitch" , self.filtered_error_pitch_)
                # print("error_yaw" , self.filtered_error_yaw_)
                # # Controllo se sono passa  ti più di 5 secondi dall'ultimo rilevamento della faccia
                if (time.time() - self.last_detection_time > 5):
                    if(self.homing_status == 1):
                        self.error_pitch_ = 0
                        self.error_yaw_ = 0
                        self.target_pitch *= 0.8
                        self.target_yaw *= 0.8
                    #     if (abs(self.target_yaw) < 0.5 and abs(self.target_pitch) < 3):
                    #         self.homing_status = 2
                    # elif self.homing_status == 2:
                    #     self.target_pitch = 10



                
                # Conversione della posa target della testa (in gradi) in quaternione
                head_pose_quat_target = tf.transformations.quaternion_from_euler(0, -self.target_pitch * math.pi / 180, self.target_yaw * math.pi / 180)

                # Creazione della posa interpolata
                interp_pose = Pose()
                interp_pose.position.x = 0
                interp_pose.position.y = 0
                interp_pose.position.z = 0
                interp_pose.orientation.x = head_pose_quat_target[0]
                interp_pose.orientation.y = head_pose_quat_target[1]
                interp_pose.orientation.z = head_pose_quat_target[2]
                interp_pose.orientation.w = head_pose_quat_target[3]
                        
                # Pubblicazione della posa interpolata
                self.head_pose_pub.publish(interp_pose)


            self.rate.sleep()

if __name__ == '__main__':
    face_tracker = FaceTracker()
    try:
        face_tracker.main_loop()
    except rospy.ROSInterruptException:
        pass