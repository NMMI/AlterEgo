import rospy
import rosbag
import rospkg
import os
from std_msgs.msg import String
from std_msgs.msg import Bool

#!/usr/bin/env python

class RosbagPlayer:
    def __init__(self):
        self.robot_name = os.getenv('ROBOT_NAME', 'robot_alterego3')

        rospy.init_node('rosbag_play', anonymous=False)
        self.rate = rospy.Rate(100)  # 100 Hz
        self.publishers = {}
        rospy.Subscriber(f'/{self.robot_name}/auto_mode_status', Bool, self.auto_mode_status_callback)

        rospack = rospkg.RosPack()
        package_path = rospack.get_path('alterego_rosbags_play')
        bag_name = rospy.get_param('~bag_name', 'breath.bag')
        self.bag_file = os.path.join(package_path, 'rosbags', bag_name)

        self.auto_mode_status = False
        self.messages = []

    def auto_mode_status_callback(self, msg):
        self.auto_mode_status = msg.data
        print(f"Auto mode status: {self.auto_mode_status}")

    def load_bag(self):
        with rosbag.Bag(self.bag_file, 'r') as bag:
            for topic, msg, t in bag.read_messages():
                # Replace the namespace with the robot_name
                new_topic = f'/{self.robot_name}' +'/'+ topic.split('/', 2)[-1]
                self.messages.append((new_topic, msg))

    def play_rosbag(self):
        while not rospy.is_shutdown():
            if self.auto_mode_status:
                for topic, msg in self.messages:
                    if not self.auto_mode_status:
                        rospy.loginfo("Auto mode is off. Pausing playback.")
                        break

                    if rospy.is_shutdown():
                        break
                    if topic not in self.publishers:
                        self.publishers[topic] = rospy.Publisher(topic, type(msg), queue_size=10)
                    self.publishers[topic].publish(msg)
                    self.rate.sleep()  # Sleep to maintain the 100 Hz rate
            else:
                rospy.sleep(0.1)  # Sleep for a short time to allow callback processing

if __name__ == '__main__':
    try:
        player = RosbagPlayer()
        player.load_bag()
        player.play_rosbag()
    except rospy.ROSInterruptException:
        pass