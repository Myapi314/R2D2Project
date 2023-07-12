#!/usr/bin/env python3
import rclpy
from subprocess import call
from rclpy.node import Node
from std_msgs.msg import String
from time import sleep

class ProjectorSubscriber(Node):

    def __init__(self):
        super().__init__('projector_subscriber')

        self.subscription = self.create_subscription(
            String,
            'projector',
            self.listener_callback,
            10
        )
        self.subscription   # prevent unused variable warning

        self.get_logger().info("Projector node running!")

    def listener_callback(self, msg):
        self.get_logger().info('Message "%s"' % msg.data)

    def reset_projector(self):
        call(["xset", "-display", ":0.0", "dpms", "force", "off"])
        sleep(2)
        call(["xset", "-display", ":0.0", "dpms", "force", "on"])

    def play_video(self, video_name = "leia-only-hope"):
        # Need to check to see if projector is turned on already
        # If not, we need to turn it on, wait for it to turn on,
        # reset the connection to it, then fire up vlc

        call(["vlc", "/home/redleader/ros2_ws/src/my_id_robot/my_id_robot/Videos/'%s'.mp4" % video_name])

def main(args=None):
    rclpy.init(args=args)
    projector_subscriber = ProjectorSubscriber()
    rclpy.spin(projector_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()