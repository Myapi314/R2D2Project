#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ProjectorSubscriber(Node):

    def __init__(self):
        super().__init__('projector_subscriber')

        self.subscription = self.create_subscription(
            String,
            'projetor',
            self.listener_callback,
            10
        )
        self.subscription   # prevent unused variable warning

        self.get_logger().info("Projector node running!")

    def listener_callback(self, msg):
        self.get_logger().info('Message "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    projector_subscriber = ProjectorSubscriber()
    rclpy.spin(projector_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()