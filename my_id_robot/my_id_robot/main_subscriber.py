import rclpy
from subprocess import call
from rclpy.node import Node
from time import sleep

from std_msgs.msg import String


class MainSubscriber(Node):

    def __init__(self):
        super().__init__('main_subscriber')
        self.subscription = self.create_subscription(
            String,
            'voice',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.ir_subscription =self.create_subscription(
            String,
            'sensor',
            self.sensor_callback,
            10
        )
        self.opencvpublisher_ = self.create_publisher(String, 'opencv', 10)
        self.servopublisher_ = self.create_publisher(String, 'servo', 10)    
        self.arduinopublisher_ = self.create_publisher(String, 'arduino', 10)

    def listener_callback(self, msg):
        self.get_logger().info('Main Voice data "%s"' % msg.data)
        # Turn off the microphone
        call(["pactl", "set-source-mute", "1", "toggle"])
#        # Execute espeak using subprocess.run() 
        call(["espeak", "-ven-us+f3", msg.data])
#        call(["espeak", "-ven-us+f3", msg.data])
                #        sleep(2)
        #Turn on the microphone
        call(["pactl", "set-source-mute", "1", "toggle"])
        if (msg.data == "hello"):
            call(["aplay", "/home/redleader/ros2_ws/src/my_id_robot/my_id_robot/sounds/R2D2.wav"])
        elif (msg.data == "reach"):
            servo_msg = String()
#            servo_msg.data = '%s' % msg.data
#            self.servopublisher_.publish(servo_msg)
            self.get_logger().info('Main to servo: "%s"' % servo_msg.data)
        elif (msg.data == "wave"):
            servo_msg = String()
#            servo_msg.data = '%s' % msg.data
#            self.servopublisher_.publish(servo_msg)
            self.get_logger().info('Main to servo: "%s"' % servo_msg.data)      
        elif (msg.data == "find"):
            opencv_msg = String()
            opencv_msg.data = '%s' % msg.data
            self.opencvpublisher_.publish(opencv_msg)
            self.get_logger().info('Main to Opencv: "%s"' % opencv_msg.data)
        elif (msg.data == "scan right"):
            motor_msg = String()
            motor_msg.data = '%s' % msg.data
            self.arduinopublisher_.publish(motor_msg)
            self.get_logger().info('Main to Motor: "%s"' % motor_msg.data)
        elif (msg.data == "scan left"):
            motor_msg = String()
            motor_msg.data = '%s' % msg.data
            self.arduinopublisher_.publish(motor_msg)
            self.get_logger().info('Main to Motor: "%s"' % motor_msg.data)
        elif (msg.data == "forward"):
            motor_msg = String()
            motor_msg.data = '%s' % msg.data
            self.arduinopublisher_.publish(motor_msg)
            self.get_logger().info('Main to Motor: "%s"' % motor_msg.data)

    def sensor_callback(self, msg):
        self.get_logger().info(msg.data)
          
def main(args=None):
    rclpy.init(args=args)

    main_subscriber = MainSubscriber()

    rclpy.spin(main_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    main_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
