import rclpy
from subprocess import call
from rclpy.node import Node
from time import sleep

from std_msgs.msg import String
from my_id_robot_interfaces.msg import Sensor

start_commands = ["hey are too", "hey are to", "hey are two", "hey or too", "hey or to", "hey or two", "he are to", "here to", "he or to", "here are two"]
arduino_commands = ["forward", "left", "right", "back", "stop", "projector", "turn head", "spin", "roam", "explore", "wander"]

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
            Sensor,
            'sensor',
            self.sensor_callback,
            10
        )
        self.opencvpublisher_ = self.create_publisher(String, 'opencv', 10)
        self.servopublisher_ = self.create_publisher(String, 'servo', 10)    
        self.arduinopublisher_ = self.create_publisher(String, 'arduino', 10)
        self.projectorpublisher_ = self.create_publisher(String, 'projector', 10)
        

    def listener_callback(self, msg):
        self.get_logger().info('Main Voice data "%s"' % msg.data)
        # Turn off the microphone
        call(["pactl", "set-source-mute", "1", "toggle"])
#        # Execute espeak using subprocess.run() 

        # repeat msg over audio
        # call(["espeak", "-ven-us+f3", msg.data])

#        call(["espeak", "-ven-us+f3", msg.data])
                #        sleep(2)
        #Turn on the microphone
        call(["pactl", "set-source-mute", "1", "toggle"])

        motor_msg = String()

        # Find key to look for command
        send_command = False
        for i in range(len(start_commands)):
            if (msg.data.find(start_commands[i]) != -1):
                send_command = True
                break

        if (send_command):
            if (msg.data.find("video") != -1):
                # publish projector message
                motor_msg.data = "video"
                self.projectorpublisher_.publish(motor_msg)
                # publish arduino message
                self.arduinopublisher_.publish(motor_msg)
                
            for i in range(len(arduino_commands)):
                if (msg.data.find(arduino_commands[i]) != -1):
                    motor_msg.data = '%s' % arduino_commands[i]
                    self.arduinopublisher_.publish(motor_msg)
                    self.get_logger().info('Main to Arduino: "%s"' % motor_msg.data)
        
        if (msg.data.find("hello") != -1):
            call(["aplay", "/home/redleader/ros2_ws/src/my_id_robot/my_id_robot/sounds/R2D2c.wav"])
            motor_msg.data = "led"
            self.arduinopublisher_.publish(motor_msg)

        if (msg.data.find("stop") != -1):
            motor_msg.data = "stop"
            self.arduinopublisher_.publish(motor_msg)
            self.get_logger().info('Main to Arduino: "%s"' % motor_msg.data)

        if (msg.data.find("thank you") != -1):
            call(["aplay", "/home/redleader/ros2_ws/src/my_id_robot/my_id_robot/sounds/bro-Grimmet.wav"])
   
        elif (msg.data == "find"):
            opencv_msg = String()
            opencv_msg.data = '%s' % msg.data
            self.opencvpublisher_.publish(opencv_msg)
            self.get_logger().info('Main to Opencv: "%s"' % opencv_msg.data)

    def sensor_callback(self, msg):
        # self.get_logger().info(str(msg.pin))
        if msg.avoid:
            self.get_logger().info(f"React to msg from pin {msg.pin}")

        
          
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
