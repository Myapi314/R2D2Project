#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
from subprocess import call
from std_msgs.msg import String
from my_id_robot_interfaces.msg import Sensor

class SerialServer(Node):

    def __init__(self):
        super().__init__('arduino_subscriber')

        self.subscription = self.create_subscription(
            String,
            'arduino',
            self.listener_callback,
            10
        )
        self.subscription   # prevent unused variable warning

        self.sensor_subscription =self.create_subscription(
            Sensor,
            'sensor',
            self.sensor_callback,
            10
        )

        self.get_logger().info("Serial Server Running")

        # Setup serial connection
        self.arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.arduino.reset_input_buffer()

    def listener_callback(self, msg):
        self.get_logger().info('Arduino message "%s"' % msg.data)
        if msg.data == "BOOT UP SEQ":
            # Start up sequence
            self.red_led_on()
            self.go_forward(secs=0.5)
            self.stop()
            self.send_string('hl\n', secs=1)
            self.send_string('hr\n', secs=1)
            self.send_string('hs\n')
            self.purple_led_on()
            call(["aplay", "/home/redleader/ros2_ws/src/my_id_robot/my_id_robot/sounds/R2D2a.wav"])
            
        if msg.data == "forward":
            call(["aplay", "/home/redleader/ros2_ws/src/my_id_robot/my_id_robot/sounds/R2D2-relax.wav"])
            self.go_forward()
        elif msg.data == "stop":
            self.stop()
        elif msg.data == "right":
            self.go_right()
        elif msg.data == "left":
            self.go_left()
        elif msg.data == "back": # We need a function call for going backwards, right?
            self.go_backward()

    def sensor_callback(self, msg: Sensor):
        if msg.avoid and (
            msg.pin == 9 
            or msg.pin == 10 
            or msg.pin == 11
            or msg.pin == 15):
            pass
            # self.stop()
        elif msg.avoid and (
            msg.pin == 20
            or msg.pin == 21
        ):
            pass
            # self.send_string('hs\n')

    def turn_off_everything(self):
        self.get_logger().error("Turn off everything!")
        self.arduino.write(b"q\n")

    def receive_msg(self):
        while self.arduino.in_waiting > 0:
            line = self.arduino.readline().decode('utf-8').rstrip()
            self.get_logger().info(line)
        # else:
        #     self.get_logger().info('Nothing incoming')

    def send_string(self, instruction, secs: float = 0):
        self.get_logger().info(f'Send String to Arduino: {instruction}')
        self.arduino.write(instruction.encode('utf-8'))
        time.sleep(secs)

    def red_led_on(self):
        self.get_logger().info('Turning on RED LED...')
        self.arduino.write(b"lrh\n")

    def purple_led_on(self):
        self.get_logger().info('Turning on PURPLE LED...')
        self.arduino.write(b"lbh\n")

    def turn_off_leds(self):
        self.get_logger().info('Turning off lights!')
        self.arduino.write(b"lo\n")

    def toggle_leds(self):
        self.get_logger().info('Enjoy the light show!')
        led = 3
        for i in range(10):
            self.arduino.write(str(led).encode('utf-8'))
            led = led % 2 + 3
            time.sleep(1)

    def move(self, direction):
        self.get_logger().info(f'Arduino move motors {direction}...')
        self.arduino.write(direction.encode('utf-8'))

    def stop(self):
        self.get_logger().info("Tell motors to STOP")
        self.arduino.write(b"ms\n")
        self.receive_msg()

    # Control Wheels
    def go_forward(self, secs: float = 0):
        self.get_logger().info("Tell motors to go FORWARD...")
        self.arduino.write(b"mf\n")
        time.sleep(secs)
        self.receive_msg()

    def go_backward(self):
        self.get_logger().info("Tell motors to go BACKWARD...")
        self.arduino.write(b"mb\n")

    def go_left(self):
        self.get_logger().info("Tell motors to go LEFT...")
        self.arduino.write(b"ml\n")

    def go_right(self):
        self.get_logger().info("Tell motors to go RIGHT...")
        self.arduino.write(b"mr\n")

    def speed_up(self):
        self.get_logger().info("Tell motors to SPEED UP...")
        self.arduino.write(b"su\n")

    def slow_down(self):
        self.get_logger.info("Tell motors to SLOW DOWN...")
        self.arduino.write(b"sd\n")

def main(args=None):
    rclpy.init(args=args)
    serial_server = SerialServer()
    rclpy.spin(serial_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()