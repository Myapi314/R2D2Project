#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
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

        self.arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.arduino.reset_input_buffer()

        # self.create_timer(1.0, self.receive_msg)

    def listener_callback(self, msg):
        self.get_logger().info('Arduino message "%s"' % msg.data)
        if msg.data == "forward":
            self.go_forward()
        elif msg.data == "scan right":
            self.go_right()
        elif msg.data == "scan left":
            self.go_left()

    def sensor_callback(self, msg: Sensor):
        if msg.avoid and (
            msg.pin == 9 
            or msg.pin == 10 
            or msg.pin == 11
            or msg.pin == 15):
            self.stop()
        elif msg.avoid and (
            msg.pin == 20
            or msg.pin == 21
        ):
            self.send_string('hs\n')


    def receive_msg(self):
        while self.arduino.in_waiting > 0:
            line = self.arduino.readline().decode('utf-8').rstrip()
            self.get_logger().info(line)
        # else:
        #     self.get_logger().info('Nothing incoming')
    
    def send_char(self, instruction):
        self.get_logger().info('Sending to Arduino ' + instruction + '...')
        self.arduino.write(instruction.encode('utf-8'))

    def send_string(self, instruction):
        self.get_logger().info(f'Send String to Arduino: {instruction}')
        self.arduino.write(instruction.encode('utf-8'))

    def red_led_on(self):
        self.get_logger().info('Turning on RED LED...')
        self.arduino.write(str(3).encode('utf-8'))

    def turn_off(self):
        self.get_logger().info('Turning off lights!')
        self.arduino.write(str(-1).encode('utf-8'))

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

    def go_forward(self):
        self.get_logger().info("Tell motors to go FORWARD...")
        self.arduino.write(b"mf\n")
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