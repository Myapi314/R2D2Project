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

        # Setup serial connection
        self.arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.arduino.reset_input_buffer()
        self.get_logger().info("Serial Server Running")

        self.stop_head = False
        self.projector_is_on = False

        # Start up sequence
        self.red_led_on()
        self.go_forward()
        self.send_string('hl', secs=2)
        self.send_string('hr', secs=2)
        self.send_string('hs')
        self.purple_led_on()
        self.send_string('d')
        call(["aplay", "/home/redleader/ros2_ws/src/my_id_robot/my_id_robot/sounds/R2D2idea.wav"])

    def listener_callback(self, msg):
        self.get_logger().info('Arduino message "%s"' % msg.data)
            
        if msg.data == "forward":
            self.go_forward()
            # call(["aplay", "/home/redleader/ros2_ws/src/my_id_robot/my_id_robot/sounds/R2D2-relax.wav"])
        elif msg.data == "stop":
            self.stop()
        elif msg.data == "roam":
            self.start_roaming_mode()
        elif msg.data == "wander":
            self.start_roaming_mode()
        elif msg.data == "explore":
            self.start_roaming_mode()
        elif msg.data == "right":
            self.go_right()
        elif msg.data == "left":
            self.go_left()
        elif msg.data == "back":
            self.go_backward()
        elif msg.data == "projector":
            self.set_proj()
        elif msg.data == "video":
            self.set_proj("on")
            self.stop()
        elif msg.data == "turn head":
            self.send_string('hl')
        elif msg.data == "led":
            self.yellow_led_on()
            self.red_led_on()
        elif msg.data == "spin":
            self.go_left()
            self.red_led_on()
            self.green_led_on()
            call(["aplay", "/home/redleader/ros2_ws/src/my_id_robot/my_id_robot/sounds/EasterEggs/cantina.wav"])
            # self.toggle_leds()

    def sensor_callback(self, msg: Sensor):
        if msg.avoid and (
            msg.pin == 9 
            or msg.pin == 10 
            or msg.pin == 11
            or msg.pin == 15):
            # self.stop()
            pass

        if msg.avoid and (
            msg.pin == 20
            or msg.pin == 21
        ):
            self.send_string('hs')
            self.stop_head = True
        else:
            self.stop_head = False

    def start_roaming_mode(self):
        self.get_logger().info('Starting Roaming mode...')
        self.arduino.write(b"rs\n")

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
        instruction = instruction + '\n'
        self.arduino.write(instruction.encode('utf-8'))
        time.sleep(secs)

    def red_led_on(self):
        self.arduino.write(b"lbo\n")
        self.get_logger().info('Turning on RED LED...')
        self.arduino.write(b"lrh\n")

    def blue_led_on(self):
        self.get_logger().info('Turning on BLUE LED...')
        self.arduino.write(b"lbh\n")

    def green_led_on(self):
        self.arduino.write(b"lyo\n")
        self.get_logger().info('Turning on GREEN LED...')
        self.arduino.write(b"lgh\n")

    def purple_led_on(self):
        self.get_logger().info('Turning on PURPLE LED...')
        self.arduino.write(b"lrh\n")
        self.arduino.write(b"lbh\n")

    def yellow_led_on(self):
        self.get_logger().info('Turning on YELLOW LED...')
        self.arduino.write(b"lyh\n")

    def toggle_leds(self):
        led_states = ["lrh", "lro", "lbh", "lbo", "lgh", "lgo", "lyh", "lyo", "lrh", "lbh", "lgh", "lyh"]
        for i in range(len(led_states)):
            string = led_states[i] + "\n"
            self.arduino.write(string.encode('utf-8'))
            time.sleep(1)

    def turn_off_leds(self):
        self.get_logger().info('Turning off lights!')
        self.arduino.write(b"lo\n")

    def stop(self):
        self.get_logger().info("Tell motors to STOP")
        self.arduino.write(b"s\n")
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
        self.arduino.write(b"wu\n")

    def slow_down(self):
        self.get_logger().info("Tell motors to SLOW DOWN...")
        self.arduino.write(b"wd\n")

    def set_proj(self, command = "toggle"):
        if (command == "toggle"):
            if (self.projector_is_on):
                self.arduino.write(b"p0\n")
            else:
                self.arduino.write(b"p1\n")
            
            self.projector_is_on = 1 - self.projector_is_on
        elif (command == "on"):
            self.arduino.write(b"p1\n")
            self.projector_is_on = True
        else:
            self.arduino.write(b"p0\n")
            self.projector_is_on = False
        self.get_logger().info("Command to projector: " + command)
        


def main(args=None):
    rclpy.init(args=args)
    serial_server = SerialServer()

    try:
        rclpy.spin(serial_server)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        serial_server.turn_off_everything()
        serial_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()