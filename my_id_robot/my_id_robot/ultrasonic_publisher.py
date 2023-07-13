import serial
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
import random
from my_id_robot_interfaces.msg import Sensor

TX_PIN = 14
RX_PIN = 15

DETECT_DIST = 50

class UltrasonicPublisher(Node):
    last_string = "None"
    def __init__(self):
        super().__init__('ultrasonic_publisher')
        # self.publisher_ = self.create_publisher(String, 'voice', 10)
        self.get_logger().info("Ultrasonic node running!")
        self.publisher_ = self.create_publisher(Sensor, 'sensor', 10)

        self.pulse_start_ = 0
        self.pulse_end_ = 0
        self.pulse_durr_ = 0
        self.distance_ = 0

        # For keeping track of the mode
        self.wandering_mode = False

        # TESTING
        self.arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.arduino.reset_input_buffer()

        while True:
            self.setup()
            self.get_logger().debug("Distance Measurement in progress...")
            GPIO.output(TX_PIN, False)
            self.get_logger().debug("Waiting for sensor to settle")
            time.sleep(0.5)
            GPIO.output(TX_PIN, True)
            time.sleep(0.00001)
            GPIO.output(TX_PIN, False)

            while GPIO.input(RX_PIN) == 0:
                self.pulse_start_ = time.time()
            while GPIO.input(RX_PIN) == 1:
                self.pulse_end_ = time.time()
            
            self.pulse_durr_ = self.pulse_end_ - self.pulse_start_
            self.distance_ = round(self.pulse_durr_ * 17150, 2)
            self.get_logger().debug(str(self.distance_) + " cm")
            self.publish_sensor(self.distance_)  
            GPIO.cleanup()          


    def publish_sensor(self, distance):

        msg = Sensor()
        msg.pin = RX_PIN
        if distance <= DETECT_DIST:
            # Obstacle detected
            msg.avoid = True
            self.arduino.write(b"ms\n") # Stop motors if obstacle is encountered

            # Tell Arduino how to avoid obstacle
            time.sleep(1)
            self.arduino.write(b"rb\n")
            time.sleep(1)
            turnDirection = "r" + str(random.randint(0,1))
            self.arduino.write(turnDirection.encode('utf-8'))
            time.sleep(1)
            self.arduino.write(b"rf\n")

            self.get_logger().info(str(self.distance_) + " cm")
        else:
            if distance <= (DETECT_DIST * 2):
                self.arduino.write(b"rn\n")
            else:
                self.arduino.write(b"ru\n")
            self.arduino.write(b"rf\n")
            msg.avoid = False
        self.publisher_.publish(msg)


    def setWandering(self, wanderModeIn):
        self.wandering_mode = wanderModeIn


    def setup(self):
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(RX_PIN, GPIO.IN)
        GPIO.setup(TX_PIN, GPIO.OUT)

def main(args=None):
    rclpy.init(args=args)
    ultrasonic_publisher = UltrasonicPublisher()
               
    rclpy.spin(ultrasonic_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ultrasonic_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
