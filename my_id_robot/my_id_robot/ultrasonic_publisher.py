import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
from my_id_robot_interfaces.msg import Sensor

TX_PIN = 14
RX_PIN = 15

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

        while True:
            self.setup()
            self.get_logger().info("Distance Measurement in progress...")
            GPIO.output(TX_PIN, False)
            self.get_logger().info("Waiting for sensor to settle")
            time.sleep(2)
            GPIO.output(TX_PIN, True)
            time.sleep(0.00001)
            GPIO.output(TX_PIN, False)

            while GPIO.input(RX_PIN) == 0:
                self.pulse_start_ = time.time()
            while GPIO.input(RX_PIN) == 1:
                self.pulse_end_ = time.time()
            
            self.pulse_durr_ = self.pulse_end_ - self.pulse_start_
            self.distance_ = round(self.pulse_durr_ * 17150, 2)
            self.publish_sensor(self.distance_)  
            self.get_logger().info(str(self.distance_) + " cm")
            GPIO.cleanup()          


    def publish_sensor(self, distance):
        # str(distance) + " cm"
        msg = Sensor()
        msg.pin = RX_PIN
        if distance <= 10:
            # Obstacle detected
            msg.avoid = True
        else:
            msg.avoid = False
        self.publisher_.publish(msg)


    def setup(self):
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(RX_PIN, GPIO.IN)
        GPIO.setup(TX_PIN, GPIO.OUT)

def main(args=None):
    rclpy.init(args=args)
    ir_publisher = UltrasonicPublisher()
               
    rclpy.spin(ir_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ir_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
