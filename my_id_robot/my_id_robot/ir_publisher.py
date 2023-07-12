import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
from my_id_robot_interfaces.msg import Sensor

LFIR_PIN = 10
RFIR_PIN = 9
CFIR_PIN = 11
LH_PIN = 20
RH_PIN = 21

SENSOR_PINS = [LFIR_PIN, RFIR_PIN, CFIR_PIN, LH_PIN, RH_PIN]

class InfraredPublisher(Node):
    last_string = "None"
    def __init__(self):
        super().__init__('ir_publisher')
        # self.publisher_ = self.create_publisher(String, 'voice', 10)
        self.get_logger().info("Infrared node running!")
        self.publisher_ = self.create_publisher(Sensor, 'sensor', 10)

        GPIO.setmode(GPIO.BCM)
        self.setup_sensors()

        try:
            while True:
                for pin in SENSOR_PINS:
                    self.publish_sensor(pin)
        except KeyboardInterrupt:
            GPIO.cleanup()

    def publish_sensor(self, pin):
        msg = Sensor()
        msg.pin = pin
        msg.avoid = False

        if (pin == LFIR_PIN):
            if not GPIO.input(pin):
                self.get_logger().debug("Infrared Sensed Something on " + str(pin) )
                msg.avoid = True
        else:
            if not GPIO.input(pin):
                self.get_logger().debug("Infrared Sensed Something on " + str(pin))
                msg.avoid = True
        self.publisher_.publish(msg)

    def setup_sensors(self):
        GPIO.setup(LFIR_PIN, GPIO.IN)
        GPIO.setup(RFIR_PIN, GPIO.IN)
        GPIO.setup(CFIR_PIN, GPIO.IN)
        GPIO.setup(LH_PIN, GPIO.IN)
        GPIO.setup(RH_PIN, GPIO.IN)
        

def main(args=None):
    rclpy.init(args=args)
    ir_publisher = InfraredPublisher()
               
    rclpy.spin(ir_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ir_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
