import rclpy
from subprocess import call
from rclpy.node import Node
from time import sleep
import cv2
import numpy as np

from std_msgs.msg import String

video = cv2.VideoCapture(0)

class OpencvSubscriber(Node):

    def __init__(self):
        super().__init__('opencv_subscriber')
        self.subscription = self.create_subscription(
            String,
            'opencv',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(String, 'servo', 10)

        
    def listener_callback(self, msg):
        self.get_logger().info('Opencv from Main "%s"' % msg.data)
        if (msg.data == "find" or msg.data == "get"):
            ret, frame = video.read()
            #cv2.imshow("Web Cam", frame)
            #cv2.waitKey(2000) 
#            video.release()
            #font = cv2.FONT_HERSHEY_COMPLEX 
            gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#            cv2.imshow("Gray", gray_image)
#            cv2.waitKey(2000) 
            image_noise = cv2.medianBlur(gray_image, 3)
            _, threshold = cv2.threshold(image_noise, 0, 255, cv2.THRESH_OTSU) 
# Detecting contours in image.
#            cv2.imshow("Threshold", threshold)
#            cv2.waitKey(2000) 
            contours, _= cv2.findContours(threshold, cv2.RETR_TREE, 
                                          cv2.CHAIN_APPROX_SIMPLE) 

            if len(contours) != 0:
                for cnt in contours:
                    x,y,w,h = cv2.boundingRect(cnt)
                    area = cv2.contourArea(cnt)
                    if area > 2000 and area < 50000:
                        cv2.rectangle(frame, (x,y), (x+w,y+h), (0, 255, 0), 2)
# Showing the final image. 
                        #cv2.imshow('Contours', frame) 
                        cv2.waitKey(2000)
                        position = str(x) + "," + str(y)
                        position_msg = String()
                        position_msg.data = position
                        self.publisher_.publish(position_msg)
                        self.get_logger().info('OpenCV to Servo: "%s"' % position_msg.data)
            cv2.destroyAllWindows()

        
def main(args=None):
    rclpy.init(args=args)

    opencv_subscriber = OpencvSubscriber() 

    rclpy.spin(opencv_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    opencv_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
