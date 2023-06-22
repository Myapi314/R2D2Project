import rclpy
from rclpy.node import Node
from pocketsphinx import LiveSpeech, get_model_path

from std_msgs.msg import String
import os


class VoicePublisher(Node):
    last_string = "None"
    def __init__(self):
        super().__init__('voice_publisher')
        self.publisher_ = self.create_publisher(String, 'voice', 10)
 #       timer_period = 0.5  # seconds
 #       self.timer = self.create_timer(timer_period, self.timer_callback)
 #       self.i = 0

 #   def timer_callback(self):
    def publish_string(self, phrase):
        msg = String()
        msg.data = '%s' % phrase
        self.get_logger().info('Voice Received: "%s"' % msg.data)
        if msg.data != VoicePublisher.last_string and len(msg.data) != 0:
            VoicePublisher.last_string = msg.data
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)


        
def main(args=None):
    rclpy.init(args=args)
    model_path = get_model_path()
    # this configures pocketsphinx to return voice commands
    voice_publisher = VoicePublisher()
    for phrase in LiveSpeech(
            verbose=False,
            sampling_rate=16000,
            buffer_size=2048,
            no_search=False,
            full_utt=False,
        #    hmm=os.path.join(model_path, 'en-us'),
            lm='/home/redleader/ros2_ws/src/my_id_robot/my_id_robot/2301.lm',
            dic='/home/redleader/ros2_ws/src/my_id_robot/my_id_robot/2301.dic'):
        voice_publisher.publish_string(phrase)
 

               
    rclpy.spin(voice_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    voice_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
