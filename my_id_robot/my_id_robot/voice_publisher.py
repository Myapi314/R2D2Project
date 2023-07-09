import sys
import rclpy
from rclpy.node import Node
import queue
import sounddevice as sd
from vosk import Model, KaldiRecognizer

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

q = queue.Queue()

def callback(indata, frames, time, status):
    """This is called (from a separate thread) for each audio block."""
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))  

model = Model(lang="en-us", model_path="/home/redleader/ros2_ws/src/my_id_robot/my_id_robot/vosk-model-small-en-us-0.15")

def main(args=None):
    rclpy.init(args=args)
    voice_publisher = VoicePublisher()
    # Was having some issues with blocksize=8000, 4000 seems to be working better
    with sd.RawInputStream(samplerate=44100, blocksize = 4000, device=None,
                       dtype="int16", channels=1, callback=callback):
        rec = KaldiRecognizer(model, 44100)
        while True:
            data = q.get()
            if rec.AcceptWaveform(data):
                command = rec.Result()
                command_split = command.split(":")
                final_command = command_split[1].strip("\" \n\"}")
  #               print(final_command)
                voice_publisher.publish_string(final_command)
 

               
    rclpy.spin(voice_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    voice_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
