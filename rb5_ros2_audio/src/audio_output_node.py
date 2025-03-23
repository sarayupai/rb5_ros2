#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3

class AudioOutputNode(Node):
    def __init__(self):
        super().__init__('audio_module')
        self.speaker = pyttsx3.init()
        self.subscription = self.create_subscription(String, 'speech_text', self.speak_text, 10)
        self.get_logger().info("Text-to-Speech Node Initialized")
        
    # callback function 
    def speak_text(self, msg):
        # Speak the text using pyttsx3
        self.get_logger().info(f"Received: {msg.data}")
        self.speaker.say(msg.data)
        self.speaker.runAndWait()

def main(args=None):
    print('audio_output')
    rclpy.init(args=args)
    node = AudioOutputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()