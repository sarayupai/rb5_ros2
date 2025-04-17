#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class Main(Node):
    def __init__(self):
        super().__init__('voice_activated_guide')

        # Map of words to goal poses (TODO: update based on SLAM)
        self.location_map = {
            'kitchen': (1.0, 2.0, 0.0),
            'lab': (3.5, -1.0, 1.57)
        }

        # Subscribers
        self.create_subscription(String, '/transcribed_speech', self.speech_callback, 10)  

        # Publishers
        self.state_publisher(String, '/state', 10)
        self.speech_publisher = self.create_publisher(String, '/speaker_text', 10)        

        # Initialize state 
        self.publish_state('record') # 2 options: record, navigate 

    def publish_state(self, state):
        msg = String()
        msg.data = state
        self.get_logger().info(f"[MAIN STATE]: {state}")
        self.state_publisher.publish(msg)

    def publish_speech(self, message):
        msg = String()
        msg.data = message
        self.get_logger().info(f"[SPEECH]: {message}")
        self.speech_publisher.publish(msg)

    def speech_callback(self, msg):
        words = msg.data.lower().strip().split()               # TODO: decide how to process input 
        for word in words: 
            if word in self.location_map:                      # TODO: how to handle multiple valid words 
                goal = self.location_map[word]
                self.publish_speech(f"Heading to the {word}")
                self.publish_state('navigate')                 # Update state 
                return 
        
        self.publish_speech(f"Unknown locations: {words}")

    

def main(args=None):
    rclpy.init(args=args)
    node = Main()
    rclpy.spin(node)
    rclpy.shutdown()


