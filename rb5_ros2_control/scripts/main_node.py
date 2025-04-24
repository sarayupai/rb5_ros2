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
        self.state_publisher = self.create_publisher(String, '/state', 10)
        self.speech_publisher = self.create_publisher(String, '/speaker_text', 10)        

        # Initialize state 
        self.state = 'record' # 2 options: record, navigate 
        self.publish_state() 

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.get_logger().info(f"[MAIN STATE]: {self.state}")
        self.state_publisher.publish(msg)

    def publish_speech(self, message):
        msg = String()
        msg.data = message
        self.get_logger().info(f"[SPEECH]: {message}")
        self.speech_publisher.publish(msg)

    def speech_callback(self, msg):
        # Process transcribed speech only if system is in record state 
        if self.state == 'record':
            words = msg.data.lower().strip().split()               # TODO: decide how to process input (how to handle multiple valid words, etc.)
            for word in words: 
                # if valid location
                if word in self.location_map:
                    
                    # Update state 
                    self.state = 'navigate'    
                    self.publish_state() 

                    # Set goal          
                    goal = self.location_map[word]

                    self.get_logger().info(f"Going to {word} at {goal}") 
                    self.publish_speech(f"Heading to the {word}")

                    # TODO: launch navigation 
                    return 
        
            self.publish_speech(f"Unknown locations: {words}")
        else: 
            self.get_logger().info("Ignoring transcribed speech")

    

def main(args=None):
    rclpy.init(args=args)
    node = Main()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


