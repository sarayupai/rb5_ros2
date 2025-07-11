#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from path_planner import PathPlanner
from rb5_ros2_control.srv import NavigatePath
import string

class Main(Node):
    def __init__(self):
        super().__init__('voice_activated_guide')

        # Map of words to goal poses 
        self.location_map = {
            'kitchen': "C"
        }

        # Define your graph nodes here  
        GRAPH_NODES = {
            "A": {"pos": (0.0, 0.0), "neighbors": ["B", "D"]},
            "B": {"pos": (0.0, 0.01), "neighbors": ["A", "C"]},
            "C": {"pos": (0.0, 0.02), "neighbors": ["B", "F"]},
            "D": {"pos": (0.03, 0.0), "neighbors": ["A", "G"]},
            #"E": {"pos": (0.03, 0.01), "neighbors": ["C"]},
            "F": {"pos": (0.03, 0.02), "neighbors": ["C", "I"]},
            "G": {"pos": (0.06, 0.0), "neighbors": ["D", "H"]},
            "H": {"pos": (0.06, 0.01), "neighbors": ["G", "I"]},
            "I": {"pos": (0.06, 0.02), "neighbors": ["F", "H"]},
        }

        self.planner = PathPlanner(GRAPH_NODES)

        # Subscribers
        self.create_subscription(String, '/transcribed_speech', self.speech_callback, 10)  
        self.create_subscription(PoseStamped, '/camera_pose', self.pose_callback, 10)

        # Publishers
        self.state_publisher = self.create_publisher(String, '/state', 10)
        self.speech_publisher = self.create_publisher(String, '/speaker_text', 10)     

        # Client 
        self.cli = self.create_client(NavigatePath, 'navigate_path')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = NavigatePath.Request()   

        # Initialize state 
        self.state = 'record' # 2 options: record, navigate 
        self.pose = (0.0, 0.0)
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

    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.pose = (x, y)

    def speech_callback(self, msg):
        # Process transcribed speech only if system is in record state 
        if self.state == 'record':
            words = msg.data.lower().strip().split()               # TODO: decide how to process input (how to handle multiple valid words, etc.)
            for word in words: 
                word = word.strip(string.punctuation)
                # if valid location
                if word in self.location_map:
                    
                    # Update state 
                    self.state = 'navigate'    
                    self.publish_state() 

                    # Set goal          
                    goal = self.location_map[word]
                    
                    # Plan path 
                    path = self.planner.get_path(goal, self.pose) 

                    # TODO: launch navigation 
                    self.get_logger().info(f"Going to {word} at {goal}") 
                    self.publish_speech(f"Heading to the {word}")

                    # Use service to spin up navigation  
                    response = self.send_waypoints(path)
                    self.get_logger().info("Reached destination")
                    
                    # Update state 
                    self.state = 'record'    
                    self.publish_state()   
        
            self.publish_speech(f"Unknown locations: {words}")
        else: 
            self.get_logger().info("Ignoring transcribed speech")

    def send_waypoints(self, waypoints):
        # Test waypoints 
        # waypoints = [0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.06, 0.0, 0.0] 

        self.req.waypoints = waypoints
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info(f"Service complete")

        if self.future.result() is not None:
            self.get_logger().info(f"Service responded: success = {self.future.result().success}")
        else:
            self.get_logger().error("Service call failed")
        return self.future.result()
   
def main(args=None):
    rclpy.init(args=args)
    node = Main()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()