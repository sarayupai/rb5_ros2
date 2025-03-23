#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import sys
import termios
import tty
import os

class KeyboardToJoy(Node):
    def __init__(self):
        super().__init__('keyboard_to_joy')
        self.publisher_ = self.create_publisher(Joy, '/joy', 10)
        self.axes = [0.0] * 8  # Standard Joy message has at least 8 axes
        self.buttons = [0] * 12  # Standard Joy message has at least 12 buttons

    def get_key(self):
        """Reads a single key press without blocking"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = os.read(fd, 3)
            return key.decode('utf-8')  # return the key as string
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def process_input(self, key):
        self.axes = [0.0] * 8  # Reset axes
        if key == 'w':
            self.axes[1] = 1.0  # Forward
        elif key == 's':
            self.axes[1] = -1.0  # Reverse
        elif key == 'a':
            self.axes[0] = 1.0  # Left
        elif key == 'd':
            self.axes[0] = -1.0  # Right
        elif key == 'e':
            self.axes[3] = -1.0  # Clockwise turn
        elif key == 'q':
            self.axes[3] = 1.0  # Counter-clockwise turn
        elif key == ' ' or key == 'x':
            self.axes = [0.0] * 8  # Reset axes when space is pressed

        # No button presses mapped, but this can be customized
        self.publish_joy()

    def publish_joy(self):
        joy_msg = Joy()
        joy_msg.axes = self.axes
        joy_msg.buttons = self.buttons
        self.publisher_.publish(joy_msg)
        
        self.get_logger().info(f'Published Joy: Axes {self.axes}, Buttons {self.buttons}')

    def run(self):
        """Main loop to capture key presses and publish messages"""
        self.get_logger().info('Press keys: WASD for movement, Q/E for turning, Space to reset, X to quit')
        while rclpy.ok():
            key = self.get_key()
            self.process_input(key)
            if key == 'x': 
                self.get_logger().info("x pressed. Shutting down...")
                break 
            

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardToJoy()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()