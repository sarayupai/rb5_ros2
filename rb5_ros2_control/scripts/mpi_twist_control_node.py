#!/usr/bin/env python3
""" MegaPi Controller ROS2 Wrapper"""
#import rospy
import rclpy # replaces rospy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from mpi_control import MegaPiController
import numpy as np


class MegaPiControllerNode(Node):
    def __init__(self, verbose=True, debug=False):
        super().__init__('megapi_controller_node')
        self.mpi_ctrl = MegaPiController(verbose=verbose)
        self.r = 0.025 # radius of the wheel
        self.lx = 0.055 # half of the distance between front wheel and back wheel
        self.ly = 0.07 # half of the distance between le2ft wheel and right wheel
        self.calibration = 100.0  # changed from 125 (75 overshoots)
        self.turn_calibration = 1.7  # changed from 1.5
        self.max_value = 120.0
        self.subscription = self.create_subscription(Twist, '/twist', self.twist_callback, 10)
        self.subscription


    def is_turning(self, cmd):
        # r1, r2, r3, r4 = cmd
        # turning_left = (r1 < 0) and (r3 < 0) and (r2 > 0) and (r4 > 0)
        # turning_right = (r1 > 0) and (r3 > 0) and (r2 < 0) and (r4 < 0)
        # turning = turning_left or turning_right
        pos = 0
        neg = 0
        for r in cmd:
            if r < 0:
                neg += 1
            else:
                pos += 1
        turning = (pos == 2) and (neg == 2)
        return turning


    def scale_down(self, cmd):
        print('scaling down')
        command = np.array(cmd)
        cmd_norm = np.linalg.norm(command)
        if cmd_norm > self.max_value:
            command = (command / cmd_norm) * self.max_value
        return command


    def twist_callback(self, twist_cmd):
        desired_twist = self.calibration * np.array([[twist_cmd.linear.x], [twist_cmd.linear.y], [twist_cmd.angular.z]])
        print(f'Twist: {desired_twist}')
        # calculate the jacobian matrix
        jacobian_matrix = np.array([[1, -1, -(self.lx + self.ly)],
                                    [1, 1, (self.lx + self.ly)],
                                    [1, 1, -(self.lx + self.ly)],
                                    [1, -1, (self.lx + self.ly)]]) / self.r
        # calculate the desired wheel velocity
        result = np.dot(jacobian_matrix, desired_twist)
        print(f'result wheel velocities: {result}')
        cmd = np.array([result[i][0] for i in range(4)])
        cmd = self.scale_down(cmd)  # in case any speed is over 60

        # send command to each wheel
        if self.is_turning(result):
            print(f"Command before {cmd}")
            cmd = [int(self.turn_calibration * val) for val in cmd]
            print(f"Command after {cmd}")
            self.mpi_ctrl.setFourMotors(cmd[0], cmd[1], cmd[2], cmd[3])
        else:
            print(f"Command before {cmd}")
            cmd = [int(val) for val in cmd]
            print(f"Command after {cmd}")
            self.mpi_ctrl.setFourMotors(cmd[0], cmd[1], cmd[2], cmd[3])



if __name__ == "__main__":
    rclpy.init()
    mpi_ctrl_node = MegaPiControllerNode()
    rclpy.spin(mpi_ctrl_node) # Spin for until shutdown
    # Destroy node and shutdown when done. (Optional, as node would be cleaned up by garbage collection)
    mpi_ctrl_node.destroy_node()
    rclpy.shutdown()
