#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped 
import numpy as np
from typing import List, Set, Dict, Tuple, Type
import csv
from tf_transformations import euler_from_quaternion
from rb5_ros2_control.srv import NavigatePath

class PIDcontroller(Node):
    def __init__(self, Kp, Ki, Kd):
        super().__init__('PID_Controller_NodePub')
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = None
        self.I = np.array([0.0,0.0,0.0])
        self.lastError = np.array([0.0,0.0,0.0])
        self.timestep = 0.1
        self.maximumValue = 0.02  # changed for meters
        self.publisher_ = self.create_publisher(Twist, '/twist', 10)

    def setTarget(self, target):
        """
        Set the target pose.
        """
        self.I = np.array([0.0, 0.0, 0.0])
        self.lastError = np.array([0.0, 0.0, 0.0])
        self.target = np.array(target)

    def getError(self, currentState, targetState):
        """
        Return the difference between two states.
        """
        result = targetState - currentState
        result[2] = 0.0
        # result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
        return result

    def setMaximumUpdate(self, mv):
        """
        Set maximum velocity for stability.
        """
        self.maximumValue = mv

    def update(self, currentState):
        """
        Calculate the update value on the state based on the error between current state and target state with PID.
        """
        e = self.getError(currentState, self.target)
        P = self.Kp * e
        self.I += self.Ki * e * self.timestep
        D = self.Kd * (e - self.lastError)
        result = P + self.I + D
        self.lastError = e

        # Scale down the twist if its norm is more than the maximum value
        resultNorm = np.linalg.norm(result)
        if resultNorm > self.maximumValue:
            result = (result / resultNorm) * self.maximumValue
            self.I = 0.0

        return result

def feet_to_meters(feet):
    if type(feet) is float:
        return feet * 0.3048
    else:
        nums = [num * 0.3048 for num in feet]
        return np.array(nums)

def meters_to_feet(meters):
        if type(meters) is float:
            return meters * 3.28084
        else:
            nums = [num * 3.28084 for num in meters]
            return np.array(nums)

class RobotStateEstimator(Node):
    def __init__(self):
        super().__init__('robot_state_estimator')
        self.subscription = self.create_subscription(PoseStamped, '/camera_pose', self.pose_callback, 10)
        self.subscription
        self.current_state = np.array([0.0, 0.0, 0.0]) # TODO: update start pose 
        self.pose_updated = True

    def pose_callback(self, msg):
        self.pose_updated = False
        #self.get_logger().info(f"Received SLAM Pose")
        self.current_state = self.pose_to_2d(msg)
        self.pose_updated = True

    def pose_to_2d(self, pose_msg: PoseStamped):
        x = pose_msg.pose.position.x
        y = pose_msg.pose.position.y

        q = pose_msg.pose.orientation
        quat = [q.x, q.y, q.z, q.w]

        _, _, yaw = euler_from_quaternion(quat)
        theta = yaw
        return np.array([x, y, theta])

def genTwistMsg(desired_twist):
    """
    Convert the twist to twist msg.
    """
    twist_msg = Twist()
    twist_msg.linear.x = desired_twist[0]
    twist_msg.linear.y = desired_twist[1]
    twist_msg.linear.z = 0.0
    twist_msg.angular.x = 0.0
    twist_msg.angular.y = 0.0
    twist_msg.angular.z = desired_twist[2]
    return twist_msg

def coord(twist, current_state):
    J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                  [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0],
                  [0.0,0.0,1.0]])
    desired_twist = np.dot(J, twist)
    # desired_twist[:2] = desired_twist[:2] #* 0.3048  # convert from ft/s to m/s
    return desired_twist

def waypoints_to_meters(waypoints: np.array):
    for wp in waypoints:
        wp[:2] = wp[:2] * 0.3048

def state_to_feet_list(state: np.array) -> List[float]:
    new_state = state.copy()
    new_state[:2] = new_state[:2] * 3.28084
    return list(state)

class NavigationService(Node):
    def __init__(self):
        super().__init__('navigation_service')
        self.srv = self.create_service(NavigatePath, 'navigate_path', self.navigate_callback)
        self.robot_state_estimator = RobotStateEstimator()
        self.pid = PIDcontroller(0.1, 0.005, 0.005)

    def navigate_callback(self, request, response):
        #self.get_logger().info('Incoming request:\n', request.waypoints)
        waypoints = np.array([tuple(request.waypoints[i:i+3]) for i in range (0, len(request.waypoints), 3)])
        print(waypoints)

        self.get_logger().info(f"Received {len(waypoints)} waypoints:")
        for wp in waypoints:
            self.get_logger().info(f"x: {wp[0]}, y: {wp[1]}, theta: {wp[2]}")
   
        # init pid controller
        current_state = self.robot_state_estimator.current_state

        trajectory = [['x', 'y', 'theta']]  # save trajectory of robot

        for wp in waypoints:
            print("move to way point", wp)
            # set wp as the target point
            self.pid.setTarget(wp)

            # calculate the current twist
            update_value = self.pid.update(current_state)
            # publish the twist
            self.pid.publisher_.publish(genTwistMsg(coord(update_value, current_state)))
            #print(coord(update_value, current_state))
            time.sleep(0.05)
            print('here')

            # update the current state
            current_state += update_value
            rclpy.spin_once(self.robot_state_estimator)
            found_state, estimated_state = self.robot_state_estimator.pose_updated, self.robot_state_estimator.current_state
            if found_state: # if the tag is detected, we can use it to update current state.
                current_state = estimated_state

            # state_feet = state_to_feet_list(current_state)
            trajectory.append(current_state)
            print("waypoint:", wp)
            print('current_state:', current_state)
            print()

            # check the error between current state and current way point
            while(np.linalg.norm(self.pid.getError(current_state, wp)) > 0.05): # scaled up from 0.05
                # calculate the current twist
                update_value = self.pid.update(current_state)
                # publish the twist
                self.pid.publisher_.publish(genTwistMsg(coord(update_value, current_state)))
                #print(coord(update_value, current_state))
                time.sleep(0.05)

                # update the current state
                current_state += update_value
                rclpy.spin_once(self.robot_state_estimator)
                found_state, estimated_state = self.robot_state_estimator.pose_updated, self.robot_state_estimator.current_state
                if found_state: # if the tag is detected, we can use it to update current state.
                    current_state = estimated_state

                # state_feet = state_to_feet_list(current_state)
                trajectory.append(current_state)
                print("waypoint:", wp)
                print('current_state:', current_state)
                print('here')
                print()

        # stop the car and exit
        self.pid.publisher_.publish(genTwistMsg(np.array([0.0,0.0,0.0])))

        # Save trajectory
        with open("trajectory.csv", "w") as f:
            wr = csv.writer(f)
            wr.writerows(trajectory)

        self.get_logger().info(f"Arrived at goal")
        self.robot_state_estimator.destroy_node()
        response.success = True
        return response 

def main(args=None):
    rclpy.init(args=args)
    node = NavigationService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

    