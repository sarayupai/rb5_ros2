#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Joy

import time 

from megapi import *

# TODO: confirm these 
MFR = 2     # port for motor front right
MBL = 3     # port for motor back left
MBR = 10    # port for motor back right
MFL = 11    # port for motor front left

class MegaPiController(Node):
    def __init__(self, verbose=True):
        super().__init__('rb5_control')
        self.bot = MegaPi()
        self.bot.start('/dev/ttyUSB0')
        time.sleep(1)

        self.verbose = verbose
        if verbose:
            self.printConfiguration()

        self.joy_sub = self.create_subscription(
                        Joy,
                        '/joy',
                        self.move_joy, 1)

        self.joy_sub

        self.mfr = MFR  # port for motor front right
        self.mbl = MBL  # port for motor back left
        self.mbr = MBR  # port for motor back right
        self.mfl = MFL  # port for motor front left

    def printConfiguration(self):
        print('MegaPiController:')
        print("Communication Port:" + repr(self.port))
        print("Motor ports:" +
              " MFR: " + repr(MFR) +
              " MBL: " + repr(MBL) +
              " MBR: " + repr(MBR) +
              " MFL: " + repr(MFL))

    def move(self, direction, speed):
        # port1: front right (wheel 1)
        # port2: rear left (wheel 0)
        # port9: rear right (wheel 3)
        # port10: front left (wheel 2)
        W0 = 3   # 10
        W1 = 2   # 11 
        W2 = 11 # 2 
        W3 = 10  # 3 

        # rear right - port 10 
        # rear left - port 3 
        # front left - port 11 
        # front right - port 2

        if direction == "left":
            self.get_logger().info("Moving left")
            # fl wheel
            self.bot.motorRun(W2, speed)

            # fr wheel
            self.bot.motorRun(W1, speed)

            # rl wheel
            self.bot.motorRun(W0, -speed)

            # rr wheel
            self.bot.motorRun(W3, -speed)
            return
        elif direction == "right":
            self.get_logger().info("Moving right")
            # fl wheel
            self.bot.motorRun(W2, -speed)

            # fr wheel
            self.bot.motorRun(W1, -speed)

            # rl wheel
            self.bot.motorRun(W0, speed)

            # rr wheel
            self.bot.motorRun(W3, speed)
            return
        elif direction == "forward":
            self.get_logger().info("Moving forward")
            # fl wheel
            self.bot.motorRun(W2, -speed)

            # fr wheel
            self.bot.motorRun(W1, speed)

            # rl wheel
            self.bot.motorRun(W0, -speed)

            # rr wheel
            self.bot.motorRun(W3, speed)


            return
        elif direction == "reverse":
            self.get_logger().info("Moving in reverse")
            # fl wheel
            self.bot.motorRun(W2, speed)

            # fr wheel
            self.bot.motorRun(W1, -speed)

            # rl wheel
            self.bot.motorRun(W0, speed)

            # rr wheel
            self.bot.motorRun(W3, -speed)

            return
        elif direction == "ccwise":
            # Counter clockwise
            self.get_logger().info("Moving counter clockwise")
            # fl wheel
            self.bot.motorRun(W2, speed)

            # fr wheel
            self.bot.motorRun(W1, speed)

            # rl wheel
            self.bot.motorRun(W0, speed)

            # rr wheel
            self.bot.motorRun(W3, speed)
        elif direction == "cwise":
            # Clockwise
            self.get_logger().info("Moving clockwise")
            # fl wheel
            self.bot.motorRun(W2, -speed)

            # fr wheel
            self.bot.motorRun(W1, -speed)

            # rl wheel
            self.bot.motorRun(W0, -speed)

            # rr wheel
            self.bot.motorRun(W3, -speed)

        else:
            self.get_logger().info("Stopping")
            # fl wheel
            self.bot.motorRun(W2, 0)

            # fr wheel
            self.bot.motorRun(W1, 0)

            # rl wheel
            self.bot.motorRun(W0, 0)

            # rr wheel
            self.bot.motorRun(W3, 0)
            return

    def move_joy(self, joy_cmd):

        #if joy_cmd.buttons[5]:
        if joy_cmd.axes[0] > 0.0:
            # left
            self.move("left", 50)
        elif joy_cmd.axes[0] < 0.0:
            # right
            self.move("right", 50)
        elif joy_cmd.axes[1] > 0.0:
            # forward
            self.move("forward", 30)
        elif joy_cmd.axes[1] < 0.0:
            # reverse
            self.move("reverse", 30)
        elif joy_cmd.axes[3] < 0.0:
            # turn clock-wise 
            self.move("cwise", 35)
        elif joy_cmd.axes[3] > 0.0:
            # turn counter clock-wise 
            self.move("ccwise", 35)
        #elif abs(joy_cmd.axes[0]) <= 0.1 and abs(joy_cmd.axes[1]) <= 0.0:
        #    self.move("stop", 0)
        else:
            self.move("stop", 0)


    def move_dir(self, str_cmd):
        
        #if direction == "left":
        #elif direction == "right":
        #elif direction == 
        return    
    
    def setFourMotors(self, vfl=0, vfr=0, vbl=0, vbr=0):
        if self.verbose:
            print("Set Motors: vfl: " + repr(int(round(vfl,0))) +
                             " vfr: " + repr(int(round(vfr,0))) +
                             " vbl: " + repr(int(round(vbl,0))) +
                             " vbr: " + repr(int(round(vbr,0))))
        self.bot.motorRun(self.mfl,-int(vfl))
        self.bot.motorRun(self.mfr,int(vfr))
        self.bot.motorRun(self.mbl,-int(vbl))
        self.bot.motorRun(self.mbr,int(vbr))


    def carStop(self):
        if self.verbose:
            print("CAR STOP:")
        self.setFourMotors()


    def carStraight(self, speed):
        if self.verbose:
            print("CAR STRAIGHT:")
        self.setFourMotors(-speed, speed, -speed, speed)


    def carRotate(self, speed):
        if self.verbose:
            print("CAR ROTATE:")
        self.setFourMotors(speed, speed, speed, speed)


    def carSlide(self, speed):
        if self.verbose:
            print("CAR SLIDE:")
        self.setFourMotors(speed, speed, -speed, -speed)


    def carMixed(self, v_straight, v_rotate, v_slide):
        if self.verbose:
            print("CAR MIXED")
        self.setFourMotors(
            v_rotate-v_straight+v_slide,
            v_rotate+v_straight+v_slide,
            v_rotate-v_straight-v_slide,
            v_rotate+v_straight-v_slide
        )

    def close(self):
        self.bot.close()
        self.bot.exit()

def main(args=None):
    rclpy.init(args=args)
    ctrl_node = MegaPiController(verbose=True)
    #rclpy.Subscriber('/rb5/ctrl_cmd_str', String, ctrl.move_dir) 
    #rclpy.Subscriber('/joy', Joy, ctrl.move_joy, queue_size=1) 
    '''
    ctrl_node.move("forward", 30)
    time.sleep(3)
    ctrl_node.move("right", 30)
    time.sleep(3)
    ctrl_node.move("left", 30)
    time.sleep(3)
    ctrl_node.move("cwise", 30)
    time.sleep(3)
    ctrl_node.move("ccwise", 30)
    time.sleep(3)
    ctrl_node.move("stop", 0)



    '''
    rclpy.spin(ctrl_node)
    ctrl_node.destroy_node()
    rclpy.shutdown()

    # mpi_ctrl.carStraight(30)
    #mpi_ctrl.setFourMotors(0,0,0,0)
    #time.sleep(1)
    # mpi_ctrl.carSlide(30)
    # time.sleep(1)
    # mpi_ctrl.carRotate(30)
    # time.sleep(1)
    # mpi_ctrl.carStop()
    #mpi_ctrl.close()

if __name__ == "__main__":
    main()