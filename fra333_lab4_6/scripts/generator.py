#!/usr/bin/python3
import math
from re import S
import numpy as np
from numpy import ones, zeros
import rclpy
import sys, os, yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker

class Generator(Node):

    def __init__(self):
        super().__init__('generator_node')
        self.pi_y = 0.0
        self.pi_z = 0.0
        self.pf_y = 0.0
        self.pf_z = 0.0
        self.count = 0.0
        self.T = 0.0
        self.end_up  = 0.0

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription1 = self.create_subscription(Float32MultiArray,'/via_points',self.via_point,10)
        self.ref_task_states_publihser = self.create_publisher(Float32MultiArray,'/reference/task_states', 10) # to kinematics server

    def timer_callback(self):
        self.count += 0.1
        if self.count <= self.T:
            self.traject_gen()

    def via_point(self,msg:Float32MultiArray):
        self.count = 0.0
        self.pi_y = msg.data[0]
        self.pi_z = msg.data[1]
        self.pf_y = msg.data[2]
        self.pf_z = msg.data[3]
        self.T = msg.data[4]
        self.end_up = msg.data[5]
        print(self.pi_y,self.pi_z,self.pf_y,self.pf_z)

    def traject_gen(self):
        self.pr_y = ((1 - self.count) * self.pi_y) + (self.count * self.pf_y)
        self.pr_y_dot = 0.05 * (self.pf_y-self.pi_y)

        self.pr_z = ((1 - self.count) * self.pi_z) + (self.count * self.pf_z)
        self.pr_z_dot = 0.05 * (self.pf_z-self.pi_z)

        if self.count == 0 or self.count == self.T:
            self.pr_y_dot = 0.0
            self.pr_z_dot = 0.0

        send_task_states = Float32MultiArray()
        send_task_states.data = [self.pr_y,self.pr_z,self.pr_y_dot,self.pr_z_dot,self.end_up]

        self.ref_task_states_publihser.publish(send_task_states)

def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = Generator()
    try:
        while rclpy.ok():
            rclpy.spin_once(joint_trajectory_object)
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        joint_trajectory_object.destroy_node()
        rclpy.shutdown() 


if __name__ == '__main__':
    main()