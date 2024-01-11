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
from std_msgs.msg import String

class Proximity(Node):

    def __init__(self):
        super().__init__('proximity_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('thereshold', None)
            ])

        self.thereshold = self.get_parameter('thereshold').get_parameter_value().double_value
        self.p_y = 0.0
        self.p_z = 0.0
        self.p_x = 0.0

        self.pf_y = 1.0
        self.pf_z = 1.0
        self.pf_x = 0.12

        self.update_point = 0

        self.end_up = 0.0

        self.subscription1 = self.create_subscription(Float32MultiArray,'/via_points',self.via_point,10) #from scheduler
        self.subscription2 = self.create_subscription(Float32MultiArray,'/task_states',self.task_states,10) #from kinematics server
        self.reached_publihser = self.create_publisher(String,'/hasReached', 10)


    def via_point(self,msg:Float32MultiArray):
        self.pf_y = msg.data[2]
        self.pf_z = msg.data[3]
        self.end_up = msg.data[5]

        if self.end_up > 0.5:
            self.pf_x = 0.1
        if self.end_up < 0.5:
            self.pf_x = 0.12


        self.update_point = 1


    def task_states(self,msg:Float32MultiArray):

        if self.update_point == 1:
            self.p_x = msg.data[0]
            self.p_y = msg.data[1]
            self.p_z = msg.data[2]

            print("from_pf",self.pf_x,self.pf_y,self.pf_z)
            print("from_fk",self.p_x,self.p_y,self.p_z)

            if abs(self.pf_x - self.p_x) <= self.thereshold and abs(self.pf_y - self.p_y) <= self.thereshold and abs(self.pf_z - self.p_z) <= self.thereshold:
                send_reach = String()
                send_reach.data = "Reach"

                self.reached_publihser.publish(send_reach)
                self.update_point = 0


def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = Proximity()
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