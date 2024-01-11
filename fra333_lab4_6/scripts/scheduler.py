#!/usr/bin/python3
import math
from re import S
import numpy as np
from numpy import ones, zeros
import rclpy
import time
import sys, os, yaml
from rclpy.node import Node
from std_srvs.srv import Empty as Empty_srv
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from std_msgs.msg import Bool

class Scheduler(Node):

    def __init__(self):
        super().__init__('scheduler_node')
        send_int_point = Float32MultiArray()

        self.T = 1.0
        self.end_up  = 0.0
        self.count = 0

        self.declare_parameters(
            namespace='',
            parameters=[
                ('p_y', None),
                ('p_z', None),
                ('end_up', None)
            ])

        self.p_y = self.get_parameter('p_y').get_parameter_value().double_array_value
        self.p_z = self.get_parameter('p_z').get_parameter_value().double_array_value
        self.end_up = self.get_parameter('end_up').get_parameter_value().double_array_value
        
        self.via_point_publisher = self.create_publisher(Float32MultiArray,'/via_points',10)
        self.enable_publisher = self.create_publisher(Bool,'/enable',10)
        self.subscription = self.create_subscription(String,'/hasReached',self.reached,10)

        time.sleep(6)

        send_int_point.data = [0.0, 0.185, 0.15, 0.05, 1.0, 1.0]
        self.via_point_publisher.publish(send_int_point)

    def reached(self,msg:String):
        send_via_point = Float32MultiArray()
        enable = Bool()

        if msg.data == 'Reach':
            print('reach')
            if self.count < len(self.p_y) - 1:

                enable.data = False
                self.enable_publisher.publish(enable)

                send_via_point.data = [self.p_y[self.count],self.p_z[self.count],self.p_y[self.count+1],self.p_z[self.count+1],self.T,self.end_up[self.count]]

                self.get_logger().info(f'send_via_point: {self.count}')

                enable.data = True
                self.via_point_publisher.publish(send_via_point)
                self.enable_publisher.publish(enable)
                
                self.count += 1

def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = Scheduler()
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