#!/usr/bin/python3
import math
from re import S
import numpy as np
from numpy import ones, zeros
import rclpy
import sys, os, yaml
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

class Tracker(Node):

    def __init__(self):
        super().__init__('tracker_node')
        #yaml

        self.declare_parameters(
            namespace='',
            parameters=[
                ('kp', None),
                ('ki', None)
            ])

        self.Kp = self.get_parameter('kp').get_parameter_value().double_value
        self.Ki = self.get_parameter('ki').get_parameter_value().double_value


        self.joint1 = 0.0
        self.joint2 = 0.0
        self.joint3 = 0.0

        self.joint1_dot = 0.0
        self.joint2_dot = 0.0
        self.joint3_dot = 0.0

        self.send_joint1 = 0.0
        self.send_joint2 = 0.0
        self.send_joint3 = 0.0

        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0

        self.Ki_1 = 0.0
        self.Ki_2 = 0.0
        self.Ki_3 = 0.0

        self.enable_send = 1

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.subscription1 = self.create_subscription(JointState,'/joint_states',self.get_joint_states,10) #from rviz
        self.subscription2 = self.create_subscription(Float32MultiArray,'/reference/joint_states',self.ref_joint_states,10) #from generator
        self.subscription3 = self.create_subscription(Bool,'/enable',self.enable,10)
        self.velo_publihser = self.create_publisher(Float64MultiArray,'/joint_velocity_controller/commands', 10)


    def enable(self,msg:Bool):
        if msg.data:
            self.enable_send = 1
        else:
            self.enable_send = 0


    def timer_callback(self):
        send_velo = Float64MultiArray()
        send_velo.data = [self.send_joint1,self.send_joint2,self.send_joint3]
        if self.enable_send == 1:
            self.velo_publihser.publish(send_velo)
        else:
            send_velo.data = [0.0,0.0,0.0]
            self.velo_publihser.publish(send_velo)

    def ref_joint_states(self,msg:Float32MultiArray):
        self.joint1 = msg.data[0]
        self.joint2 = msg.data[1]
        self.joint3 = msg.data[2]
        self.joint1_dot = msg.data[3]
        self.joint2_dot = msg.data[4]
        self.joint3_dot = msg.data[5]

        if self.enable_send == 1:
            self.tracker_control()
        else:
            self.Ki_1 = 0.0
            self.Ki_2 = 0.0
            self.Ki_3 = 0.0

    def get_joint_states(self,msg:JointState):
        self.q1 = msg.position[0]
        self.q2 = msg.position[1]
        self.q3 = msg.position[2]

    def tracker_control(self):
        self.Ki_1 = self.Ki_1 + (self.joint1-self.q1) * self.timer_period
        self.send_joint1 = ((self.joint1_dot + self.Kp) * (self.joint1-self.q1)) + (self.Ki * self.Ki_1)

        self.Ki_2 = self.Ki_2  + (self.joint2-self.q2) * self.timer_period
        self.send_joint2 = ((self.joint2_dot + self.Kp) * (self.joint2-self.q2)) + (self.Ki * self.Ki_2)

        self.Ki_3 = self.Ki_3 + (self.joint3-self.q3) * self.timer_period
        self.send_joint3 = ((self.joint3_dot + self.Kp) * (self.joint3-self.q3)) + (self.Ki * self.Ki_3)

 
def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = Tracker()
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