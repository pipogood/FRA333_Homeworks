#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import sys
# linear = 0.0
# angular = 0.0

class VelocityMux(Node):
    def __init__(self):
        super().__init__('velocity_mux')
        # get the rate from argument or default
        if sys.argv[1]:
             self.rate = float(sys.argv[1])
        else:
            self.rate = 5.0

        
        # add codes here
        self.cmd_publisher = self.create_publisher(Twist,'/turtle1/cmd_vel',10)
        self.linear_subscription = self.create_subscription(Float64,'/linear/noise',self.linear_vel_sub_callback,10)
        self.angular_subscription = self.create_subscription(Float64,'/angular/noise',self.angular_vel_sub_callback,10)
        #self.subscription
        self.timer = self.create_timer(1/self.rate,self.timer_callback)  #เรียกไปเรื่อยๆ ทุก 0.1 วิ
        # additional attributes
        self.cmd_vel = Twist() 
        self.get_logger().info(f'Starting {self.get_name()} Get_rate {self.rate}')

    def linear_vel_sub_callback(self,msg:Float64):
        self.linear_noise = msg.data
        print("linear",msg.data)

    def angular_vel_sub_callback(self,msg:Float64):
        self.angular_noise = msg.data
        print("angular",msg.data)
    
    def timer_callback(self):
        # remove pass and add codes here
        msg = Twist()
        msg.linear.x = 1.0 + self.linear_noise
        msg.angular.z = 1.0 + self.angular_noise
        self.cmd_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = VelocityMux()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
    # remove pass and add codes here

if __name__=='__main__':
    main()
