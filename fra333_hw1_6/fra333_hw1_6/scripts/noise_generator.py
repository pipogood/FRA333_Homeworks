#!/usr/bin/python3

# import all other neccesary libraries here
import rclpy
from rclpy.node import Node
import sys
from std_msgs.msg import Float64
import numpy as np
from lab1_interfaces.srv import SetNoise

class NoiseGenerator(Node):

    def __init__(self):
        # get the rate from argument or default
        super().__init__('noise_generator')
        if len(sys.argv)>2: 
            self.rate = float(sys.argv[3])
        else:
            self.rate = 5.0
        # add codes here
        self.timer = self.create_timer(1/self.rate,self.timer_callback)  #เรียกไปเรื่อยๆ ทุก 0.1 วิ
        #additional attributes
        self.mean = 0
        self.variance = 0
        self.launch_mean = sys.argv[1]
        self.launch_variance = sys.argv[2]
        self.noise_publisher = self.create_publisher(Float64,'/noise',10)
        self.set_noise_service = self.create_service(SetNoise,'/set_noise',self.set_noise_callback)
        self.get_logger().info(f'Starting {self.get_namespace()}/{self.get_name()} with the default parameter. mean: {self.launch_mean}, variance: {self.launch_variance}, rate: {self.rate}')
    
    def create_linear_noise(self):
        msg = Float64()
        l = np.random.normal(self.mean, np.sqrt(self.variance))
        print(l)
        msg.data = l
        return msg

    def set_noise_callback(self,request:SetNoise.Request,response:SetNoise.Response):
        self.mean = request.mean.data
        self.variance = request.variance.data
        return response
    
    def timer_callback(self):
        #print(self.variance)
        msg = self.create_linear_noise()
        self.noise_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = NoiseGenerator()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
