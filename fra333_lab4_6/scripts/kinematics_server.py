#!/usr/bin/python3

#from simple_description.simple_module import simple_function, simple_var
import numpy as np
import math
from math import sin,cos
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from simple_kinematics_interfaces.srv import GetPosition
from simple_kinematics_interfaces.srv import SolveIK
from visualization_msgs.msg import Marker


class DummyNode(Node):
    def __init__(self):
        self.joint = []
        super().__init__('dummy_node')
        self.joint1 = 0.0
        self.joint2 = 0.0
        self.joint3 = 0.0

        self.joint1_dot = 0.0
        self.joint2_dot = 0.0
        self.joint3_dot = 0.0

        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0

        self.x = 0.12
        self.y = 0.0
        self.z = 0.0

        self.x_dot = 0.0
        self.y_dot = 0.0
        self.z_dot = 0.0
        
        self.end_up = 0.0 

        self.id_count = 0

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription1 = self.create_subscription(JointState,'/joint_states',self.get_joint_states,10) #from rviz
        self.task_states_publihser = self.create_publisher(Float32MultiArray,'/task_states', 10) # to prox

        self.subscription2 = self.create_subscription(Float32MultiArray,'/reference/task_states',self.ref_task_states,10) #from generator
        self.ref_joint_states_publihser = self.create_publisher(Float32MultiArray,'/reference/joint_states', 10) # to tracker

        self.maker_publihser = self.create_publisher(Marker,'/robot_marker', 10) # to rviz

    def timer_callback(self):
        send_ref_joint_states = Float32MultiArray()
        send_ref_joint_states.data = [self.joint1,self.joint2, self.joint3, self.joint1_dot, self.joint2_dot, self.joint3_dot]
        self.ref_joint_states_publihser.publish(send_ref_joint_states)

    def get_joint_states(self,msg:JointState): 
        self.q1 = msg.position[0]
        self.q2 = msg.position[1]
        self.q3 = msg.position[2]
        self.solve_pos_fk(self.q1,self.q2,self.q3)


    def ref_task_states(self,msg:Float32MultiArray):
        self.y = msg.data[0]
        self.z = msg.data[1]
        self.y_dot = msg.data[2]
        self.z_dot = msg.data[3]
        self.end_up = msg.data[4]

        if self.end_up == 1.0:
            self.x = 0.1
        else:
            self.x = 0.12

        self.solve_pos_ik(self.x,self.y,self.z)
        self.solve_velo_ik(self.x_dot,self.y_dot,self.z_dot)

    def solve_pos_fk(self,j1,j2,j3):
        self.id_count += 1
        self.get_homo(j1,j2,j3)
        self.x = self.p0_e[0]
        self.y = self.p0_e[1]
        self.z = self.p0_e[2]
        send_task_states = Float32MultiArray()
        send_marker = Marker()

        send_marker.header.frame_id = 'world'
        send_marker.id = self.id_count 

        send_marker.type = Marker.SPHERE

        send_marker.action = Marker.ADD

        send_marker.scale.x = 0.01
        send_marker.scale.y = 0.01
        send_marker.scale.z = 0.01

        send_marker.color.r = 1.0
        send_marker.color.b = 0.0
        send_marker.color.a = 1.0
        send_marker.color.g = 0.0

        send_marker.pose.orientation.w = 1.0
        send_marker.pose.position.x = self.x
        send_marker.pose.position.y = self.y
        send_marker.pose.position.z = self.z

        send_task_states.data = [self.x,self.y,self.z]
        self.task_states_publihser.publish(send_task_states)

        if self.end_up == 0.0:
            self.maker_publihser.publish(send_marker)

        if self.id_count > 4.29*math.pow(10,9):
            self.id_count = 0

    def solve_pos_ik(self,x,y,z):
        #base_offset
        x = x + 0.025
        z = z - 0.03 #base
        z = z - 0.035 #joint1 to joint2
        d1 = 0.12
        d2 = 0.145
        config = -1

        r = math.sqrt((x*x)+(y*y)+(z*z))
        self.joint1 = math.atan2(y,x)

        c2 = ((x*x)+(y*y)+(z*z)-(d1*d1)-(d2*d2))/2/d1/d2
        s2 = config * np.sqrt(1-(c2*c2))
        self.joint3 = math.atan2(s2,c2) + (math.pi/2)
        self.joint2 = math.asin(z/r) - math.atan2(d2*s2,d1+(d2*c2)) - (math.pi/2)

        print("pose_ik",self.joint1,self.joint2,self.joint3)   

    def get_homo(self,q1,q2,q3):

        # ที่มาอยู่ในกระดาษทด
        h0_1 = np.array([[cos(q1), -sin(q1), 0, -0.025],
                [sin(q1) , cos(q1), 0, 0],
                [0, 0, 1, 0.03],
                [0, 0, 0, 1]])  #homogeneous matrix of joint 1 relative with base_link

        h1_2 = np.array([[cos(q2), -sin(q2), 0, 0],
                [0, 0, -1, 0],
                [sin(q2), cos(q2), 0, 0.035],
                [0, 0, 0, 1]])  #homogeneous matrix of joint 3 relative with joint 2

        h2_3 = np.array([[cos(q3), -sin(q3), 0, 0],
                [sin(q3) , cos(q3), 0, 0.12],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])  #homogeneous matrix of joint 1 relative with base_link

        h3_e = np.array([[1, 0, 0, 0.145],
                [0, 0, -1, 0],
                [0, 1, 0, 0],
                [0, 0, 0, 1]])  #homogeneous matrix of joint end effector relative with joint 3
   
        h0_2 = h0_1 @ h1_2
        h0_3 = h0_2 @ h2_3
        h0_e = h0_3 @ h3_e

        h0_1 = h0_1[(0,1,2),:]
        h0_2 = h0_2[(0,1,2),:]
        h0_3 = h0_3[(0,1,2),:]
        h0_e = h0_e[(0,1,2),:]

        p0_1 = h0_1[:,-1]       #Position ของ Joint 1 เทียบกับ Frame Gazebo 
        p0_2 = h0_2[:,-1]       #Position ของ Joint 2 เทียบกับ Frame Gazebo 
        p0_3 = h0_3[:,-1]       #Position ของ Joint 3 เทียบกับ Frame Gazebo
        self.p0_e = h0_e[:,-1]       #Position ของ End-effector เทียบกับ Frame Gazebo

        r0_1 = h0_1[:,(0,1,2)]  #Rotation Matrix ของ Joint 1 เทียบกับ Frame Gazebo
        r0_2 = h0_2[:,(0,1,2)]  #Rotation Matrix ของ Joint 2 เทียบกับ Frame Gazebo
        r0_3 = h0_3[:,(0,1,2)]  #Rotation Matrix ของ Joint 3 เทียบกับ Frame Gazebo
        r0_e = h0_e[:,(0,1,2)]  #Rotation Matrix ของ End-effector เทียบกับ Frame Gazebo

        z1 = r0_1[:,2]  #การดึงค่า Z1 จาก Rotation Matrix r0_1
        z2 = r0_2[:,2]  #การดึงค่า Z2 จาก Rotation Matrix r0_2
        z3 = r0_3[:,2]  #การดึงค่า Z3 จาก Rotation Matrix r0_3

        # Jacobain แถว 1-3 ซึ่งเป็นส่วนของ Angular Velocity
        J1 = [z1[0],z2[0],z3[0]]
        J2 = [z1[1],z2[1],z3[1]]
        J3 = [z1[2],z2[2],z3[2]]

        # Jacobain แถว 4-6 ซึ่งเป็นส่วนของ Linear Velocity โดยคำนวณตามแนวคิดที่อยู่ในกระดาษทดข้อแรก
        J4 = [np.cross(z1, (self.p0_e-p0_1))[0],np.cross(z2, (self.p0_e-p0_2))[0],np.cross(z3, (self.p0_e-p0_3))[0]]
        J5 = [np.cross(z1, (self.p0_e-p0_1))[1],np.cross(z2, (self.p0_e-p0_2))[1],np.cross(z3, (self.p0_e-p0_3))[1]]
        J6 = [np.cross(z1, (self.p0_e-p0_1))[2],np.cross(z2, (self.p0_e-p0_2))[2],np.cross(z3, (self.p0_e-p0_3))[2]]

        J = [J4,J5,J6]
        J = np.asarray(J)
        if (abs(np.linalg.det(J)) > 0.001):
            #print(J)
            self.inv = np.linalg.inv(J)
        else:
            print("Sing")
     

    def solve_velo_ik(self,vel_x,vel_y,vel_z):
        self.get_homo(self.joint1,self.joint2,self.joint3)
        self.velmat = np.array([[vel_x],[vel_y],[vel_z]])
        ans = self.inv @ self.velmat
        self.joint1_dot = ans[0][0]
        self.joint2_dot = ans[1][0]
        self.joint3_dot = ans[2][0]

        print("velo_ik",self.joint1_dot,self.joint2_dot,self.joint3_dot)
        
        
    def solve_velo_fk(self,j1,j2,j3):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
