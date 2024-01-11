#!/usr/bin/python3
import math
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty as Empty_srv
from math import sin,cos,pi
import numpy as np
from sensor_msgs.msg import JointState

class Algorithm_publisher(Node):
        def __init__(self):
                super().__init__('Algorithm_publisher_node')
        
        # Define variable q for recieve value from Gazebo
                self.q1 = 0.0 
                self.q2 = 0.0
                self.q3 = 0.0
        # Define variable of velocity to send to Gazebo 
                self.joint_velo = np.array([0.0,0.0,0.0])
        # Define variable to collect twist data from Imu
                self.velmat = np.array([[0.0],[0.0]])
                self.velq3 = 0.0

                self.current_joint = self.create_subscription(Imu,'/imu/data',self.Imu_value,10) #return from complementary_filter
                self.Imu_raw_sub = self.create_subscription(Imu,'/Imu_arduino',self.Imu_raw_value,10)  #value from arduino
                self.raw_imu_publihser = self.create_publisher(Imu,'/imu/data_raw', 10) #send raw imu data to complementary_filter 
                self.joint_sub = self.create_subscription(JointState,'/joint_states',self.joint_sub,10) #current joint configurates from gazebo
                self.velo_publihser = self.create_publisher(Float64MultiArray,'/joint_velocity_controller/commands', 10) #joint velo control to Gazebo

        def Imu_raw_value(self,msg:Imu):
                self.raw_imu_publihser.publish(msg)  #send data to complementary filter when node get value from arduino

        def Imu_value(self,msg:Imu):

                #recieve value from arduino Imu
                angular_velo_z = msg.angular_velocity.z
                linear_acc_x = msg.linear_acceleration.x
                linear_acc_y = msg.linear_acceleration.y


                #Scale Imu value
                self.velq3 = np.around(angular_velo_z,2)
                self.velmat[1][0] = np.around((linear_acc_x*-1) / 100, 2)
                self.velmat[0][0] = np.around((linear_acc_y*-1) / 100, 2)

                #Solve Jacobian function
                self.Jacob(self.q1,self.q2,self.q3)

                
        def joint_sub(self,msg:JointState):
                #Get current joint config from Gazebo
                self.q1 = msg.position[0]
                self.q2 = msg.position[1]
                self.q3 = msg.position[2]


        def Jacob(self,q1,q2,q3):
                l0 = 0.035
                l1 = 0.12
                l2 = 0.175
                l3 = 0.05

                # ที่มาอยู่ในกระดาษทด
                h0_b = np.array([[0, -1, 0, 0],
                        [0, 0, -1, 0],
                        [1, 0, 0, l0],
                        [0, 0, 0, 1]]) #homogeneous matrix of base_link relative with Gazebo frame

                h0_1 = np.array([[cos(q1), -sin(q1), 0, 0],
                        [sin(q1) , cos(q1), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])  #homogeneous matrix of joint 1 relative with base_link
                h1_2 = np.array([[cos(q2), -sin(q2), 0, l1],
                        [sin(q2) , cos(q2), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])  #homogeneous matrix of joint 2 relative with joint 1

                h2_3 = np.array([[cos(q3), -sin(q3), 0, 0],
                        [0, 0, -1, -l2],
                        [sin(q3), cos(q3), 0, 0],
                        [0, 0, 0, 1]])  #homogeneous matrix of joint 3 relative with joint 2

                h3_e = np.array([[0, 0, 1, 0],
                        [0, -1, 0, 0],
                        [1, 0, 0, l3],
                        [0, 0, 0, 1]])  #homogeneous matrix of joint end effector relative with joint 3

                h0_1 = h0_b @ h0_1      
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
                p0_e = h0_e[:,-1]       #Position ของ End-effector เทียบกับ Frame Gazebo

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
                J4 = [np.cross(z1, (p0_e-p0_1))[0],np.cross(z2, (p0_e-p0_2))[0]]
                J5 = [np.cross(z1, (p0_e-p0_1))[1],np.cross(z2, (p0_e-p0_2))[1]]
                J6 = [np.cross(z1, (p0_e-p0_1))[2],np.cross(z2, (p0_e-p0_2))[2]]


                # ทำการลดรูปของ Jacobain Matrix ให้เหลือขนาด 2*2 โดยจะทำการเลือกแค่ในส่วนของ Linear Velocity ในแกน XZ
                # เนื่องจากว่าแขนกลของเรานั้นสามารถเคลื่อนที่ไดในแกน XZ เท่านั้น
                J = [J4,J6]

                J = np.asarray(J)
                det  = np.linalg.det(J)

                send_gazebo = Float64MultiArray()


                #ทำการ check singularity ของหุ่นยนต์
                if abs(det) > 0.002:
                        inv = np.linalg.inv(J)
                        ans = inv @ self.velmat

                        send_gazebo.data = [ans[0][0],ans[1][0],self.velq3]
                        print(p0_e)

                        self.velo_publihser.publish(send_gazebo)
   
                else:
                        send_gazebo.data = [0.0,0.0,self.velq3]
                        print(det)

                        self.velo_publihser.publish(send_gazebo)


def main(args=None):
        rclpy.init(args=args)
        joint_trajectory_object = Algorithm_publisher()
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


