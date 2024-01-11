#!/usr/bin/python3
import numpy as np
import math
from HW2_utils import FKHW2

'''
    Name:   <Chakrapat> <Promasit> <63340500006>
            <Suparach> <Intarasopa> <63340500063>
    Date:   <31-10-65>
    Description: *******************
    ********************************ในส่วน code จะเป็นแค่การ comment ในเรื่องของการดึง function มาใช้และการแก้สมการคณิตศาสตร์เท่านั้น ส่วนที่มาและแนวคิดจะอยู่ในกระดาษทดครับ********************
'''

# Question 1
def endEffectorJacobianHW2(q):
    R,P,R_e,p_e = FKHW2(q) #เรียกใช้งาน function FKHW2 เพื่อนำค่า R(Rotation matrix ของแต่ละ Joint Frame เทียบกับ Base Frame) และค่า P(ซึ่งเป็น postion ในแต่ละ joint รวมถึง end-effector เทียบกับ Base Frame)
    R0_1 = np.asarray(R[:,:,0]) # Rotation matrix ของ Joint1 เทียบกับ Base Frame
    R0_2 = np.asarray(R[:,:,1]) # Rotation matrix ของ Joint2 เทียบกับ Base Frame
    R0_3 = np.asarray(R[:,:,2]) # Rotation matrix ของ Joint3 เทียบกับ Base Frame
    p1 = np.asarray(P[:,0]) # Position ของ Joint 1 เทียบกับ Base Frame 
    p2 = np.asarray(P[:,1]) # Position ของ Joint 2 เทียบกับ Base Frame 
    p3 = np.asarray(P[:,2]) # Position ของ Joint 3 เทียบกับ Base Frame 
    pend = np.asarray(P[:,3]) # Position ของ End-effector เทียบกับ Base Frame 

    z1 = R0_1[:,2] # การดึงค่า Z0_1 จาก Rotation Matrix R0_1
    z2 = R0_2[:,2] # การดึงค่า Z0_1 จาก Rotation Matrix R0_1
    z3 = R0_3[:,2] # การดึงค่า Z0_1 จาก Rotation Matrix R0_1

    # Jacobain แถว 1-3 ซึ่งเป็นส่วนของ Angular Velocity
    J1 = [z1[0],z2[0],z3[0]]
    J2 = [z1[1],z2[1],z3[1]]
    J3 = [z1[2],z2[2],z3[2]]

    # Jacobain แถว 4-6 ซึ่งเป็นส่วนของ Linear Velocity โดยคำนวณตามแนวคิดที่อยู่ในกระดาษทดข้อแรก
    J4 = [np.cross(z1, (pend-p1))[0],np.cross(z2, (pend-p2))[0], np.cross(z3, (pend-p3))[0]]
    J5 = [np.cross(z1, (pend-p1))[1],np.cross(z2, (pend-p2))[1], np.cross(z3, (pend-p3))[1]]
    J6 = [np.cross(z1, (pend-p1))[2],np.cross(z2, (pend-p2))[2], np.cross(z3, (pend-p3))[2]]

    J = [J1,J2,J3,J4,J5,J6]

    return J

# Question 2
def checkSingularityHW2(q):
    R,P,R_e,p_e = FKHW2(q) #เรียกใช้งาน function FKHW2 เพื่อนำค่า R(Rotation matrix ของแต่ละ Joint Frame เทียบกับ Base Frame) และค่า P(ซึ่งเป็น postion ในแต่ละ joint รวมถึง end-effector เทียบกับ Base Frame)
    R0_1 = np.asarray(R[:,:,0]) # Rotation matrix ของ Joint1 เทียบกับ Base Frame
    R0_2 = np.asarray(R[:,:,1]) # Rotation matrix ของ Joint2 เทียบกับ Base Frame
    R0_3 = np.asarray(R[:,:,2]) # Rotation matrix ของ Joint3 เทียบกับ Base Frame
    p1 = np.asarray(P[:,0]) # Position ของ Joint 1 เทียบกับ Base Frame 
    p2 = np.asarray(P[:,1]) # Position ของ Joint 2 เทียบกับ Base Frame 
    p3 = np.asarray(P[:,2]) # Position ของ Joint 3 เทียบกับ Base Frame 
    pend = np.asarray(P[:,3]) # Position ของ End-effector เทียบกับ Base Frame 

    z1 = R0_1[:,2] # การดึงค่า Z0_1 จาก Rotation Matrix R0_1
    z2 = R0_2[:,2] # การดึงค่า Z0_1 จาก Rotation Matrix R0_1
    z3 = R0_3[:,2] # การดึงค่า Z0_1 จาก Rotation Matrix R0_1

    # Jacobain แถว 1-3 ซึ่งเป็นส่วนของ Angular Velocity
    J1 = [z1[0],z2[0],z3[0]]
    J2 = [z1[1],z2[1],z3[1]]
    J3 = [z1[2],z2[2],z3[2]]

    # Jacobain แถว 4-6 ซึ่งเป็นส่วนของ Linear Velocity โดยคำนวณตามแนวคิดที่อยู่ในกระดาษทดข้อแรก
    J4 = [np.cross(z1, (pend-p1))[0],np.cross(z2, (pend-p2))[0], np.cross(z3, (pend-p3))[0]]
    J5 = [np.cross(z1, (pend-p1))[1],np.cross(z2, (pend-p2))[1], np.cross(z3, (pend-p3))[1]]
    J6 = [np.cross(z1, (pend-p1))[2],np.cross(z2, (pend-p2))[2], np.cross(z3, (pend-p3))[2]]


    # ทำการลดรูปของ Jacobain Matrix ให้เหลือขนาด 3*3 โดยจะทำการเลือกแค่ในส่วนของ Linear Velocity ในแกน XYZ 
    # เนื่องจากว่าแขนกลของเรานั้นเคลื่อนที่แบบ Task space ทำให้ปลายแขนของเรานั้นสามารถเปลี่ยนแปลงค่าแค่ในเชิงเส้นเท่านั้น
    J = [J4,J5,J6]

    #ทำการหา Determinent ของ matrix ที่ลดรูปแล้ว
    det = np.linalg.det(J)

    # ทำการ check ว่าตอนนี้แขนนั้นอยู่ใน singularity หรือไม่ โดยการเทียบจาก thereshold ที่โจทย์กำหนด
    if abs(det) < 0.001:
        flag = True
    else:
        flag = False
    #print(abs(det),flag)
    return flag




# Question 3
def computeEffortHW2(q,w):
    R,P,R_e,p_e = FKHW2(q) #เรียกใช้งาน function FKHW2 เพื่อนำค่า R(Rotation matrix ของแต่ละ Joint Frame เทียบกับ Base Frame) และค่า P(ซึ่งเป็น postion ในแต่ละ joint รวมถึง end-effector เทียบกับ Base Frame)
    R0_1 = np.asarray(R[:,:,0]) # Rotation matrix ของ Joint1 เทียบกับ Base Frame
    R0_2 = np.asarray(R[:,:,1]) # Rotation matrix ของ Joint2 เทียบกับ Base Frame
    R0_3 = np.asarray(R[:,:,2]) # Rotation matrix ของ Joint3 เทียบกับ Base Frame
    R0_end = np.asarray(R[:,:,3]) # Rotation matrix ของ End-effector เทียบกับ Base Frame

    p1 = np.asarray(P[:,0]) # Position ของ Joint 1 เทียบกับ Base Frame 
    p2 = np.asarray(P[:,1]) # Position ของ Joint 2 เทียบกับ Base Frame 
    p3 = np.asarray(P[:,2]) # Position ของ Joint 3 เทียบกับ Base Frame 
    pend = np.asarray(P[:,3]) # Position ของ End-effector เทียบกับ Base Frame 

    z1 = R0_1[:,2] # การดึงค่า Z0_1 จาก Rotation Matrix R0_1
    z2 = R0_2[:,2] # การดึงค่า Z0_1 จาก Rotation Matrix R0_1
    z3 = R0_3[:,2] # การดึงค่า Z0_1 จาก Rotation Matrix R0_1

    # Jacobain แถว 1-3 ซึ่งเป็นส่วนของ Angular Velocity
    J1 = [z1[0],z2[0],z3[0]]
    J2 = [z1[1],z2[1],z3[1]]
    J3 = [z1[2],z2[2],z3[2]]

    # Jacobain แถว 4-6 ซึ่งเป็นส่วนของ Linear Velocity โดยคำนวณตามแนวคิดที่อยู่ในกระดาษทดข้อแรก
    J4 = [np.cross(z1, (pend-p1))[0],np.cross(z2, (pend-p2))[0], np.cross(z3, (pend-p3))[0]]
    J5 = [np.cross(z1, (pend-p1))[1],np.cross(z2, (pend-p2))[1], np.cross(z3, (pend-p3))[1]]
    J6 = [np.cross(z1, (pend-p1))[2],np.cross(z2, (pend-p2))[2], np.cross(z3, (pend-p3))[2]]

    J = [J1,J2,J3,J4,J5,J6]

    #ทำการแยก moment และ force จาก input โดยค่าที่โจทย์ให้จะเป็นงานที่อ้างอิงกับ Frame end-effector
    nend = np.transpose([[w[0],w[1],w[2]]]) 
    fend = np.transpose([[w[3],w[4],w[5]]])

    # ทำการหา Moment และ Force ของ XYZ ที่ปลาย End-effector เทียบกับ Base Frame โดยการใช้ Rotation Matrix มาคูณเพื่อแปลง Frame
    f0 = R0_end @ fend
    n0 = R0_end @ nend

    wj1 = np.array([n0[0],n0[1],n0[2],f0[0],f0[1],f0[2]])

    # ทำการหาค่า tua ของแต่ละ joint โดยการนำ Transpose ของ Jacobian มาคูณกับงานของปลาย End-effector เทียบกับ Base Frame 
    J = np.transpose(J)
    tua = J @ wj1
    tua = np.transpose(tua)


    return tua
