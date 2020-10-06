#! /usr/bin/env python3


"""
    # Shounak Chakraborty
    # shounakc@kth.se
"""
from math import *
import numpy as np

def scara_IK(point):
    
    L0=0.07
    L1=0.3
    L2=0.35
    
    x = point[0] - L0
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]

    #Fill in your IK solution here and return the three joint values in q
    
    
   
    #elbow angle degree
    q2= acos((pow(x,2)+ pow(y,2) - pow(L1,2) - pow(L2,2))/(2*L1*L2))

    #shoulder angle degree
    q1= atan2(y,x) - acos((x**2 + y**2 + L1**2 - L2**2)/(2*L1*sqrt(x**2 +y**2)))
    #q1= atan2(y,x) - atan2(L2*sin(q2),(L1+ L2*cos(q2)))
    #q1= atan2((y*(L1+L2*cos(q2))- x*L2*sin(q2)),(x*(L1+L2*cos(q2))+y*L2*sin(q2)))

    #prismatic joint    
    q3= z

    q = [q1, q2, q3]
    #rospy.loginfo("revolute joints:------ %s ,%s", q1, q2 )
    #rospy.loginfo("prismatic joint:------ %s ", q3 )
    
    
    
    return q

def kuka_IK(point, R, joint_positions):
    
## D-H parameters syms - length, offset and rotational angles

    x = point[0]
    y = point[1]
    z = point[2]

    q = joint_positions

    L0_2 = 0.311
    L = 0.4
    M = 0.39
    L6_7 = 0.078

    #distance between links 
    
 
    
   
    
    while(True):

    

    # Creating individual transformation matrices
        T0_1 = [[cos(q[0]),0,sin(q[0]),0], [sin(q[0]),0,-cos(q[0]),0],   [0,1,0,0],   [0,0,0,1]]
        T1_2 = [[cos(q[1]),0,-sin(q[1]),0],[sin(q[1]),0,cos(q[1]),0],    [0,-1,0,0],  [0,0,0,1]]
        T2_3 = [[cos(q[2]),0,-sin(q[2]),0],[sin(q[2]),0,cos(q[2]),0],    [0,-1,0,L],  [0,0,0,1]]
        T3_4 = [[cos(q[3]),0,sin(q[3]),0], [sin(q[3]),0,-cos(q[3]),0],   [0,1,0,0],   [0,0,0,1]]
        T4_5 = [[cos(q[4]),0,sin(q[4]),0], [sin(q[4]),0,-cos(q[4]),0],   [0,1,0,M],   [0,0,0,1]]
        T5_6 = [[cos(q[5]),0,-sin(q[5]),0],[sin(q[5]),0,cos(q[5]),0],    [0,-1,0,0],  [0,0,0,1]]
        T6_7 = [[cos(q[6]),-sin(q[6]),0,0],[sin(q[6]),cos(q[6]),0,0],    [0,0,1,0],   [0,0,0,1]]
        
        
    
    # Mathematical equivalents
       #T0_1 = [[cos(q[0]),0,sin(q[0]),0],[sin(q[0]),0,-cos(q[0]),0],[0,1,0,0],[0,0,0,1]]
        T0_2 = np.dot(T0_1,T1_2)
        T0_3 = np.dot(T0_2,T2_3)
        T0_4 = np.dot(T0_3,T3_4)
        T0_5 = np.dot(T0_4,T4_5)
        T0_6 = np.dot(T0_5,T5_6)
    # Final product of matrix multiplication
        T0_7 = np.dot(T0_6,T6_7)

    # Position matrices
    # Getting the fourth column viz. Translational part from the Transformation matrix T and then extracting the first three elements
        P0 = [0,0,0]
        P1 = np.dot(T0_1,[0,0,0,1])
        P1 = P1[0:3]
        P2 = np.dot(T0_2,[0,0,0,1])
        P2 = P2[0:3]
        P3 = np.dot(T0_3,[0,0,0,1])
        P3 = P3[0:3]
        P4 = np.dot(T0_4,[0,0,0,1])
        P4 = P4[0:3]
        P5 = np.dot(T0_5,[0,0,0,1])
        P5 = P5[0:3]
        P6 = np.dot(T0_6,[0,0,0,1])
        P6 = P6[0:3]
        
        P = np.dot(T0_7,[0,0,0,1])
        P = P[0:3]

    # Getting the third column of the Rotational part from the Transformation matrix and then extracting the first three elements
        Z0 = [0,0,1]
        R1 = np.dot(T0_1,[0,0,1,0])
        Z1 = R1[0:3]
        R2 = np.dot(T0_2,[0,0,1,0])
        Z2 = R2[0:3]
        R3 = np.dot(T0_3,[0,0,1,0])
        Z3 = R3[0:3]
        R4 = np.dot(T0_4,[0,0,1,0])
        Z4 = R4[0:3]
        R5 = np.dot(T0_5,[0,0,1,0])
        Z5 = R5[0:3]
        R6 = np.dot(T0_6,[0,0,1,0])
        Z6 = R6[0:3]

        #Subtracting the end effector postion with the each of the relative positions
        P_0= P-P0
        P_1= P-P1
        P_2= P-P2
        P_3= P-P3
        P_4= P-P4
        P_5= P-P5
        P_6= P-P6
        
        #Finding the Jacobian       
        
        J = np.transpose([np.concatenate((np.cross(Z0,P_0), Z0)), 
                    np.concatenate((np.cross(Z1,P_1), Z1)), 
                    np.concatenate((np.cross(Z2,P_2), Z2)), 
                    np.concatenate((np.cross(Z3,P_3), Z3)), 
                    np.concatenate((np.cross(Z4,P_4), Z4)), 
                    np.concatenate((np.cross(Z5,P_5), Z5)), 
                    np.concatenate((np.cross(Z6,P_6), Z6))])

        #Taking the inverse of the Jacobian
        J_inv = np.linalg.pinv(J)
      
        # Finding the current position with the fiven offsets and desired position
        current_position = np.dot(T0_7,[0, 0, L6_7, 1])
        current_position = current_position[0:3]
        current_position[2] =   current_position[2] + L0_2
    
        desired_position = [x, y, z]
   
        position_error = current_position - desired_position

        orientation_error = 1/2*(  np.cross(np.dot(R, [1, 0, 0]), np.dot(T0_7,[1, 0, 0, 0])[0:3]) 
                            + np.cross(np.dot(R, [0, 1, 0]), np.dot(T0_7,[0, 1, 0, 0])[0:3]) 
                            + np.cross(np.dot(R, [0, 0, 1]), np.dot(T0_7,[0, 0, 1, 0])[0:3]))
    
        total_error = np.concatenate((position_error, orientation_error))
        
        joint_error = np.dot(J_inv, total_error)
        
        #Taking the Norm of the joint error
        joint_err_norm = np.linalg.norm(joint_error)

        if joint_err_norm < 0.01:
            break
            
        q = q - joint_error
        q = np.array(q).reshape(7,).tolist()
    
    return q

    
  












    
   



       