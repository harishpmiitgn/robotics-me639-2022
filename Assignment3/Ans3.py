import math
from math import sin, cos
from operator import matmul
import numpy as np 

def solve(N, DH_param, joint_vel, joints = []):  # DH = [a, alpha , d, theta], input alpha and theta in degrees.

    def T(N,DH):
        mat_list = []
        for i in range(N):
            a = DH[i][0]
            alpha = math.radians(DH[i][1])
            d = DH[i][2]
            theta = math.radians(DH[i][3])

            A_matrix = [[cos(theta), -(sin(theta))*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],[sin(theta), (cos(theta))*cos(alpha), -(cos(theta))*sin(alpha), a*sin(theta)], [0,sin(alpha),cos(alpha),d], [0,0,0,1]]
            A_matrix = np.array(A_matrix)
            mat_list.append(A_matrix)

        T_mat = mat_list[0]

        for j in mat_list[1:]:
            T_mat = matmul(T_mat, j)
        
        z_N = np.array([[T_mat[0][2]],[T_mat[1][2]],[T_mat[2][2]]])
        d_N = np.array([[T_mat[0][3]],[T_mat[1][3]],[T_mat[2][3]]])
        return (z_N,d_N)

    if joints == []: 
        joints = ['R']*N
    
    Jacob = None
    for i in range(len(joints)):
        if joints[i] == 'R':
            if i == 0:
                z_cross_O = np.transpose(np.cross(np.array([0,0,1]), np.transpose(T(N,DH_param)[1])))
                z = np.array([[0],[0],[1]])
                Jacob = np.concatenate((z_cross_O,z), axis = 0)
            else:
                z_cross_O = np.transpose(np.cross(np.transpose(T(i,DH_param)[0]), np.transpose(T(N,DH_param)[1] - T(i,DH_param)[1])))
                z = T(i,DH_param)[0]
                Jacob_i = np.concatenate((z_cross_O,z), axis = 0)
                Jacob = np.concatenate((Jacob,Jacob_i),axis = 1)    

        if joints[i] == 'P':
            if i == 0:
                z = np.array([[0],[0],[1]])
                zero_mat = np.array([[0],[0],[0]])
                Jacob = np.concatenate((z,zero_mat), axis = 0)
            else:
                z = T(i,DH_param)[0]
                zero_mat = np.array([[0],[0],[0]])
                Jacob_i = np.concatenate((z,zero_mat), axis = 0)
                Jacob = np.concatenate((Jacob,Jacob_i),axis = 1)
    
    Coord = (T(N,DH_param)[1]).round(4)
    velocity = (matmul(Jacob,joint_vel)).round(4)
    Jacob = Jacob.round(4)
    return (Coord,velocity,Jacob)

#currently DH parameters for question 6 of the assignment 3 is given as the input. 
#You can check for different manipulator and their different configuration by changing 
# the variables DH_P, Joint_vel, N, joints_.

#Input no.of joints/links
N = 6 

#input DH parameters 
#Note that it should be a NX4 array.
#Input alpha and theta parameters in degrees.
DH_p = np.array([[0,90,3,30],[4,0,0,60],[5,0,0,45],[0,-90,0,0],[0,-90,0,45],[0,0,6,0]])   #[a,alpha,d,theta] 

#Input Joint velocities
Joint_vel = np.array([[10],[10],[10],[0],[0],[0]]) #Note that the length of the array should be equal to variable N

#Input joint type.  string 'R' for revolute and string 'P' for prismatic joint. 
joints_ = ['R','R','R','R','R','R'] #Note that the length of the list should be equal to variable N
  
coord, velocity,J = solve(N,DH_p,Joint_vel,joints=joints_)

print(DH_p)
print('End Effector position')
print(coord)
print()
print('End Effector Velocity')
print(velocity)
print()
print('Jacobian Matrix')
print(J)
