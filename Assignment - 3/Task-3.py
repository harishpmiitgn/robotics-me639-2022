import numpy as np
import sympy as sp

n = int(input("Enter the no. of links: "))
matr = input("Enter the DH Parameters in Matrix form, eg:-[[alpha1,theta1,d1,a1],[alpha2,theta2,d2,a2]..] :")
links = matr.removeprefix("[[").removesuffix("]]").split("],[")
alphas,thetas,offset,length = [],[],[],[]

def DH_cal(theta,alpha,d,a):
    dh = sp.Matrix([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), np.cos(theta)*a],
    [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), np.sin(theta)*a],
    [0, np.sin(alpha), np.cos(alpha),d],
    [0,0,0,1]])
    return dh

qi_dot = input("Enter the joint velocities, eg: q1,q2,q3..:")
qi_dot_array = []

for q in qi_dot.split(','):
    qi_dot_array.append(float(q))

qi_dot_array = sp.Matrix(qi_dot_array)

O,z0,o0 = [],np.array([0,0,1]),np.array([0,0,0])
J = []

if len(links) == n:
    for link in links:
        params = link.split(",")
        alphas.append(float(params[0]))
        thetas.append(float(params[1]))
        offset.append(float(params[2]))
        length.append(float(params[3]))
    t = sp.Matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    
    for i in range(0,n):
        t = t*DH_cal(thetas[i],alphas[i],offset[i],length[i])
        O.append(np.array(t.col(3)[0:3]))
    # print(np.cross(z0,O[n-1]-o0))
    c=0
    J.append(np.append(np.cross(z0,O[n-1]-o0),[0,0,1]))
    
    for o in O[0:n-1]:
        c+=1
        J.append(np.append(np.cross(z0,O[n-1]-o),[0,0,1]))
        # print(np.append(np.cross(z0,O[n-1]-o),[0,0,1]))
    print("End Effector Position = "+ str(np.dot(t,[0,0,0,1])[0:3]))
    # print(J)
    J = sp.Matrix(np.transpose(J))
    print(J)
    print(J*qi_dot_array)
else:
    print("Please provide correct input.")
