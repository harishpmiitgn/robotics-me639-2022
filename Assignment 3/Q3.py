from cmath import pi
import numpy
#enter the DH Parameter table below in matrix form
#Enter theta_i, d_i, a_i and alpha_i (enter angles in degrees)
n=int(input("Enter the number of links: "))
DH=[]
for i in range(n):
    print("Link ",i+1)
    a=float(input("Enter theta_i (in degrees): "))
    b=float(input("Enter d_i: "))
    c=float(input("Enter a_i: "))
    d=float(input("Enter alpha_i (in degrees): "))
    DH.append([a,b,c,d])
#Make the below array with length equal to your number of joints
#Enter 1 at the joint number if the joint is prismatic
#Else, enter 0 if it is R joint
print("Enter the joint constants in series. 0 for R and 1 for P.")
RP=[int(i) for i in input("For example if RRP, type 0 0 1. Enter 9 to skip this step: ").split()]
if RP[0]==9:
    RP=[0]*n
for i in range(len(DH)):
    DH[i][0]=DH[i][0]*pi/180
    DH[i][3]=DH[i][3]*pi/180
A=[]
for i in range(len(DH)):
    t,d,a,al=DH[i]
    A.append(numpy.array([[numpy.cos(t), -numpy.sin(t)*numpy.cos(al), numpy.sin(t)*numpy.sin(al), a*numpy.cos(t)],[numpy.sin(t), numpy.cos(t)*numpy.cos(al), -numpy.cos(t)*numpy.sin(al), a*numpy.sin(t)],[0, numpy.sin(al), numpy.cos(al), d],[0, 0, 0, 1]]))
tr=A[0]
for i in range(len(DH)-1):
   tr=tr@A[i+1]
for i in range(4):
    for j in range(4):
        tr[i][j]=round(tr[i][j],6)
print("Transformation matrix T:")
print(tr)
print("Position of the end effector:")
print(tr[0:3,3].T)
print("Orientation of the end effector:")
print(tr[0:3,0:3])
temp=numpy.identity(4)
o=[0]*(len(DH)+1)
o[0]=numpy.array([0, 0, 0])
z=[0]*(len(DH)+1)
z[0]=numpy.array([[0],[0],[1]])
for i in range(len(DH)):
    temp=temp@A[i]
    o[i+1]=temp[0:3,3].T
    z[i+1]=temp[0:3, 0:3]@z[0]
Jt=[0]*len(DH)
for i in range(len(DH)):
    if RP[i]==0:
        Jt[i]=numpy.concatenate((numpy.cross(z[i].T,(o[len(DH)]-o[i]).T).T,z[i]),axis=0)
    else:
        Jt[i]=numpy.concatenate((z[i],numpy.array([[0,0,0]]).T),axis=0)
J=Jt[0]
for i in range(len(DH)-1):
    J=numpy.concatenate((J,Jt[i+1]),axis=1)
for i in range(len(J)):
    for j in range(len(J[i])):
        J[i][j]=round(J[i][j],6)
print("Jacobian Matrix:")
print(J)
#Joint velocities q_dot and d_dot 
dots=[float(i) for i in input("Enter q_dot or d_dot for joints in series seperated by space: ").split()]
print("Velocity of the end effector:")
print((J@dots)[0:3])