import numpy as np
from numpy import pi, sin, cos, sqrt, absolute, arccos, arctan, sign
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd
l1,l2 = 10,10
m1,m2 = 0.2,0.2
v1,v2 = 2,2
l = input("Enter the length of links as l1,l2 :")
l1 = float(l.split(",")[0])
l2 = float(l.split(",")[1])
m = input("Enter the mass of links as m1,m2 :")
m1 = float(m.split(",")[0])
m2 = float(m.split(",")[1])
v = input("Enter the high and low velocities as v1,v2 :")
v1 = float(v.split(",")[0])
v2 = float(v.split(",")[1])
p0 = input("Enter start point as x,y :")
x0,y0 = float(p0.split(",")[0]),float(p0.split(",")[0])
p1 = input("Enter end point as x,y :")
x1,y1 = float(p1.split(",")[0]),float(p1.split(",")[0])
t1_arr, t2_arr = [],[]
s1_arr, s2_arr = [],[]
j1_arr,j2_arr = [],[]
#assuming linear interpolation
for x2 in np.arange(x0, x1+0.2, 0.2):
    y2 = (y1-y0)*x2/(x1-x0) + (x1*y0-x0*y1)/(x1-x0)
    theta2 = -arccos((x2**2+y2**2-l1**2-l2**2)/(2*l1*l2))
    theta1 = arccos((x2*(l1+l2*cos(theta2))+y2*l2*sin(theta2))/(x2**2 +y2**2))
    q1,q2 = theta1,theta2
    j1,j2 = l2*cos(q2)*v1 + l2*sin(q2)*v1, -l1*cos(q1)*v1 - l1*sin(q1)*v1
    a1,a2 = j2*(-l2*sin(q2)*v1 + l2*cos(q2)*v1), j1*(l1*sin(q1)*v1 - l1*cos(q1)*v1) 
    t1 = m1*l1**2*a1/3 + m2*l1*2*a1 + m2*l1*l2*a2*cos(q2-q1)/2 - m2*l1*l2*j2*(j2-j1)*sin(q2-q1)/2 + m1*9.81*l1*cos(q2)/2 + m2*9.81*l1*cos(q2)
    t2 = m2*l2**2*a1/2 + m2*l2**2*a2/l1 + m2*l1*l2*a1*cos(q2-q1)/2 - m2* l1*l2*j1*(j2-j1)*sin(q2-q1)/2 + m2*9.81*l2*sin(q2)/2
    t1_arr.append(t1)
    t2_arr.append(t2)
    j1_arr.append(j1)
    j2_arr.append(j2)
v1=v2
for x2 in np.arange(x0, x1+0.2, 0.2):
    y2 = (y1-y0)*x2/(x1-x0) + (x1*y0-x0*y1)/(x1-x0)
    theta2 = -arccos((x2**2+y2**2-l1**2-l2**2)/(2*l1*l2))
    theta1 = arccos((x2*(l1+l2*cos(theta2))+y2*l2*sin(theta2))/(x2**2 +y2**2))
    q1,q2 = theta1,theta2
    j1,j2 = l2*cos(q2)*v1 + l2*sin(q2)*v1, -l1*cos(q1)*v1 - l1*sin(q1)*v1
    a1,a2 = j2*(-l2*sin(q2)*v1 + l2*cos(q2)*v1), j1*(l1*sin(q1)*v1 - l1*cos(q1)*v1) 
    t1 = m1*l1**2*a1/3 + m2*l1*2*a1 + m2*l1*l2*a2*cos(q2-q1)/2 - m2*l1*l2*j2*(j2-j1)*sin(q2-q1)/2 + m1*9.81*l1*cos(q2)/2 + m2*9.81*l1*cos(q2)
    t2 = m2*l2**2*a1/2 + m2*l2**2*a2/l1 + m2*l1*l2*a1*cos(q2-q1)/2 - m2* l1*l2*j1*(j2-j1)*sin(q2-q1)/2 + m2*9.81*l2*sin(q2)/2
    s1_arr.append(t1)
    s2_arr.append(t2)

print(j1_arr)
print(t1_arr)
# fig, axes = plt.subplots(2,1)
df = pd.DataFrame(columns=['Torque at joint 1','Torque at joint 2'])#,'High Velocity Torque at joint 1','High Velocity Torque at joint 2'
# df['Low Velocity Torque at joint 2'] = t2_arr
# df['Low Velocity Torque at joint 1'] = t1_arr
df['Torque at joint 1'] = t1_arr
df['Torque at joint 2'] = t2_arr
df2 = pd.DataFrame(columns=['Torque at joint 1','Torque at joint 2'])
df2['Torque at joint 1'] = s1_arr
df2['Torque at joint 2'] = s2_arr
df.plot() 
df2.plot()
#df.plot()
plt.show()
# fig = plt.figure()
# ax = fig.add_subplot(
#     111, aspect="equal", autoscale_on=True, xlim=(, 10), ylim=(-50, 100)
# )

# # add grid lines, title and take out the axis tick labels
# ax.grid(alpha=1)
# ax.set_title("Elbow Manipulator")
# ax.set_xticklabels([])
# ax.set_yticklabels([])
# (line, ) = ax.plot(
#     t1_arr, np.arange(x0, x1+0.2, 0.2), "o-", lw=2, color="#2b8cbe"
# ) 
# # (line, ) = ax.plot(
# #     t2_arr, t2_arr, "o-", lw=2, color="#2b8cbe"
# # )
# plt.show()