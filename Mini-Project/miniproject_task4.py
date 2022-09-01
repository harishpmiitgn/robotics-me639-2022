# Intro To Robotics (Mini-Project)
# written by Kush Patel (20110131)

import numpy as np
import math
import matplotlib.pyplot as plt
# number of samples
N = 1000

# link lengths
l1 = 6
l2 = 5

# joint angle limitations

Q1_min = 35
Q1_max = 145

Q2_min = 35
Q2_max = 145


Q1_start_end = np.array([Q1_min,Q1_max])
Q2_start_end = np.array([Q2_min,Q2_max])


Q1_start_end = Q1_start_end*math.pi/180
Q2_start_end = Q2_start_end*math.pi/180

Q1 = np.linspace(Q1_min,Q1_max,N)
Q2 = np.linspace(Q2_min,Q2_max,N)


Q1 = Q1*math.pi/180
Q2 = Q2*math.pi/180

x = np.zeros([2*len(Q1_start_end),len(Q2)])
y = np.zeros([2*len(Q1_start_end),len(Q2)])


for i in range(2):
  for j in range(len(Q1)):
    x[i][j] = l1*math.cos(Q1[j]) + l2*math.cos(Q1[j] + Q2_start_end[i])
    y[i][j] = l1*math.sin(Q1[j]) + l2*math.sin(Q1[j] + Q2_start_end[i])
  for k in range(len(Q1)):
    x[i+2][k] = l1*math.cos(Q1_start_end[i]) + l2*math.cos(Q1_start_end[i] + Q2[k])
    y[i+2][k] = l1*math.sin(Q1_start_end[i]) + l2*math.sin(Q1_start_end[i] + Q2[k])


x = np.transpose(x)
y = np.transpose(y)

x1 =[]
y1 = []
for i in range(x.shape[1]):
  for j in range(x.shape[0]):
    x1.append(x[j][i])
    y1.append(y[j][i])

plt.plot(x1,y1, "*", )
plt.show()