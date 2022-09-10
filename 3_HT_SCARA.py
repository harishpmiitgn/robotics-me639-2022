import numpy as np
import math
import matplotlib.pyplot as plt

q1=30*np.pi/360
q2=60*np.pi/360
l1=2
l2=2.5
d=4
H1_0= np.array([[math.cos(q1), -math.sin(q1), 0, 0], [math.sin(q1), math.cos(q1), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
print(H1_0)

H2_1= np.array([[math.cos(q2), -math.sin(q2), 0, 0], [math.sin(q2), math.cos(q2), 0, l1], [0, 0, 1, 0], [0, 0, 0, 1]])
print(H2_1)

H3_2= np.array([[1, 0, 0, 0], [0, 1, 0, l2], [0, 0, 1, 0], [0, 0, 0, 1]])

p3= np.array([[0], [0], [-d], [1]])
print(p3)

p2= H3_2.dot(p3)
p1= H2_1.dot(p2)
p0= H1_0.dot(p1)
#position vector in ground frame
print(p0)  