import numpy as np

l1 = float(input('Length of link 1: '))
l2 = float(input('Length of link 2: '))
q1 = float(input('Angle of link 1: '))
q1 = (q1*np.pi)/180
q2 = float(input('Angle of link 2: '))
q2 = (q2*np.pi)/180
q3 = float(input('Displacement of link 3: '))

J = [[-1*l1*np.sin(q1)-1*l2*np.sin(q1+q2), -1*l2*np.sin(q1+q2), 0, 0], [l1*np.cos(q1)+l2*np.cos(q1+q2), l2*np.cos(q1+q2), 0, 0], [0, 0, -1, 0], [0, 0, 0, 0], [0, 0, 0, 0], [1, 1, 0, -1]]

for i in J:
    print(i)
