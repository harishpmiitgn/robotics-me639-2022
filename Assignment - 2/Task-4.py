import numpy as np

l2 = float(input('Length of link 2: '))
q1 = float(input('Angle of link 1: '))
q1 = (q1*np.pi)/180
q2 = float(input('Angle of link 2: '))
q2 = (q2*np.pi)/180
q3 = float(input('Displacement of link 3: '))

p3 = [[1], [2], [3], [1]]

H = [[np.cos(q1)*np.cos(q2), -1*np.sin(q1), np.cos(q1)*np.sin(q2), q3*np.cos(q1)*np.sin(q2)-l2*np.sin(q1)], [np.sin(q1)*np.cos(q2), np.cos(q1), np.sin(q1)*np.sin(q2), q3*np.sin(q1)*np.sin(q2)+l2*np.cos(q1)], [-1*np.sin(q2), 0, np.cos(q2), q3*np.cos(q2)], [0, 0, 0, 1]]

p0 = [[0], [0], [0], [0]]
for i in range(len(H)):
    for j in range(len(p3[0])):
        for k in range(len(p3)):
            p0[i][j] += H[i][k]*p3[k][j]

for r in p0:
    print(r)
