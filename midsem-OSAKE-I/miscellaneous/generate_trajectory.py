import numpy as np

l1 = 0.1
l2 = 0.1

def inverse_kinematics(x, y) :

    q2 = - np.arccos((x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2))
    q1 = np.arccos((x * (l1 + l2 * np.cos(q2)) + y * l2 * np.sin(q2)) / (x**2 + y**2))

    return q1, q2

# Circular Trajectory
# t = np.linspace(0, 1, 500)
# x = 0.025 * np.sin(2 * np.pi * t)
# y = 0.175 + 0.025 * np.cos(2 * np.pi * t)
# np.savetxt('trajectory_circle.txt', np.row_stack((t, q1, q2)), delimiter=',', fmt='%.5e')


# Linear Trajectory
t = np.linspace(0, 1, 500)
x = np.linspace(0.0,0.0865, 500)#x = np.linspace(0.0, 0.1366, 500)
y = np.linspace(0.2,-0.0485, 500)#y = np.linspace(0.2, 0.1366, 500)

q1, q2 = inverse_kinematics(x, y)

np.savetxt('trajectory_linear_force.txt', np.row_stack((t, q1, q2)), delimiter=',', fmt='%.5e')

