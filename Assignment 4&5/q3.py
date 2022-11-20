import numpy as np

jacob = [[1, 3, 5], [2, 4, 6], [0, 0, 0]]
end_vel = [[1],[1],[1]]

joint_vel = np.dot(jacob, end_vel)
print(joint_vel)