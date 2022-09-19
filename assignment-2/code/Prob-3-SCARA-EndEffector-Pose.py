import numpy as np

l1, l2 = 0.5, 0.5	# m

def SCARAEndEffectorPose(q1, q2, q3) :

	return np.array([
		l1 * np.cos(q1) + l2 * np.cos(q1 + q2),
		l1 * np.sin(q1) + l2 * np.sin(q1 + q2),
		q3
	])

print(SCARAEndEffectorPose(np.pi/6, np.pi/3, -0.4), 'm')