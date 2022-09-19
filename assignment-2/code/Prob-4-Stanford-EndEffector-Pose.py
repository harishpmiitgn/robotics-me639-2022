import numpy as np

l1 = 0.6 # m

def StanfordEndEffectorPose(q1, q2, q3) :

	return np.array([
		q3 * np.sin(q2) * np.cos(q1),
		q3 * np.sin(q2) * np.sin(q1),
		q3 * np.cos(q2) + l1
	])

print(StanfordEndEffectorPose(np.pi/6, np.pi/3, -0.4), 'm')
