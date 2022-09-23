import numpy as np

a1 = 0.055	# m

l1 = 0.1	# m
l2 = 0.1	# m

def getEndEffectorPosition(q1:float, q2:float) -> np.ndarray :

	return np.array([
		l1 * np.cos(q1) + l2 * np.cos(q1 + q2),
		l1 * np.sin(q1) + l2 * np.sin(q1 + q2),
		a1
	])

def getJacobian(q1:float, q2:float) -> np.ndarray :

	return np.array([
		[- l2 * np.sin(q1 + q2) - l1 * np.sin(q1),	- l2 * np.sin(q1 + q2)],
		[  l2 * np.cos(q1 + q2) + l1 * np.cos(q1),	  l2 * np.cos(q1 + q2)],
		[  0,										  0],
		[  0,										  0],
		[  0,										  0],
		[  1,										  1],
	])

def getEndEffectorVelocity(q1:float, q2:float, q1_dot:float, q2_dot:float) -> np.ndarray :

	return np.matmul(
		getJacobian(q1, q2),
		np.array([
			[q1_dot],
			[q2_dot]
		])
	)

