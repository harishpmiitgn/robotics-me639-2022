import numpy as np

l1, l2, l3 = 0.5, 0.5, 0.5	# m

def RRRJacobian(q1, q2, q3) :

	J = np.zeros((6, 3))

	J[0, 0] = -l1 * np.sin(q1) - l2 * np.sin(q1 + q2) - l3 * np.sin(q1 + q2 + q3)
	J[1, 0] =  l1 * np.cos(q1) + l2 * np.cos(q1 + q2) + l3 * np.cos(q1 + q2 + q3)

	J[0, 1] = -l2 * np.sin(q1 + q2) - l3 * np.sin(q1 + q2 + q3)
	J[1, 1] =  l2 * np.cos(q1 + q2) + l3 * np.cos(q1 + q2 + q3)

	J[0, 2] = -l3 * np.sin(q1 + q2 + q3)
	J[1, 2] =  l3 * np.cos(q1 + q2 + q3)

	J[5, :] = 1

	return J

print(RRRJacobian(np.pi / 2, np.pi / 3, np.pi / 6))