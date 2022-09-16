import numpy as np

l1, l2 = 0.5, 0.5	# m

def SCARAJacobian(q1, q2, q3) :

	J = np.zeros((6, 3))

	J[0, 0] = -l1 * np.sin(q1) - l2 * np.sin(q1 + q2)
	J[1, 0] =  l1 * np.cos(q1) + l2 * np.cos(q1 + q2)

	J[0, 1] = -l2 * np.sin(q1 + q2)
	J[1, 1] =  l2 * np.cos(q1 + q2)

	J[2, 2] = 1
	J[5, 0] = 1
	J[5, 1] = 1

	return J

print(SCARAJacobian(np.pi / 2, np.pi / 3, -0.7))