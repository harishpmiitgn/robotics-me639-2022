import numpy as np
from scipy.integrate import solve_ivp

# D-H Parameters
# theta			d		a	alpha
# q_1			0		0	pi/2
# q_2 + pi/2	0		0	pi/2
# 0				q_3		0	0

m = 0.1	# kg
l = 0.4 # kg

def D(q:np.ndarray) -> np.ndarray:

	I = m * (l**2) / 3

	return np.diag([I * (np.cos(q[1])**2), I, m])

def C(q:np.ndarray, q_dot:np.ndarray) -> np.ndarray :

	c = m * (l**2) * np.cos(q[1]) * np.sin(q[1])

	return np.array([
		[-c * q_dot[1],	-c * q_dot[0],	0],
		[ c * q_dot[0],	 0,				0],
		[ 0,			 0,				0]
	])

def getQddot(t:float, y:np.ndarray, tau:np.ndarray) -> np.ndarray :

	q = y[:3]
	q_dot = y[3:]

	return np.append(
		q_dot,
		np.linalg.solve(D(q), tau - np.matmul(C(q, q_dot), q_dot))
	)

