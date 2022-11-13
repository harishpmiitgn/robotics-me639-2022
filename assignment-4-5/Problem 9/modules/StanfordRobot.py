import numpy as np

# D-H Parameters
# theta			d		a	alpha
# q_1			0		0	pi/2
# q_2 + pi/2	0		0	pi/2
# 0				q_3		0	0

# Only dynamics of link corresponding to joint#3 is considered
m = 0.2	# kg
l = 0.4	# m

q		= np.zeros(3)
q_dot	= np.zeros(3)

def D(q:np.ndarray) -> np.ndarray :

	I = m * (l**2) / 3

	return np.diag([I * (np.cos(q[1])**2), I, m])


def C(q:np.ndarray, q_dot:np.ndarray) -> np.ndarray :

	c = m * (l**2) * np.sin(2 * q[1]) / 6

	return np.array([
		[- c * q_dot[1],	- c * q_dot[0],	0],
		[  c * q_dot[0],	  0,			0],
		[  0,				  0,			0]
	])

def getQDDot(q:np.ndarray, q_dot:np.ndarray, tau:np.ndarray) -> np.ndarray :

	return np.linalg.solve(
		D(q),
		tau - np.matmul(C(q, q_dot), q_dot)
	)

def getEndEffectorPosition(q:np.ndarray) -> np.ndarray :

	return np.array([
		q[2] * np.sin(q[1]) * np.cos(q[0]),
		q[2] * np.sin(q[1]) * np.sin(q[0]),
		q[2] * np.cos(q[1])
	])

def getLinkOtherEnd(end_effector_position:np.ndarray) :

	q = np.sqrt(
		end_effector_position[0]**2 +
		end_effector_position[1]**2 +
		end_effector_position[2]**2
	)

	return (1 - l/q) * end_effector_position

def getRandomJointState() ->  np.ndarray:

	return [-np.pi, 0, 0] + np.random.random(3) * [np.pi, np.pi, l]