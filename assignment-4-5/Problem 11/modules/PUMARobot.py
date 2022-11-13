import numpy as np
import modules.Robot as Robot

# D-H Parameters
# theta	d		a	 alpha
# q_1	0		l_1	 pi/2
# q_2	0		l_2	 0
# q_3	0		l_3	 0

# Dynamics of links # 1 2 3
m = np.array([0.2, 0.2, 0.2]) 	# kg
l = np.array([0.4, 0.4, 0.4])	# m

PUMA = Robot.Robot([
	Robot.RevoluteJoint(l[0], 0, np.pi/2),
	Robot.RevoluteJoint(l[1], 0, 0),
	Robot.RevoluteJoint(l[2], 0,-np.pi/2)
])

q		= np.zeros(3)
q_dot	= np.zeros(3)

D_Matrix = np.random.random((3, 3))
C_Matrix = np.random.random((3, 3, 3))

def D(q:np.ndarray) -> np.ndarray :

	return D_Matrix

def C(q:np.ndarray, q_dot:np.ndarray) -> np.ndarray :

	return np.matmul(q_dot, C_Matrix)

def getQDDot(q:np.ndarray, q_dot:np.ndarray, tau:np.ndarray) -> np.ndarray :

	return np.linalg.solve(
		D(q),
		tau - np.matmul(C(q, q_dot), q_dot)
	)

def getEndEffectorPosition(q:np.ndarray) -> np.ndarray :

	return PUMA.getEndEffectorPosition(q)

def getLinkEnds(q:np.ndarray) -> np.ndarray :

	T0 = PUMA.getHomogenousTransform(q, 1)
	T1 = PUMA.getHomogenousTransform(q, 2)
	T2 = PUMA.getHomogenousTransform(q, 3)
	
	x1 = np.matmul(T0, [0, 0, 0, 1])[:3]
	x2 = np.matmul(T1, [0, 0, 0, 1])[:3]
	x3 = np.matmul(T2, [0, 0, 0, 1])[:3]

	return np.column_stack((np.zeros(3), x1, x2, x3))

def getRandomJointState() ->  np.ndarray:

	return [-np.pi, -np.pi, -np.pi] + np.random.random(3) * [np.pi, np.pi, np.pi]