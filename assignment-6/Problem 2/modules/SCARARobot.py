import numpy as np

# D-H Parameters
# theta	d		a	 alpha
# q_1	0		l_1	 0
# q_2	0		l_2	 pi/2
# 0		q_3		0	 0

# Dynamics of links # 1 2 3
m = np.array([0.8, 0.8, 0.8]) 	# kg
l = np.array([0.25, 0.25, 0.25])	# m

q		= np.zeros(3)
q_dot	= np.zeros(3)

# Moment of Inertias
I1  = m[0] * (l[0]**2) / 3
I2  = m[0] * (l[1]**2) / 12
I3  = m[0] * (l[1]**2) / 3
I4  = I1 + I3 + m[1]*(l[0]**2)
I5  = m[1] * l[0] * l[1] / 2
I6  = I3
I8  = 2 * m[2] * (l[0]**2 + l[1]**2)
I9  = m[2] * (l[1]**2)
I10 = m[2] * (l[0]**2 + l[1]**2)
I11 = 2 * m[2] * (l[1]**2) + I10
I12 = m[2] * l[0] * l[1]
I13 = m[2] * l[0] * l[1]
I14 = I4 + I8
I15 = I5 + I13
I16 = I6 + I9 + 0.25 * I10
I17 = I3 + I11
I18 = I5 + 2 * (I12 + I13)

def D(q:np.ndarray) -> np.ndarray :

	return np.array([
		[I14 + 2 *I12 * np.cos(q[0]) + 2 * I15 * np.cos(q[1]),	0.5 * (I17 + I18 * np.cos(q[1])),	0],
		[0.5 * (I17 + I18 * np.cos(q[1])),						I16 + 0.5 * I13 * np.cos(q[1]),		0],
		[0,														0,									m[2]]
	])

def C(q:np.ndarray, q_dot:np.ndarray) -> np.ndarray :

	return np.array([
		[- 2 * I15 * np.sin(q[1]) * q_dot[1],	- 0.5 * I18 * np.sin(q[1]) * q_dot[1],	0],
		[  I15 * np.sin(q[1]) * q_dot[0],	  	- 0.25 * I13 * np.sin(q[1]) * q_dot[1],	0],
		[  0,				  					  0,									0]
	])

def getQDDot(q:np.ndarray, q_dot:np.ndarray, tau:np.ndarray) -> np.ndarray :

	return np.linalg.solve(
		D(q),
		tau - np.matmul(C(q, q_dot), q_dot)
	)

def getEndEffectorPosition(q:np.ndarray) -> np.ndarray :

	return np.array([
		  l[1] * np.cos(q[0] + q[1]) + l[0] * np.cos(q[0]),
		  l[1] * np.sin(q[0] + q[1]) + l[0] * np.sin(q[0]),
		- q[2]
	])

def getJacobian(q:np.ndarray) -> np.ndarray :

	J = np.zeros((6, 3))

	J[0, 0] = -l[0] * np.sin(q[0]) - l[1] * np.sin(q[0] + q[1])
	J[1, 0] =  l[0] * np.cos(q[0]) + l[1] * np.cos(q[0] + q[1])

	J[0, 1] = -l[1] * np.sin(q[0] + q[1])
	J[1, 1] =  l[1] * np.cos(q[0] + q[1])

	J[2, 2] = 1
	J[5, 0] = 1
	J[5, 1] = 1

	return J

def getEndEffectorVelocity(q:np.ndarray, q_dot:np.ndarray) -> np.ndarray :

	return np.matmul(getJacobian(q), q_dot)

def getJointStateVelocity(q:np.ndarray, vel:np.ndarray) -> np.ndarray :

	J = getJacobian(q)[:3, :]

	return np.linalg.solve(J, vel)

def getLink3OtherEnd(end_effector_position:np.ndarray) :

	return end_effector_position + [0, 0, l[2]]

def getLink12Ends(q:np.ndarray) -> np.ndarray :

	x = getEndEffectorPosition(q)

	return np.array([
		[	0,	0,	l[0] * np.cos(q[0]),	x[0]],
		[	0,	0,	l[0] * np.sin(q[0]),	x[1]],
		[-0.4,	0,	0, 						0],
	])

def getRandomJointState() ->  np.ndarray:

	return [-np.pi, -np.pi, 0] + np.random.random(3) * [np.pi, np.pi, l[2]]

def inverseKinematics(coordinates_array) -> np.ndarray :

	x, y, z = coordinates_array

	theta = np.arccos(
		(x**2 + y**2 - l[1]**2 - l[2]**2) /
		(2.0 * l[1] * l[2])
	)

	theta *= 1 - 2 * (np.signbit(x))

	phi		= np.arctan2(y, x)
	varphi	= np.arctan2(
		 l[2] * np.sin(theta),
		(l[1] + l[2] * np.cos(theta))
	)

	return np.array([
		phi + varphi,
		- theta,
		- z
	])