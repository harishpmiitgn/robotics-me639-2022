import numpy as np

d1 = 1.0	# m
d2 = 1.0	# m
d3 = 1.0	# m

def inverse_kinematics(x, y, z) -> np.ndarray :

	theta = np.arccos(
		(x**2 + y**2 - d2**2 - d3**2) /
		(2.0 * d2 * d3)
	)

	theta *= 1 - 2 * (np.signbit(x))

	phi		= np.arctan2(y, x)
	varphi	= np.arctan2(
		 d3 * np.sin(theta),
		(d2 + d3 * np.cos(theta))
	)

	return np.array([
		phi + varphi,
		- theta,
		d1 - z
	])

if __name__=='__main__' :

	from modules import Robot

	x, y, z = np.random.rand(3) * [np.sqrt(d2**2 + d3**2), np.sqrt(d2**2 + d3**2), d1]

	state = inverse_kinematics(x, y, z)

	SCARA = Robot.Robot([
		Robot.RevoluteJoint(d2, d1, 0.0),
		Robot.RevoluteJoint(d3, 0.0, np.pi),
		Robot.PrismaticJoint(0.0, 0.0, 0.0)
	])

	end_effector = SCARA.getEndEffectorPosition(state)

	print("Desired Coordinates (m): ", np.round([x, y, z], 2))
	print("Robot State :", np.round(state, 2))
	print("Robot End-Effector Position (m):", np.round(end_effector, 2))