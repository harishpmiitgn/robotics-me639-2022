import numpy as np

d1 = 1.0	# m

def inverse_kinematics(x, y, z) -> np.ndarray :

	theta1 = np.arctan2(y, x)

	r = np.sqrt(x**2 + y**2)
	s = z - d1

	theta2 = np.arctan2(s, r) + np.pi/2
	d3 = np.sqrt(r**2 + s**2)

	return np.array([theta1, theta2, d3])

if __name__=='__main__' :

	from modules import Robot

	x, y, z = np.random.rand(3) * 2

	state = inverse_kinematics(x, y, z)

	Stanford = Robot.Robot([
		Robot.RevoluteJoint(0, d1, np.pi/2),
		Robot.RevoluteJoint(0, 0, np.pi/2),
		Robot.PrismaticJoint(0, 0, 0)
	])

	end_effector = Stanford.getEndEffectorPosition(state)

	print("Desired Coordinates (m): ", np.round([x, y, z], 2))
	print("Robot State :", np.round(state, 2))
	print("Robot End-Effector Position (m):", np.round(end_effector, 2))