import numpy as np

def inverse_kinematics(R_36:np.ndarray) -> np.ndarray:

	if R_36[0, 2] != 0 or R_36[1, 2] !=0 :

		theta = np.arctan2(-np.sqrt(1 - R_36[2, 2]**2), R_36[2, 2])
		
		phi = np.arctan2(-R_36[1, 2],-R_36[0, 2])
		psi = np.arctan2(-R_36[2, 1], R_36[2, 0])

	else :

		phi = 0.

		if R_36[2, 2] > 0 :

			theta = 0.
			
			phi = np.arctan2(R_36[1, 0], R_36[0, 0])

		else :

			theta = np.pi

			phi = - np.arctan2(-R_36[0, 1],-R_36[0, 0])

	return np.array([phi, theta, psi])

if __name__=='__main__' :

	from modules import Robot

	state = (2 * np.random.random(3) - 1) * np.pi

	Wrist = Robot.Robot([
		Robot.RevoluteJoint(0, 0,-np.pi/2),
		Robot.RevoluteJoint(0, 0, np.pi/2),
		Robot.RevoluteJoint(0, 0, 0)
	])

	rotation_matrix = Wrist.getHomogenousTransform(state)[:3, :3]

	calc_state = inverse_kinematics(rotation_matrix)

	print("Random State: ", np.round(state, 2))
	print("Rotation Matrix :")
	print(np.round(rotation_matrix, 2))
	print("Inverse Kinematics:", np.round(calc_state, 2))