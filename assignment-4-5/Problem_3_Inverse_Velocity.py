import numpy as np

from modules.Robot import Robot, RevoluteJoint, PrismaticJoint

# DH - a, d, alpha, theta

def getJointVelocity(num_links:int, DH_matrix:np.ndarray, vel:np.array, joint_type_array:list=None) :

	if DH_matrix.shape != (num_links, 4) and vel.shape[0] != num_links : raise ValueError('Dimension mismatch')

	if joint_type_array is None : joint_type_array = ['R' for i in range(num_links)]

	elif len(joint_type_array) != num_links : raise ValueError('Dimension mismatch')

	robot = Robot()
	state = []

	for i in range(num_links) :

		if joint_type_array[i] == 'R' :

			robot.joint_list.append(RevoluteJoint(DH_matrix[i, 0], DH_matrix[i, 1], DH_matrix[i, 2]))
			state.append(DH_matrix[i, 3])

		elif joint_type_array[i] == 'P' :

			robot.joint_list.append(PrismaticJoint(DH_matrix[i, 0], DH_matrix[i, 1], DH_matrix[i, 3]))
			state.append(DH_matrix[i, 2])

	return robot.getJointVelocity(np.array(state), vel)