# Cubic polynomials are used generate trajectory in workspace coordinates
# Joint trajectories are obtained using inverse kinematics of workspace trajectories
# Animation shows end effector following desired workspace trajectory using
# forward kinematics on joint trajectories.
# This methodology of finding joint trajectories is used to preserve
# the straight lines in the workspace trajectory.

import numpy as np
import matplotlib.pyplot as plt

from modules.Trajectory import getCubicTrajectory
from modules.SCARARobot import inverseKinematics, getJointStateVelocity, getEndEffectorPosition

Dt = 1 # s

time_B = 1 * Dt # s
time_C = 2 * Dt # s
time_D = 3 * Dt # s
time_A = 4 * Dt # s

point_A = np.array([0.4,  0.06, 0.1])
point_B = np.array([0.4,  0.01, 0.1])
point_C = np.array([0.35, 0.01, 0.1])
point_D = np.array([0.35, 0.06, 0.1])

coefficients = np.zeros((4, 4, 3))

coefficients[0, :, :] = getCubicTrajectory(point_A, point_B, 0, time_B)
coefficients[1, :, :] = getCubicTrajectory(point_B, point_C, time_B, time_C)
coefficients[2, :, :] = getCubicTrajectory(point_C, point_D, time_C, time_D)
coefficients[3, :, :] = getCubicTrajectory(point_D, point_A, time_D, time_A)

def getDesiredPosition(t) :

	t = t % time_A
	i = int(t // Dt)

	return np.array([
		np.polyval(coefficients[i, :, 0], t),
		np.polyval(coefficients[i, :, 1], t),
		np.polyval(coefficients[i, :, 2], t),
	])

def getDesiredVelocity(t) :

	t = t % time_A
	i = int(t // Dt)

	return np.array([
		np.polyval(np.polyder(coefficients[i, :, 0], 1), t),
		np.polyval(np.polyder(coefficients[i, :, 1], 1), t),
		np.polyval(np.polyder(coefficients[i, :, 2], 1), t),
	])

def getDesiredJointState(t) :

	return inverseKinematics(getDesiredPosition(t))

def getDesiredJointVelocity(t) :

	return getJointStateVelocity(getDesiredJointState(t), getDesiredVelocity(t))

if __name__=='__main__' :

	from matplotlib.animation import FuncAnimation
	
	# Matplotlib objects
	figure1 = plt.figure(figsize=[6,6])

	axes = figure1.add_subplot(1, 1, 1)

	# Desired Position
	desired_position, = axes.plot([], [], 'o')

	axes.set_xlim([0.3, 0.5])
	axes.set_ylim([0.0, 0.2])

	# Animation functions
	def init_anim() :

		x = getEndEffectorPosition(getDesiredJointState(0))
		desired_position.set_data(x[0], x[1])

		return desired_position,

	def update_anim(t) :

		x = getEndEffectorPosition(getDesiredJointState(t))
		desired_position.set_data(x[0], x[1])

		return desired_position,

	points = np.column_stack((point_A, point_B, point_C, point_D, point_A))
	axes.plot(points[0], points[1])

	animation = FuncAnimation(figure1, update_anim, np.linspace(0, time_A, time_A*100 + 1), init_anim, interval=10)

	plt.show()