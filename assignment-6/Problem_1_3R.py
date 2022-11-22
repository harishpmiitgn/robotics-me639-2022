import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import modules.Robot as Robot

robot = Robot.Robot([
	Robot.RevoluteJoint(1, 0, 0),
	Robot.RevoluteJoint(1, 0, 0),
	Robot.RevoluteJoint(1, 0, 0)
])

omega = 2 * np.pi / 10

def getDesiredPosition(t:float) -> np.ndarray :
	
	return 1.5 * np.array([
		np.cos(omega * t),
		np.sin(omega * t),
		0
	])

def getDesiredVelocity(t:float) -> np.ndarray :
	
	return 1.5 * omega * np.array([
		-np.sin(omega * t),
		np.cos(omega * t),
		0,
		0,
		0,
		0
	])

# Velocity Controller Time Step (Inverse of Controller Freq.)
time_step = 0.01

# Time in s
t = 0

# initial state
theta = np.arccos(0.25)
q_i = np.array([0, theta, -2*theta])
# q_i	= - np.pi + 2 * np.random.random(3) * np.pi

# Robot State
q		= q_i
q_dot	= np.zeros(3)

# Matplotlib objects
figure1 = plt.figure(figsize=[6,6])

axes_xy = figure1.add_subplot(1, 1, 1)

# Links
link_xy, = axes_xy.plot([], [], 'o-')

# End-Effector
end_effector_xy, = axes_xy.plot([], [], 'o')

# Desired Position
desired_position, = axes_xy.plot([], [], 'o')

axes_xy.set_xlim([-3, 3])
axes_xy.set_ylim([-3, 3])


# Draw Robot
def drawRobot() :

	T0 = robot.getHomogenousTransform(q, 1)
	T1 = robot.getHomogenousTransform(q, 2)
	T2 = robot.getHomogenousTransform(q, 3)
	
	x = np.column_stack((
		np.zeros(3),
		np.matmul(T0, [0, 0, 0, 1])[:3],
		np.matmul(T1, [0, 0, 0, 1])[:3],
		np.matmul(T2, [0, 0, 0, 1])[:3]
	))

	end_effector_xy.set_data(x[0, -1], x[1, -1])

	link_xy.set_data(x[0], x[1])

	pass

# Animation functions
def init_anim() :
	
	global t, q, q_dot

	t = 0
	q		= q_i
	q_dot	= np.zeros(3)

	drawRobot()

	x, y, z = getDesiredPosition(t)
	desired_position.set_data(x, y)

	return link_xy, end_effector_xy, desired_position,

def getBounded(vel, q_dot) :

	vel = np.where(np.abs(vel) > 10, q_dot, vel)

	return vel

def update_anim(i) :
	
	global t, q, q_dot

	error = getDesiredPosition(t) - robot.getEndEffectorPosition(q)

	J = robot.getJacobian(q)
	
	if np.linalg.det(np.matmul(J.T, J)) != 0 :
	
		q_dot = getBounded(robot.getJointVelocity(q, getDesiredVelocity(t)) + 2 * error, q_dot)

	q += (q_dot + np.random.normal(0, 0.01, 3)) * time_step

	t += time_step

	drawRobot()

	x, y, z = getDesiredPosition(t)
	desired_position.set_data(x, y)

	return link_xy, end_effector_xy, desired_position,

circle = getDesiredPosition(np.linspace(0, 1000 * time_step, 1000))
axes_xy.plot(circle[0], circle[1])

animation = FuncAnimation(figure1, update_anim, 1000, init_anim, interval=10)

plt.show()