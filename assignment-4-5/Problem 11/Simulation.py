import numpy as np
import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation

import modules.PUMARobot as Robot
import modules.Controller as Controller

# Time in s
t = 0

# initial state
q_i		= Robot.getRandomJointState()
q_i_dot	= np.zeros(3)

# Desired state
q_d		= q_i + 0.1
q_d_dot	= np.zeros(3)

desired_end_effector_position = Robot.getEndEffectorPosition(q_d)

# Matplotlib objects
figure1 = plt.figure(figsize=[4, 9])

axes_xy = figure1.add_subplot(2, 1, 1)
axes_xz = figure1.add_subplot(2, 1, 2)

# Links
link_xy, = axes_xy.plot([], [], 'o-')
link_xz, = axes_xz.plot([], [], 'o-')

# End-Effector
end_effector_xy, = axes_xy.plot([], [], 'o')
end_effector_xz, = axes_xz.plot([], [], 'o')

# Desired Position
axes_xy.plot(desired_end_effector_position[0], desired_end_effector_position[1], 'o')
axes_xz.plot(desired_end_effector_position[0], desired_end_effector_position[2], 'o')

axes_xy.set_xlim([-1, 1])
axes_xy.set_ylim([-1, 1])
axes_xz.set_xlim([-1, 1])
axes_xz.set_ylim([-1, 1])

# Draw Robot
def drawRobot() :

	x1 = Robot.getLinkEnds(Robot.q)
	x2 = Robot.getEndEffectorPosition(Robot.q)

	end_effector_xy.set_data(x2[0], x2[1])
	end_effector_xz.set_data(x2[0], x2[2])

	link_xy.set_data(x1[0], x1[1])
	link_xz.set_data(x1[0], x1[2])

	pass

# Animation functions
def init_anim() :

	Robot.q		= q_i
	Robot.q_dot	= q_i_dot

	Controller.error			= np.zeros_like(Robot.q)
	Controller.error_previous	= np.zeros_like(Robot.q)
	Controller.integral_error	= np.zeros_like(Robot.q)
	Controller.derivative_error	= np.zeros_like(Robot.q_dot)

	drawRobot()

	return link_xy, link_xz, end_effector_xz, end_effector_xy,

def update_anim(i) :
	
	global t

	t = Controller.proceedTimeStep(
		t,
		Robot.q,
		Robot.q_dot,
		q_d,
		q_d_dot
	)

	drawRobot()
	
	return link_xy, link_xz, end_effector_xz, end_effector_xy,

animation = FuncAnimation(figure1, update_anim, 1000, init_anim)

plt.show()