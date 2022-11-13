import numpy as np
import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation

import modules.StanfordRobot as StanfordRobot
import modules.Controller as Controller

# Time in s
t = 0

# initial state
q_i		= StanfordRobot.getRandomJointState()
q_i_dot	= np.zeros(3)

# Desired state
q_d		= q_i + 0.1
q_d_dot	= np.zeros(3)

desired_end_effector_position = StanfordRobot.getEndEffectorPosition(q_d)

# Matplotlib objects
figure1 = plt.figure(figsize=[4, 9])

axes_xy = figure1.add_subplot(2, 1, 1)
axes_xz = figure1.add_subplot(2, 1, 2)

# z-Support link
axes_xy.plot([0, 0], 	[0, 0], 'o-')
axes_xz.plot([0, 0], [-0.4, 0], 'o-')

# Link # 3 
link_xy, = axes_xy.plot([], [])
link_xz, = axes_xz.plot([], [])

# End-Effector
end_effector_xy, = axes_xy.plot([], [], 'o')
end_effector_xz, = axes_xz.plot([], [], 'o')

# Desired Position
axes_xy.plot(desired_end_effector_position[0], desired_end_effector_position[1], 'o')
axes_xz.plot(desired_end_effector_position[0], desired_end_effector_position[2], 'o')

axes_xy.set_xlim([-0.5, 0.5])
axes_xy.set_ylim([-0.5, 0.5])
axes_xz.set_xlim([-0.5, 0.5])
axes_xz.set_ylim([-0.5, 0.5])

# Draw Robot
def drawRobot() :

	x2 = StanfordRobot.getEndEffectorPosition(StanfordRobot.q)
	x1 = StanfordRobot.getLinkOtherEnd(x2)

	link_xy.set_data([x1[0], x2[0]], [x1[1], x2[1]])
	link_xz.set_data([x1[0], x2[0]], [x1[2], x2[2]])

	end_effector_xy.set_data(x2[0], x2[1])
	end_effector_xz.set_data(x2[0], x2[2])

	pass

# Animation functions
def init_anim() :

	StanfordRobot.q		= q_i
	StanfordRobot.q_dot	= q_i_dot

	Controller.error			= np.zeros_like(StanfordRobot.q)
	Controller.error_previous	= np.zeros_like(StanfordRobot.q)
	Controller.integral_error	= np.zeros_like(StanfordRobot.q)
	Controller.derivative_error	= np.zeros_like(StanfordRobot.q_dot)

	drawRobot()

	return link_xy, link_xz, end_effector_xz, end_effector_xy

def update_anim(i) :
	
	global t

	t = Controller.proceedTimeStep(
		t,
		StanfordRobot.q,
		StanfordRobot.q_dot,
		q_d,
		q_d_dot
	)

	print(q_d, StanfordRobot.q)

	drawRobot()
	
	return link_xy, link_xz, end_effector_xz, end_effector_xy

animation = FuncAnimation(figure1, update_anim, 1000, init_anim)

plt.show()