# Controller code is situated in modules
# The implementation is slightly different from
# textbook multivariable control.
# The controller applies torque to accelerate
# the joints from current velocity
# to that required at time t + time_step
# (where time_step is inverse of controller frequency),
# with some added proportional control on position error

import numpy as np
import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation

import modules.SCARARobot as Robot
import modules.Controller as Controller

from Subpart_b import getDesiredJointState, getDesiredJointVelocity, drawSquare

# Time in s
t = 0

# initial state
q_i		= getDesiredJointState(0)
q_i_dot	= getDesiredJointVelocity(0)

# Desired state
q_d		= getDesiredJointState(0)
q_d_dot	= getDesiredJointVelocity(0 + Controller.time_step)

# Matplotlib objects
figure1 = plt.figure(figsize=[4, 9])

axes_xy = figure1.add_subplot(2, 1, 1)
axes_xz = figure1.add_subplot(2, 1, 2)

# Link 1 and 2
link12_xy, = axes_xy.plot([], [], 'o-')
link12_xz, = axes_xz.plot([], [], 'o-')

# Link # 3 
link3_xy, = axes_xy.plot([], [])
link3_xz, = axes_xz.plot([], [])

# End-Effector
end_effector_xy, = axes_xy.plot([], [], 'o')
end_effector_xz, = axes_xz.plot([], [], 'o')

drawSquare(axes_xy)

axes_xy.set_xlim([-0.5, 0.5])
axes_xy.set_ylim([-0.5, 0.5])
axes_xz.set_xlim([-0.5, 0.5])
axes_xz.set_ylim([-0.5, 0.5])

# Draw Robot
def drawRobot() :

	x2 = Robot.getEndEffectorPosition(Robot.q)
	x1 = Robot.getLink3OtherEnd(x2)

	link3_xy.set_data([x1[0], x2[0]], [x1[1], x2[1]])
	link3_xz.set_data([x1[0], x2[0]], [x1[2], x2[2]])

	end_effector_xy.set_data(x2[0], x2[1])
	end_effector_xz.set_data(x2[0], x2[2])

	x3 = Robot.getLink12Ends(Robot.q)

	link12_xy.set_data(x3[0], x3[1])
	link12_xz.set_data(x3[0], x3[2])

	pass

# Animation functions
def init_anim() :

	Robot.q		= q_i
	Robot.q_dot	= q_i_dot

	drawRobot()

	return link3_xy, link3_xz, end_effector_xz, end_effector_xy, link12_xz, link12_xy

def update_anim(i) :
	
	global t

	q_d		= getDesiredJointState(t)
	q_d_dot = getDesiredJointVelocity(t + Controller.time_step)

	t = Controller.proceedTimeStep(
		t,
		Robot.q,
		Robot.q_dot,
		q_d,
		q_d_dot
	)

	drawRobot()
	
	return link3_xy, link3_xz, end_effector_xz, end_effector_xy, link12_xz, link12_xy

animation = FuncAnimation(figure1, update_anim, 1000, init_anim, interval=10)

plt.show()