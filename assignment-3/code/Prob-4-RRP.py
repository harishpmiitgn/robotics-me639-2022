import numpy as np
from Robot import Robot, RevoluteJoint, PrismaticJoint

print('SCARA')

SCARA = Robot([
	RevoluteJoint(0.5, 0.0, 0.0),
	RevoluteJoint(0.5, 0.0, np.pi),
	PrismaticJoint(0.0, 0.0, 0.0)
])

state = np.array([0, np.pi/2, 0.25])

print('End Effector Position : ', np.round(SCARA.getEndEffectorPosition(state), 2))
print('Jacobian : ')
print(np.round(SCARA.getJacobian(state), 3))

state = np.array([np.pi/2, 0, 0.25])

print('End Effector Position : ', np.round(SCARA.getEndEffectorPosition(state), 2))
print('Jacobian : ')
print(np.round(SCARA.getJacobian(state), 3))

state = np.array([np.pi/2, np.pi/2, 0.25])

print('End Effector Position : ', np.round(SCARA.getEndEffectorPosition(state), 2))
print('Jacobian : ')
print(np.round(SCARA.getJacobian(state), 3))

print('\n\nStanford')

Stanford = Robot([
	RevoluteJoint(0, 0.5, -np.pi/2),
	RevoluteJoint(0, 0, np.pi/2),
	PrismaticJoint(0, 0, 0)
])

state = np.array([0, np.pi/2, 0.25])

print('End Effector Position : ', np.round(Stanford.getEndEffectorPosition(state), 2))
print('Jacobian : ')
print(np.round(SCARA.getJacobian(state), 3))

state = np.array([np.pi/2, 0, 0.25])

print('End Effector Position : ', np.round(Stanford.getEndEffectorPosition(state), 2))
print('Jacobian : ')
print(np.round(SCARA.getJacobian(state), 3))

state = np.array([np.pi/2, np.pi/2, 0.25])

print('End Effector Position : ', np.round(Stanford.getEndEffectorPosition(state), 2))
print('Jacobian : ')
print(np.round(SCARA.getJacobian(state), 3))