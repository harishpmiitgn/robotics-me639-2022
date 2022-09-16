import numpy as np
from Robot import Robot, RevoluteJoint, PrismaticJoint

print('3D Printer')

printer3D = Robot([
	PrismaticJoint(0, np.pi/2, np.pi/2),
	PrismaticJoint(0, np.pi/2,-np.pi/2),
	PrismaticJoint(0.0, 0.0, 0.0)
])

state = np.array([0, 0.5, 0.25])

print('End Effector Position : ', np.round(printer3D.getEndEffectorPosition(state), 2))
print('Jacobian : ')
print(np.round(printer3D.getJacobian(state), 3))

state = np.array([0.5, 0, 0.25])

print('End Effector Position : ', np.round(printer3D.getEndEffectorPosition(state), 2))
print('Jacobian : ')
print(np.round(printer3D.getJacobian(state), 3))

state = np.array([0.5, 0.5, 0.25])

print('End Effector Position : ', np.round(printer3D.getEndEffectorPosition(state), 2))
print('Jacobian : ')
print(np.round(printer3D.getJacobian(state), 3))
