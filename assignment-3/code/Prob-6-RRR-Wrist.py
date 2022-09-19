import numpy as np
from Robot import Robot, RevoluteJoint, PrismaticJoint

print('RRR with Wrist')

l1 = 0.5
l2 = 0.5
l3 = 0.5

RRR_wrist = Robot([
	RevoluteJoint(0, l1,-np.pi/2),
	RevoluteJoint(l2, 0, 0),
	RevoluteJoint(l3, 0, 0),
	RevoluteJoint(0, 0, np.pi/2),
	RevoluteJoint(0, 0, np.pi/2),
	RevoluteJoint(0, 0, 0)
])

state = np.array([0, 0, 0, 0, np.pi/2, 0])

print('End Effector Position : ', np.round(RRR_wrist.getEndEffectorPosition(state), 2))
print('Jacobian : ')
print(np.round(RRR_wrist.getJacobian(state), 3))