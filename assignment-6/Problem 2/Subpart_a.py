import numpy as np
import matplotlib.pyplot as plt

import modules.SCARARobot as SCARA

point_A = np.array([0.45, 0.075, 0.1])
point_B = np.array([0.45,-0.075, 0.1])
point_C = np.array([0.25,-0.075, 0.1])
point_D = np.array([0.25, 0.075, 0.1])

def verify_point_in_workspace(point) :

	joint_state = SCARA.inverse_kinematics(point)

	print('Workspace Coordinate (m, m, m):\t', point)
	print('Joint State (rad, rad, m):\t', np.round(joint_state, 2))
	print('Forward Kinematics (m, m, ):\t', SCARA.getEndEffectorPosition(joint_state))

# Point A
print('Point A')
verify_point_in_workspace(point_A)

# Point B
print('\nPoint B')
verify_point_in_workspace(point_B)

# Point C
print('\nPoint C')
verify_point_in_workspace(point_C)

# Point D
print('\nPoint D')
verify_point_in_workspace(point_D)


# Output - Yes
# Point A
# Workspace Coordinate (m, m, m):  [0.45  0.075 0.1  ]
# Joint State (rad, rad, m):       [ 0.58679993 -0.84330251 -0.1       ]
# Forward Kinematics (m, m, ):     [0.45  0.075 0.1  ]

# Point B
# Workspace Coordinate (m, m, m):  [ 0.45  -0.075  0.1  ]
# Joint State (rad, rad, m):       [ 0.25650258 -0.84330251 -0.1       ]
# Forward Kinematics (m, m, ):     [ 0.45  -0.075  0.1  ]

# Point C
# Workspace Coordinate (m, m, m):  [ 0.25  -0.075  0.1  ]
# Joint State (rad, rad, m):       [ 0.73012748 -2.04316854 -0.1       ]
# Forward Kinematics (m, m, ):     [ 0.25  -0.075  0.1  ]

# Point D
# Workspace Coordinate (m, m, m):  [0.25  0.075 0.1  ]
# Joint State (rad, rad, m):       [ 1.31304106 -2.04316854 -0.1       ]
# Forward Kinematics (m, m, ):     [0.25  0.075 0.1  ]