import numpy as np

def getCubicTrajectory(start, stop, t_start, t_stop) :

	A = np.array([
		[t_start**3,		t_start**2,		t_start,	1],
		[3 * t_start**2,	2 * t_start,	1,			0],
		[t_stop**3,			t_stop**2,		t_stop,		1],
		[3 * t_stop**2,		2 * t_stop,		1,			0]
	])
	
	B = np.vstack((start, np.zeros(3), stop, np.zeros(3)))
	
	return np.linalg.solve(A, B)