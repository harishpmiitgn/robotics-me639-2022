import numpy as np
from modules import System

def openLoopControlSystem1(state_desired) :

	return (	
		System.m * state_desired[2] +
		System.c * state_desired[1] +
		System.b * state_desired[0]
	)


def openLoopControlSystem2(state_desired) :

	return (
		System.m * state_desired[2] +
		System.c * state_desired[1] +
		System.b * state_desired[0] +
		np.sin(state_desired[0]) +
		state_desired[0]**2
	)

def PDControlSystem1(state_desired, state) :

	return (
		10.0 * (state_desired[0] - state[0]) +
		10.0 * (state_desired[1] - state[1])
	)

def PDControlSystem2(state_desired, state) :

	return (
		50.0 * (state_desired[0] - state[0]) +
		10.0 * (state_desired[1] - state[1])
	)

if __name__=='__main__' :

	import matplotlib.pyplot as plt
	from scipy.integrate import solve_ivp

	t      =   np.linspace(0, 10, 101)
	q      =   np.sin(2 * np.pi * t / 10)
	q_dot  =   0.2 * np.pi * np.cos(2 * np.pi * t / 10)
	q_ddot = - 0.04 * (np.pi**2) * np.sin(2 * np.pi * t / 10)

	plt.plot(t, q)

	q_real = [q[0]]
	q_dot_real = [q_dot[0]]

	for i in np.arange(1, 101) :

		tau = PDControlSystem2([q[i], q_dot[i], q_ddot[i]], [q_real[-1], q_dot_real[-1]])
		solution = solve_ivp(System.system2StateDerivative, [t[i-1], t[i]], [q_real[-1], q_dot_real[-1]], args=(tau,))

		q_real.append(solution.y[0, -1])
		q_dot_real.append(solution.y[1, -1])

	plt.plot(t, q_real)
	plt.show()
