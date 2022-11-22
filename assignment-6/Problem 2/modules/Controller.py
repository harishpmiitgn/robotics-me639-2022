import numpy as np
import modules.SCARARobot as Robot

from scipy.integrate import solve_ivp

controller_frequency	= 10.	# Hz
time_step				= 1. / controller_frequency # s

K_0 = np.array([0.1, 0.1, 0.1])

def getEffort(q, q_dot, q_desired, q_dot_desired) -> np.ndarray :

	effort =  np.matmul(Robot.C(q, q_dot), q_dot)
	effort += np.matmul(Robot.D(q), (q_dot_desired - q_dot) / time_step)
	effort += K_0 * (q_desired - q)

	np.copyto(
		effort,
		np.sign(effort) * 0.5,
		where= np.abs(effort) > 0.5
	)

	return effort

def getState(q:np.ndarray, q_dot:np.ndarray) -> np.ndarray :

	return np.append(q, q_dot)

def getStateDerivative(t:float, state:np.ndarray, tau:np.ndarray) -> np.ndarray :

	q		= state[:3]
	q_dot	= state[3:]

	return np.append(
		q_dot,
		Robot.getQDDot(q, q_dot, tau)
	)

def proceedTimeStep(t, q:np.ndarray, q_dot:np.ndarray, q_d:np.ndarray, q_d_dot:np.ndarray) -> float:

	tau = getEffort(q, q_dot, q_d, q_d_dot)
	tau += np.random.normal(0, 0.01, 3)

	state = getState(q, q_dot)
	solution = solve_ivp(getStateDerivative, [0, time_step], state, args=(tau,))

	Robot.q		= solution.y[:3, -1]
	Robot.q_dot	= solution.y[3:, -1]

	return t + solution.t[-1]