import numpy as np
import modules.SCARARobot as Robot

from scipy.integrate import solve_ivp

controller_frequency	= 10.	# Hz
time_step				= 1. / controller_frequency # s

k_p = np.array([1, 1, 1])
k_i = np.array([0.0001, 0.0001, 0.0001])
k_d = np.array([0.1, 0.1, 0.1])

error				= np.zeros_like(Robot.q)
error_previous		= np.zeros_like(Robot.q)
integral_error		= np.zeros_like(Robot.q)
derivative_error	= np.zeros_like(Robot.q_dot)

def updateError(q_d:np.ndarray, q_d_dot:np.ndarray, q:np.ndarray, q_dot:np.ndarray) :

	global error, error_previous, integral_error, derivative_error

	error_previous		= error
	error				= q_d - q
	integral_error		= 0.5 * (error + error_previous) * time_step
	derivative_error	= q_d_dot - q_dot

	pass

def getEffort(error : np.ndarray, integral_error:np.ndarray, derivative_error : np.ndarray) -> np.ndarray :

	effort  = k_p * error
	effort += k_i * integral_error
	effort += k_d * derivative_error

	np.copyto(
		effort,
		np.sign(effort) * 0.1,
		where= np.abs(effort) > 0.1
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

	updateError(q_d, q_d_dot, q, q_dot)
	tau = getEffort(error, integral_error, derivative_error)

	state = getState(q, q_dot)
	solution = solve_ivp(getStateDerivative, [0, time_step], state, args=(tau,))

	Robot.q		= solution.y[:3, -1]
	Robot.q_dot	= solution.y[3:, -1]

	return t + solution.t[-1]