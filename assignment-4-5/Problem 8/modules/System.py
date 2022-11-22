import numpy as np

m = 2
c = 1
b = 0.1

def system1StateDerivative(t:float, state:np.ndarray, tau:float) -> np.ndarray :

	return np.array([
		state[1],
		(tau - c*state[1] - b*state[0]) / m
	])

def system2StateDerivative(t:float, state:np.ndarray, tau:float) -> np.ndarray :

	return np.array([
		state[1],
		(tau - c*state[1] - b*state[0] - np.sin(state[0]) - state[0]**2) / m
	])

if __name__=='__main__' :

	import matplotlib.pyplot as plt
	from scipy.integrate import solve_ivp

	solution = solve_ivp(system1StateDerivative, [0, 10], [0, 0], t_eval=np.linspace(0, 10, 1000), args=(1,))

	plt.plot(solution.t, solution.y[0])
	plt.show()
