import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from sympy import N

from modules import System
from modules import Controller

# Trajectory
T = 100	# s
N = 1001
t = np.linspace(0, T, N)

alpha = 1.5 # Parameter for the trajectory

# Sinusoid
desired_q		= np.sin(2 * np.pi * alpha * t / T)
desired_q_dot	= (2 * np.pi * alpha / T) * np.cos(2 * np.pi * alpha * t / T)
desired_q_ddot	=-((2 * np.pi * alpha / T)**2) * np.sin(2 * np.pi * alpha * t / T)

plt.plot(t, desired_q, label=r'$q_d$')

# Disturbances
t_impulse = np.random.random(1) * T
disturb_tau = np.random.normal(0, 0.1, N) + (1 - np.tanh((t - t_impulse))**2)

# Simulation
q		= np.zeros(N)
q_dot	= np.zeros(N)

q[0]		= desired_q[0]
q_dot[0]	= desired_q_dot[0]

for i in np.arange(1, N) :

	state			= [q[i-1], q_dot[i-1]]
	desired_state	= [desired_q[i], desired_q_dot[i], desired_q_ddot[i]]

	tau  = disturb_tau[i]
	tau += Controller.openLoopControlSystem1(desired_state)
	tau += Controller.PDControlSystem1(desired_state, state)

	solution = solve_ivp(System.system1StateDerivative, [t[i-1], t[i]], state, args=(tau,))

	q[i]		= solution.y[0, -1]
	q_dot[i]	= solution.y[1, -1]

plt.plot(t, q, label=r'$q$')

plt.xlabel('Time, $t$ in seconds')
plt.ylabel('Value of State Variable, $q$')

plt.title('Sinusoid Response of PD + Feed-forward Control\nImpulse disturbance at t='+str(round(t_impulse[0],1))+' s')

plt.legend()
plt.show()