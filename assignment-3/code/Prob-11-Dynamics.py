import sympy

g = sympy.symbols('g')

def getEquationsOfMotion(D, V, q, q_dot, q_ddot, tau) :

	n = len(q)
	equations = []

	for k in range(n) :

		expr = - tau[k]

		for j in range(n) :

			expr += D[k][j] * q_ddot[j]

		for i in range(n) :

			for j in range(n) :

				expr += 0.5 * (sympy.Derivative(D[k][j], q[i]) + sympy.Derivative(D[k][i], q[j]) - sympy.Derivative(D[i][j], q[k])) * q_dot[i] * q_dot[j]

		expr += sympy.Derivative(V, q[k])

		equations.append(sympy.simplify(expr))

	return equations

if __name__ == '__main__' :

	q1, q2 = sympy.symbols('q_1, q_2')
	q1_dot, q2_dot = sympy.symbols('q\dot_1, q\dot_2')
	q1_ddot, q2_ddot = sympy.symbols('q\ddot_1, q\ddot_2')
	q = [q1, q2]
	q_dot = [q1_dot, q2_dot]
	q_ddot = [q1_ddot, q2_ddot]

	m1, m2 = sympy.symbols('m_1, m_2')
	I1, I2 = sympy.symbols('I_1, I_2')
	l1, l2 = sympy.symbols('l_1, l_2')

	config = m2 * l1 * (l2 / 2) * sympy.cos(q2 - q1)

	D = [
		[0.25 * m1 * (l1**2) + m2 * (l1**2) + I1,	config],
		[config,	0.25 * m2 * (l2**2) + I2]
	]

	V = m1 * g * (l1/2) * sympy.sin(q1) + m2 * g * ( l1 * sympy.sin(q1) + (l2/2) * sympy.sin(q2))

	eqns = getEquationsOfMotion(D, V, q, q_dot, q_ddot, sympy.symbols('tau_1, tau_2'))

	print(eqns)



		