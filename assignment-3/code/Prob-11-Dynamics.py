import sympy

t, g = sympy.symbols('t, g')

def getEquationsOfMotion(D, V, q, tau) :

	n = len(q)
	equations = []

	q_dot  = [sympy.Derivative(q[i], t) for i in range(n)]
	q_ddot = [sympy.Derivative(q_dot[i], t) for i in range(n)]

	for k in range(n) :

		expr = - tau[k]

		for j in range(n) :

			expr += D[k][j] * q_ddot[j]

		for i in range(n) :

			for j in range(n) :

				expr += 0.5 * (sympy.Derivative(D[k][j], q[i]) + sympy.Derivative(D[k][i], q[j]) - sympy.Derivative(D[i][j], q[k]))

		expr += sympy.Derivative(V, q[k])

		equations.append(expr)

	return equations

if __name__ == '__main__' :

	q1, q2 = sympy.symbols('q_1, q_2')
	q = [q1, q2]

	m1, m2 = sympy.symbols('m_1, m_2')
	I1, I2 = sympy.symbols('I_1, I_2')
	l1, l2 = sympy.symbols('l_1, l_2')

	config = m2 * l1 * (l2 / 2) * sympy.cos(q2 - q1)

	D = [
		[0.25 * m1 * (l1**2) + m2 * (l1**2) + I1,	config],
		[config,	0.25 * m2 * (l2**2) + I2]
	]

	V = m1 * g * (l1/2) * sympy.sin(q1) + m2 * g * ( l1 * sympy.sin(q1) + (l2/2) * sympy.sin(q2))

	eqns = getEquationsOfMotion(D, V, q, sympy.symbols('tau_1, tau_2'))

	print(eqns)



		