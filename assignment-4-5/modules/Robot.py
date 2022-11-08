import numpy as np

class Joint :

	def __init__(self) :
		
		self.a = 0.0
		self.alpha = 0.0

		pass

	def _getRotationXAlpha(self) :

		c = np.cos(self.alpha)
		s = np.sin(self.alpha)

		return np.array([
			[1, 0, 0, 0],
			[0, c,-s, 0],
			[0, s, c, 0],
			[0, 0, 0, 1]
		])

	def _getTranslationXA(self) :

		return np.array([
			[1, 0, 0, self.a],
			[0, 1, 0, 0],
			[0, 0, 1, 0],
			[0, 0, 0, 1]
		])

	def _getRotationZTheta(self, theta:float) :
		
		c = np.cos(theta)
		s = np.sin(theta)
		
		return np.array([
			[c,-s, 0, 0],
			[s, c, 0, 0],
			[0, 0, 1, 0],
			[0, 0, 0, 1]
		])

	def _getTranslationZD(self, d:float) :

		return np.array([
			[1, 0, 0, 0],
			[0, 1, 0, 0],
			[0, 0, 1, d],
			[0, 0, 0, 1]
		])

	def getT(self, theta:float, d:float) :

		return np.matmul(
			np.matmul(self._getRotationZTheta(theta), self._getTranslationZD(d)),
			np.matmul(self._getTranslationXA(), self._getRotationXAlpha())
		)
	
class RevoluteJoint(Joint) :

	def __init__(self, a:float, d:float, alpha:float):

		super(RevoluteJoint, self).__init__()

		self.d = d
		self.a = a

		self.alpha = alpha
		
		return

	def getTransform(self, q:float) :

		return self.getT(q, self.d)

class PrismaticJoint(Joint) :

	def __init__(self, a:float, alpha:float, theta:float):
		
		super(PrismaticJoint, self).__init__()

		self.a = a
		
		self.alpha = alpha
		self.theta = theta

		pass

	def getTransform(self, q:float) :

		return self.getT(self.theta, q)

class Robot :

	def __init__(self, joint_list:list) :
		
		self.joint_list = joint_list

		pass

	def _getUnitZ(self, i:int, q:np.ndarray) :

		R = np.identity(3)

		for j in range(i) :

			R = np.matmul(R, self.joint_list[j].getTransform(q[j])[:3, :3])

		return np.matmul(R, np.array([0, 0, 1]))

	def _getOrigin(self, i:int, q:np.ndarray) :

		T = np.identity(4)

		for j in range(i) :

			T = np.matmul(T, self.joint_list[j].getTransform(q[j]))
			# print(T)
			# print(self.joint_list[j].getTransform(q[j]))

		return np.matmul(T, np.array([0, 0, 0, 1]))[:3]

	def _getJacobianColumn(self, i:int, q:np.ndarray, o_n:np.ndarray) :

		z = self._getUnitZ(i, q)

		if isinstance(self.joint_list[i], RevoluteJoint) :

			return np.append(
				np.cross(z, (o_n - self._getOrigin(i, q))),
				z
			)

		elif isinstance(self.joint_list[i], PrismaticJoint) :

			return np.append(
				z,
				np.zeros(3)
			)

		else :

			return np.zeros(6)

	def getJacobian(self, q:np.ndarray) :

		n = len(self.joint_list)

		if n != q.shape[-1] : raise ValueError('Dimension mismatch')

		o_n = self._getOrigin(n, q)

		J = np.array(self._getJacobianColumn(0, q, o_n)).reshape((6, 1))

		for i in range(1, n) :

			J = np.append(J, self._getJacobianColumn(i, q, o_n).reshape(6,1), axis=1)

		return J

	def getEndEffectorPosition(self, q:np.ndarray) :

		n = len(self.joint_list)

		if n != q.shape[-1] : raise ValueError('Dimension mismatch')

		return self._getOrigin(n, q)

	def getEndEffectorVelocity(self, q:np.ndarray, q_dot:np.ndarray) :

		n = len(self.joint_list)

		if n != q_dot.shape[-1] : raise ValueError('Dimension mismatch')

		return np.matmul(self.getJacobian(q), q_dot)

	def getJointVelocity(self, q:np.ndarray, vel:np.ndarray) :

		n = len(self.joint_list)

		if n != vel.shape[-1] : raise ValueError('Dimension mismatch')

		J = self.getJacobian(q)

		return np.matmul(
			np.linalg.inv(np.matmul(J.T, J)),
			np.matmul(J.T, vel)
		)

	def getHomogenousTransform(self, q:np.ndarray) :

		n = len(self.joint_list)

		if n != q.shape[-1] : raise ValueError('Dimension mismatch')

		T = np.identity(4)

		for j in range(n) :

			T = np.matmul(T, self.joint_list[j].getTransform(q[j]))

		return T


