from math import sqrt, atan, sin, cos, pi
p_x, p_y, p_z = 1.23, 2.65, 0.45
d1, a2, a3 = 1, 2, 1
D = (p_x**2 + p_y**2 + (p_z - d1)**2 - a2**2 - a3**2)/(2*a2*a3)

q1 = atan(p_y/p_x)
q3 = atan(sqrt(1 - D**2)/D)
q2 = atan(p_z - d1/sqrt(p_x**2 + p_y**2)) - atan(a3*sin(q3)/ a2 + a3*cos(q3))
# print(q1)
# print(q2)
# print(q3)

r11, r12, r21, r22, r13, r23, r33 = 0.404, 0, 0, 1, 0.404, 0, 0.404

theta4 = atan((-cos(q1)*sin(q2+q3)*r13 - sin(q1)*sin(q2+q3)*r23 - cos(q2+q3)*r33)/(cos(q1)*cos(q2+q3)*r13 + sin(q1)*cos(q2+q3)*r23 - sin(q2+q3)*r23))
theta5 = atan(sqrt(1 - (sin(q1)*r13 - cos(q1)*r23)**2)/(sin(q1)*r13 - cos(q1)*r23))
theta6 = atan((sin(q1)*r12 + cos(q1)*r22)/(sin(q1)*r11 - cos(q1)*r21))
# print(theta4)
# print(theta5)
# print(theta6)

print("Angle4="+str(theta4*180/pi)+" Angle5="+str(theta5*180/pi)+" Angle6="+str(theta6*180/pi))