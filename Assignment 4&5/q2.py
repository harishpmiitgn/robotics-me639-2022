from math import pi, sin, cos, sqrt, atan

p_x, p_y, p_z = 1.23, 2.65, 0.45

a1, a2= 2, 1

r = sqrt((p_x**2 + p_y**2 - a1**2 - a2**2)/(2*a1*a2))
# print(r)
theta2 = atan(r/sqrt(1 - r**2))
# print(theta2)
theta1 = atan(p_y/p_x) - atan((a2*sin(theta2))/(a1 + a2*cos(theta2)))
# print(theta1)
d3 = p_z

print("Angle1="+str(theta1*180/pi)+" Angle2="+str(theta2*180/pi)+" Linear="+str(d3))

