from math import pi, sqrt, atan 

a2 = 1

p_x, p_y, p_z = 1, 0.404, 0.404

d1, d2= 1, 1

r = sqrt((p_x)**2 + (p_y)**2)
s = p_z - d1
d3 = sqrt((r**2)+(s**2)) - a2

print("Angle1="+str((atan(p_y/p_x)*180)/pi)+" Angle2="+str((atan(s/r)*180)/pi)+" Linear="+str(d3))

