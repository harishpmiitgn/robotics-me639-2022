import numpy as np
import math
import matplotlib.pyplot as plt
from sympy import *

py=10
px=10
pz=10
d1=5
a2=2
theta1=math.pi+math.atan(py/px)
r=(px**2+py**2)**(1/2)
s=pz-d1
theta2=math.pi-math.atan(s/r)
d3=(r**2+s**2)**(1/2)-a2
print(d3)
print(theta1)
print(theta2)
