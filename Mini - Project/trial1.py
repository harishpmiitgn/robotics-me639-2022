from cProfile import label
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import cv2
import scipy
import sympy as sp
import os
import moviepy.video.io.ImageSequenceClip

# Task - 1: End Effoctor following a given trajectory
l1 = 15
l2 = 10
L = l1 + l2

t = 0

while t < 360:
	x = 10*math.cos((t*math.pi)/(180))
	y = 10*math.sin((t*math.pi)/(180))
	filename = str(t) + '.png'
	t += 1
	
	if ((x**2)+(y**2)) <= (L**2):
		theta = math.acos(((x**2) + (y**2) - (l1**2) - (l2**2))/(2*l1*l2))
		q1 = math.atan(y/x) - math.atan((l2*math.sin(theta))/(l1 + l2*math.cos(theta)))
		if 90 < t < 180 and q1 < 0:
			q1 = math.pi + q1
		q2 = q1 + theta
	
		x1 , y1 = [0, l1*math.cos(q1)], [0, l1*math.sin(q1)]
		x2 , y2 = [l1*math.cos(q1), l1*math.cos(q1)+l2*math.cos(q2)], [l1*math.sin(q1), l1*math.sin(q1)+l2*math.sin(q2)]
		plt.plot(x1, y1, x2, y2)
		plt.savefig(filename)
		# plt.clf()
		# plt.show()
	

image_folder='images'
fps=15

image_files = [os.path.join(image_folder,img)
               for img in os.listdir(image_folder)
               if img.endswith(".png")]
clip = moviepy.video.io.ImageSequenceClip.ImageSequenceClip(image_files, fps=fps)
clip.write_videofile('trialsim2.mp4')

# Task - 2: End effoctor arranges itself to apply the required force on the wall

x0 = float(input('x-coordinate of the point of application: '))
y0 = float(input('y-coordinate of the point of application: '))

theta = math.acos(((x0**2) + (y0**2) - (l1**2) - (l2**2))/(2*l1*l2))
q1 = math.atan(y0/x0) - math.atan((l2*math.sin(theta))/(l1 + l2*math.cos(theta)))
q2 = q1 + theta

Fx = float(input('X-component of Force: '))
Fy = float(input('Y-component of Force: '))
Tau_1 = -1*(Fx)*l1*math.sin(q1) + (Fy)*l1*math.cos(q1)
Tau_2 = -1*(Fx)*l2*math.sin(q2) + (Fy)*l2*math.cos(q2)

x1 , y1 = [0, l1*math.cos(q1)], [0, l1*math.sin(q1)]
x2 , y2 = [l1*math.cos(q1), l1*math.cos(q1)+l2*math.cos(q2)], [l1*math.sin(q1), l1*math.sin(q1)+l2*math.sin(q2)]
plt.text(L, L+1, 'Tau_1 = ' + str(Tau_1))
plt.text(L, L, 'Tau_2 = ' + str(Tau_2))
plt.plot(x1, y1, x2, y2)
plt.show()


# Task - 3: End Effector's virtual spring effect

x0 = 10
y0 = 10
k = 50

Fx = k*(x - x0)
Fy = k*(y - y0)

