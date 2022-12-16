from turtle import color
import numpy as np
from numpy import pi, sin, cos, sqrt, absolute, arccos, arctan, sign
import matplotlib.pyplot as plt
import matplotlib.animation as animation
l1 = 10
l2 = 10
half_wall =2
i0,j0 =2,3

x2_arr = np.zeros(200)  #Point 2
y2_arr = np.zeros(200)
x1_arr = np.zeros(200)  #Point 2
y1_arr = np.zeros(200)
llim,hlim = 35,146

def workspace(llim,hlim):
    px,py,p1,p2 = [],[],[],[]
    for o1 in range(llim,hlim,5):
        for o2 in range(llim,hlim,5):
            p1.append(l1*cos(o1*3.14/180))
            p2.append(l1*sin(o1*3.14/180))
            xi = l1*cos(o1*3.14/180)+l2*cos((o1+o2+270)%360*3.14/180)
            yi = l1*sin(o1*3.14/180)+l2*sin((o1+o2+270)%360*3.14/180)
            px.append(xi)
            py.append(yi)
    return px,py,p1,p2


fig = plt.figure()
ax = fig.add_subplot(
    111, aspect="equal", autoscale_on=False, xlim=(-20, 20), ylim=(-20, 20)
)

# add grid lines, title and take out the axis tick labels
ax.grid(alpha=1)
ax.set_title("Elbow Manipulator")
ax.set_xticklabels([])
ax.set_yticklabels([])
(line, ) = ax.plot(
    [], [], "o-", lw=7, color="#2b8cbe"
) 

px,py,p1,p2 = workspace(llim,hlim)
(line3,) = ax.plot(
    [], [], ".", lw=2, color='black'
) 

#workspace
def init():
    line.set_data([], [])
    return (line,)
def animate(i):
    x_points = [0, p1[i], px[i]]
    y_points = [0, p2[i], py[i]]
    line.set_data(x_points, y_points)
    line3.set_data(px[0:i],py[0:i])
    return (line,line3)#line2
ani = animation.FuncAnimation(
    fig, animate, init_func=init, frames=len(p1), interval=40, blit=True, repeat=False
)
## to save animation, uncomment the line below. Ensure ffmpeg is installed:
ani.save('Workspace_2R.mp4', fps=30, extra_args=['-vcodec', 'libx264'])
# show the animation
plt.show()
