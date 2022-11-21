from turtle import color
import numpy as np
from numpy import sin, cos, sqrt, absolute, arccos, arctan, sign
import matplotlib.pyplot as plt
import matplotlib.animation as animation
l1 = 20
l2 = 20
m1,m2 = 0.5,0.5
a1,a2 = 1,1

half_wall = 3
cent = input("Enter wall position as x,y :")
wx,wy = float(cent.split(",")[0]),float(cent.split(",")[1])
deg = float(input("Enter orientation in degrees :"))
deg = deg%360
scale_fact = 0.1
fn = float(input("Enter normal force :"))
f = fn*scale_fact
wx1,wy1 = wx + half_wall*cos(deg*3.14/180),wy - half_wall*sin(deg*3.14/180)
wx2,wy2 = wx - half_wall*cos(deg*3.14/180),wy + half_wall*sin(deg*3.14/180)
nrx,nry = wx,wy
if np.absolute(wx1)<np.absolute(wx2):
    nrx,nry = wx1,wy1
if np.absolute(wx1)>np.absolute(wx2):
    nrx,nry = wx2,wy2
x2,y2 = wx,wy
x2_arr = np.zeros(200)  #Point 2
y2_arr = np.zeros(200)
x1_arr = np.zeros(200)  #Point 2
y1_arr = np.zeros(200)
i0,j0 = 2,2

fx0,fy0 = wx,wy
if deg <= 180:
    degf = deg+ wx/np.absolute(wx) * 90
else:
    degf = deg- wx/np.absolute(wx) * 90
#For Torque analysis
# fx,fy = fn*cos(degf*3.14/180),fn*sin(degf*3.14/180)
# tau1,tau2 = -l1*sin(teta1)*fx+l1*cos(teta1)*fy,-l2*sin(teta2)*fx+l2*cos(teta1)*fy

fx1,fy1 = fx0 + f*cos(degf*3.14/180),fy0 - f*sin(degf*3.14/180)
fx_arr = np.linspace(fx0,fx1, 200)
fy_arr = np.linspace(fy0,fy1, 200)
# teta2 = -arccos((x2**2+y2**2-l1**2-l2**2)/(2*l1*l2))
# teta1 = arccos((x2*(l1+l2*cos(theta2))+y2*l2*sin(theta2))/(x2**2 +y2**2))
for i in range(0,200):
    if(nrx == wx):
        x2 = i0 + (wx-i0)*(i/200)
        y2 = j0 + (wy-j0)*(i/200)
    else:
        if i<100:
           x2 = i0 + (nrx-i0)*(i/100)
           y2 = j0 + (nry-j0)*(i/100) 
        else:
           x2 = nrx + (wx-nrx)*(i%100/100)
           y2 = nry + (wy-nry)*(i%100/100)  
    x2_arr[i] = x2
    y2_arr[i] = y2
    theta2 = -wx/np.absolute(wx) * arccos((x2**2+y2**2-l1**2-l2**2)/(2*l1*l2))
    theta1 = arccos((x2*(l1+l2*cos(theta2))+y2*l2*sin(theta2))/(x2**2 +y2**2))
    #print(theta1*180/3.14)
    x1 = l1*cos(theta1)
    y1 = l1*sin(theta1)
    x1_arr[i]= x1
    y1_arr[i] = y1

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
(line2, ) = ax.plot(
    np.linspace(wx1, wx2, 100), np.linspace(wy1,wy2,100), ".", lw=2, color='black'
) 
(line3, ) = ax.plot(
    [], [], ".-", lw=2, color='red'
) 
# initialization function
def init():
    line.set_data([], [])
    return (line,)

path_x,path_y = [],[]

# animation function
def animate(i):
    if i<len(x1_arr):
        x_points = [0, x1_arr[i], x2_arr[i]]
        y_points = [0, y1_arr[i], y2_arr[i]]
        # path_x = [ax_points[i//200],x2_arr[i]]
        # path_y = [by_points[i//200],y2_arr[i]]
        # path_x.append(fx_arr[i])
        # path_y.append(fy_arr[i])
        line.set_data(x_points, y_points)
        line2.set_data(np.linspace(wx1, wx2, 100), np.linspace(wy1,wy2,100))
    else:
        path_x = [fx_arr[0],fx_arr[i-len(x1_arr)]]
        path_y = [fy_arr[0],fy_arr[i-len(x1_arr)]]
        line3.set_data(path_x,path_y)

        x_points = [0, x1_arr[len(x1_arr)-1], x2_arr[len(x1_arr)-1]]
        y_points = [0, y1_arr[len(x1_arr)-1], y2_arr[len(x1_arr)-1]]
        line.set_data(x_points, y_points)
        line2.set_data(np.linspace(wx1, wx2, 100), np.linspace(wy1,wy2,100))
    # line3.set_data(path_x[i],path_y[i])
    
    return (line,line3,)#line2
ani = animation.FuncAnimation(
    fig, animate, init_func=init, frames=len(x1_arr)-1+200, interval=40, blit=True, repeat=False
)
plt.show()
ani.save('WallForce_2R.mp4', fps=30, extra_args=['-vcodec', 'libx264'])

# show the animation
plt.show()