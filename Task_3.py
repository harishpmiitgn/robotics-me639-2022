from turtle import color
import numpy as np
from numpy import pi, sin, cos, sqrt, absolute, arccos, arctan, sign
import matplotlib.pyplot as plt
import matplotlib.animation as animation
l1 = 10
l2 = 10
half_wall =2
i0,j0 =2,3
cent = input("Enter position as x,y :")
wx,wy = float(cent.split(",")[0]),float(cent.split(",")[1])
r = sqrt(wx**2+wy**2)
slope =arctan(wy/wx)
wx1,wy1 = wx/np.absolute(wx)*r*cos(slope-3.14/18),wx/np.absolute(wx)*r*sin(slope+3.14/18)
wx2,wy2 = wx/np.absolute(wx)*r*cos(slope+3.14/18),wx/np.absolute(wx)*r*sin(slope-3.14/18)
print(wx1)
print(wx2)
x2_arr = np.zeros(200)  #Point 2
y2_arr = np.zeros(200)
x1_arr = np.zeros(200)  #Point 2
y1_arr = np.zeros(200)
llim,hlim = 35,145

# def workspace(llim,hlim):
#     px,py,p1,p2 = [],[],[],[]
#     for o1 in range(llim,hlim,5):
#         for o2 in range(llim,hlim,5):
#             p1.append(l1*cos(o1*3.14/180))
#             p2.append(l1*sin(o1*3.14/180))
#             xi = l1*cos(o1*3.14/180)+l2*cos((o1+o2-90)%360*3.14/180)
#             yi = l1*sin(o1*3.14/180)+l2*sin((o1+o2-90)%360*3.14/180)
#             px.append(xi)
#             py.append(yi)
#     return px,py,p1,p2


for i in range(0,150):
    x2 = i0 + (wx-i0)*(i/150)
    y2 = j0 + (wy-j0)*(i/150)
    x2_arr[i] = x2
    y2_arr[i] = y2
    theta2 = -wx/np.absolute(wx) * arccos((x2**2+y2**2-l1**2-l2**2)/(2*l1*l2))
    theta1 = arccos((x2*(l1+l2*cos(theta2))+y2*l2*sin(theta2))/(x2**2 +y2**2))
    #print(theta1*180/3.14)
    x1 = l1*cos(theta1)
    y1 = l1*sin(theta1)
    x1_arr[i]= x1
    y1_arr[i] = y1
for i in range(0,50):
    x2 = wx1 + (wx2-wx1)*sin(i%5*3.14/8)
    y2 = wy1 + (wy2-wy2)*sin(i%5*3.14/8)
    x2_arr[i+150] = x2
    y2_arr[i+150] = y2
    theta2 = -wx/np.absolute(wx) * arccos((x2**2+y2**2-l1**2-l2**2)/(2*l1*l2))
    theta1 = arccos((x2*(l1+l2*cos(theta2))+y2*l2*sin(theta2))/(x2**2 +y2**2))
    #print(theta1*180/3.14)
    x1 = l1*cos(theta1)
    y1 = l1*sin(theta1)
    x1_arr[i+150]= x1
    y1_arr[i+150] = y1
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
    wx, wy, ".", lw=2, color='black'
) 
# px,py,p1,p2 = workspace(llim,hlim)
(line3,) = ax.plot(
    [], [], ".", lw=2, color='black'
) 
def init():
    line.set_data([], [])
    return (line,)
def animate(i):
    x_points = [0, x1_arr[i], x2_arr[i]]
    y_points = [0, y1_arr[i], y2_arr[i]]
    line.set_data(x_points, y_points)
    
    return (line,)#line2
ani = animation.FuncAnimation(
    fig, animate, init_func=init, frames=len(x1_arr)-1, interval=40, blit=True, repeat=False
)
## to save animation, uncomment the line below. Ensure ffmpeg is installed:
ani.save('Spring_2R.mp4', fps=30, extra_args=['-vcodec', 'libx264'])

#workspace
# def init():
#     line.set_data([], [])
#     return (line,)
# def animate(i):
#     x_points = [0, p1[i], px[i]]
#     y_points = [0, p2[i], py[i]]
#     line.set_data(x_points, y_points)
#     line3.set_data(px[0:i],py[0:i])
#     return (line,line3)#line2
# ani = animation.FuncAnimation(
#     fig, animate, init_func=init, frames=len(p1)-1, interval=40, blit=True, repeat=False
# )
## to save animation, uncomment the line below. Ensure ffmpeg is installed:
ani.save('Spring_2R.mp4', fps=30, extra_args=['-vcodec', 'libx264'])
# show the animation
plt.show()
