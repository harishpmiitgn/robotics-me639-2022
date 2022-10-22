from turtle import color
import numpy as np
from numpy import pi, sin, cos, sqrt, absolute, arccos, arctan, sign
import matplotlib.pyplot as plt
import matplotlib.animation as animation
l1 = 10
l2 = 10
# global path_x,path_y,x2,y2,x1,y1
# increment = 0.1  # angle incremement
# rot_num = 3
# angle_minus_last = np.arange(0, rot_num * 2 * pi, increment)
# R_Angles = np.append(angle_minus_last, rot_num * 2 * pi)

x0 = 0
y0 = 0
num=200
no_points =int(input("enter no. of points:")) 
ax_points = np.zeros(no_points,dtype=float)
by_points = np.zeros(no_points,dtype=float)

x2_arr = np.zeros(200*no_points,dtype=float)  #Point 2
y2_arr = np.zeros(200*no_points,dtype=float)
x2,y2 = 0,0

x1,y1 =0,0
x1_arr = np.zeros(200*no_points,dtype=float)  #Point 3
y1_arr = np.zeros(200*no_points,dtype=float)
for n in range(0,no_points):
    coord = input("Enter x,y coordinates of point "+str(n)+":")
    ax_points[n]=float(coord.split(",")[0])
    by_points[n]=float(coord.split(",")[1])

#Linear interpolation
def lin_inter(ax_points,by_points):
    each_point =0
    for i in range(0,num*(no_points-1)):
        if i%200==0:
            a1,b1 = ax_points[each_point],by_points[each_point]
            a2,b2 = ax_points[each_point+1],by_points[each_point+1]
            each_point+=1
        x2 = a1 + (a2-a1)*(i%200)/num
        y2 = b1 + (b2-b1)*(i%200)/num
        x2_arr[i] = x2
        y2_arr[i] = y2
        theta2 = -arccos((x2**2+y2**2-l1**2-l2**2)/(2*l1*l2))
        theta1 = arccos((x2*(l1+l2*cos(theta2))+y2*l2*sin(theta2))/(x2**2 +y2**2))
        #print(theta1*180/3.14)
        x1 = l1*cos(theta1)
        y1 = l1*sin(theta1)
        x1_arr[i]= x1
        y1_arr[i] = y1
    return x1_arr,y1_arr,x2_arr,y2_arr

#circular Interpolation
def circle_inter(ax_points,by_points,n0p):
    count =0
    for n in range (0,n0p-2):
        B = [[ax_points[n]**2 - ax_points[n+1]**2 + by_points[n]**2 - by_points[n+1]**2],
        [ax_points[n+1]**2 - ax_points[n+2]**2 + by_points[n+1]**2 - by_points[n+2]**2]]
        A = [[2*(ax_points[n]-ax_points[n+1]),2*(by_points[n]-by_points[n+1])],
        [2*(ax_points[n+1]-ax_points[n+2]),2*(by_points[n+1]-by_points[n+2])]]
        a_inv = np.linalg.inv(np.array(A))
        centr_mat = a_inv.dot(np.array(B))
        r_sq = (ax_points[n+1]-centr_mat[0])**2 + (by_points[n+1]-centr_mat[1])**2
        print(centr_mat)
        # print((ax_points[2]-ax_points[0])//200)
        for i in range(0,200):
            xi = ax_points[n] + i*(ax_points[n+1]-ax_points[n])/200
            yi = centr_mat[1] + np.sqrt(r_sq-(xi-centr_mat[0])**2)
            if i==0:
                xi,yi = ax_points[count],by_points[count]
            x2,y2=xi,yi
            x2_arr[count*200+i] = x2
            y2_arr[count*200+i] = y2
            theta2 = -arccos((x2**2+y2**2-l1**2-l2**2)/(2*l1*l2))
            theta1 = arccos((x2*(l1+l2*cos(theta2))+y2*l2*sin(theta2))/(x2**2 +y2**2))
            #print(theta1*180/3.14)
            x1 = l1*cos(theta1)
            y1 = l1*sin(theta1)
            x1_arr[count*200+i]= x1
            y1_arr[count*200+i] = y1
        count+=1
    return x1_arr,y1_arr,x2_arr,y2_arr
choice = int(input("Enter 1 for Linear Interpolation and 2 for circular Interpolation :"))
if choice == 1:
    x1_arr,y1_arr,x2_arr,y2_arr = lin_inter(ax_points,by_points)
elif choice == 2:
    x1_arr,y1_arr,x2_arr,y2_arr = circle_inter(ax_points,by_points,no_points)
else:
    print("Valid Choice")
fig = plt.figure()
ax = fig.add_subplot(
    111, aspect="equal", autoscale_on=False, xlim=(-10, 25), ylim=(-10, 25)
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
    [], [], ".", lw=2, color='green'
) 
# initialization function
def init():
    line.set_data([], [])
    return (line,)

path_x,path_y = [x2_arr[1]],[y2_arr[1]]

# animation function
def animate(i):
    x_points = [x0, x1_arr[i], x2_arr[i]]
    y_points = [y0, y1_arr[i], y2_arr[i]]
    path_x.append(x2_arr[i])
    path_y.append(y2_arr[i])
    line.set_data(x_points, y_points)
    line2.set_data(path_x,path_y)
    
    return (line,line2,)#line2
ani = animation.FuncAnimation(
    fig, animate, init_func=init, frames=len(x1_arr)-1, interval=40, blit=True, repeat=False
)
## to save animation, uncomment the line below. Ensure ffmpeg is installed:
ani.save('Trajectory_2R.mp4', fps=30, extra_args=['-vcodec', 'libx264'])

# show the animation
plt.show()
