from msvcrt import kbhit
import numpy as np
import pygame
import math
from scipy.integrate import odeint

pygame.init()
height = 700
width = 700
window = pygame.display.set_mode((height,width))
backGroundColor=pygame.Color("WHITE")
window.fill(backGroundColor)

colour1 = (0,255,0)
colour2 = (255,0,0)
colour3 = (0,0,0)
circle_ = (350,350)

circle_radius = 20
border_width = 20 


def to_pygame(coords, height):
    return (coords[0], height - coords[1])

def fwd_kinematics(l1,l2,q1,q2):
    x = l1*math.cos(math.radians(q1)) + l2*math.cos(math.radians(q2))
    y = l1*math.sin(math.radians(q1)) + l2*math.sin(math.radians(q2))
    return (x,y)

q1 = 30
q2 = 60
l1 = 300
l2 = 200
(x_neutral, y_neutral) = fwd_kinematics(l1,l2,q1,q2)
x1,y1 = 0,0
k = 1000
b_ = 15

fps = 600
dt = 1/fps

m1  = 0.1 
m2  = 0.1
g = 9.8
err = 2 

A0 = (m2 + (1/3)*m1)*(l1**2) 
A1 = (m2/2)*l1*l2
A2 = m1*g*(l1/2) + m2*g*(l1)
A3 = m2*g*(l2/2)
A4 = (1/3)*(m2)*(l2**2)


x2 = x1 + math.cos(math.radians(q1)) * l1
y2 = y1 + math.sin(math.radians(q1)) * l1
x3 = x2 + math.cos(math.radians(q2)) * l2
y3 = y2 + math.sin(math.radians(q2)) * l2

def find_theta(x,y,l1,l2):
    D = ((x-x1)**2 + (y-y1)**2 - l1**2 - l2**2)/(2*l1*l2)
    #if x-x1 > y-y1:
    #    return math.degrees(math.atan2(-abs((1-D**2)**0.5),D))
    #else:
    #    return math.degrees(math.atan2(abs((1-D**2)**0.5),D))
    return math.degrees(math.atan2(-abs((1-D**2)**0.5),D))

def find_q1(x,y,l1,l2):
    theta = math.radians(find_theta(x,y,l1,l2))
    return math.degrees(math.atan2((y-y1),(x-x1)) - math.atan2((l2*math.sin(theta)),(l1 + l2*math.cos(theta))))

q1_neutral = find_q1(x_neutral,y_neutral,l1,l2)
q2_neutral = (find_q1(x_neutral,y_neutral,l1,l2) + find_theta(x_neutral,y_neutral,l1,l2))

def Fx(q1eq,q2eq,q1_new,q2_new,q1_new_,q2_new_):
    damped_f_x = -b_*(-l1*math.sin(math.radians(q1_new))*q1_new_ - l2*math.sin(math.radians(q2_new))*q2_new_)
    spring_f_x = -k*(l1*math.cos(math.radians(q1_new)) + l2*math.cos(math.radians(q2_new)) - (l1*math.cos(math.radians(q1eq)) + l2*math.cos(math.radians(q2eq))))
    return (spring_f_x + damped_f_x)

def Fy(q1eq,q2eq,q1_new,q2_new,q1_new_,q2_new_):
    damped_f_y = -b_*(l1*math.cos(math.radians(q1_new))*q1_new_ + l2*math.cos(math.radians(q2_new))*q2_new_)
    spring_f_y = -k*(l1*math.sin(math.radians(q1_new)) + l2*math.sin(math.radians(q2_new)) - (l1*math.sin(math.radians(q1eq)) + l2*math.sin(math.radians(q2eq))))
    return (spring_f_y + damped_f_y)

def T1spr(q_1,q_2,q_1_,q_2_):
    Fx_ = Fx(q1_neutral,q2_neutral,q_1,q_2,q_1_,q_2_)
    Fy_ = Fy(q1_neutral,q2_neutral,q_1,q_2,q_1_,q_2_) 
    return -Fx_*l1*math.sin(math.radians(q_1)) + Fy_*l1*math.cos(math.radians(q_1))

def T2spr(q_1,q_2,q_1_,q_2_):
    Fx_ = Fx(q1_neutral,q2_neutral,q_1,q_2,q_1_,q_2_)
    Fy_ = Fy(q1_neutral,q2_neutral,q_1,q_2,q_1_,q_2_) 
    return -Fx_*l2*math.sin(math.radians(q_2)) + Fy_*l2*math.cos(math.radians(q_2))

def modelspr(q,t): # q : [q1,q2,q1_,q2_]

  Tor1 = T1spr(q[0],q[1],q[2],q[3])
  Tor2 = T2spr(q[0],q[1],q[2],q[3])

  q2__ = (1/(A0*A4 - (A1*math.cos(math.radians(q[1]-q[0]))**2))) * (A0*Tor2 - A1*Tor1*math.cos(math.radians(q[1]-q[0])) + A1*(math.cos(math.radians(q[1] - q[0])))*(A2*math.cos(math.radians(q[0])) - A1*((q[3])**2)*math.sin(math.radians(q[1]-q[0]))) - A0*(A1*((q[2])**2)*math.sin(math.radians(q[1]-q[0])) + A3*math.cos(math.radians(q[1]))))
  q1__ = (1/A0)*(Tor1 - A1*q2__*math.cos(math.radians(q[1]-q[0])) + A1*((q[3])**2)*math.sin(math.radians(q[1]-q[0])) - A2*math.cos(math.radians(q[0])))

  return [q[2],q[3],q1__, q2__]

def Closest_pt(x1,y1,collection):
    A = np.array(collection)
    leftbottom = np.array((x1,y1))
    distances = np.linalg.norm(A-leftbottom, axis=1)
    min_index = np.argmin(distances)
    return A[min_index]
    
def Closest_pt_dist(x1,y1,collection):
    A = np.array(collection)
    leftbottom = np.array((x1,y1))
    distances = np.linalg.norm(A-leftbottom, axis=1)
    min_index = np.argmin(distances)
    return distances[min_index]

q10 = q1
q10_ = 0

q20 = q2
q20_ = 60

run = True
a = 0
speed = int((1/fps)*1000)
trajectory = [] 
num_points = 1000
num =0
b = 0 
drag = False
while run:
    if drag == True:
        mouse_x, mouse_y = pygame.mouse.get_pos()
        x_target_, y_target_ = mouse_x,mouse_y
        y_target_ = height - y_target_
        q1 = int(find_q1(x_target_,y_target_,l1,l2))
        q2 = int(find_q1(x_target_,y_target_,l1,l2) + find_theta(x_target_,y_target_,l1,l2))
        b = 0
        trajectory = []
    else:
        if b == 0:
            trajectory = []
            t_ = 0
            q10 = q1
            q20 = q2
            q1_ = 0
            q2_ = 0
            q0 = [q10,q20,q10_,q20_]
            q_v = odeint(modelspr,q0,[0,dt])
            q1 = q_v[1][0]
            q2 = q_v[1][1]
            b+=1
        else:
            t_+=dt
            q0 = q_v[1]
            q_v = odeint(modelspr,q0,[0,dt])
            q1 = q_v[1][0]
            q2 = q_v[1][1]
            b+=1
            trajectory.append(coord3)
    x2 = x1 + math.cos(math.radians(q1)) * l1
    y2 = y1 + math.sin(math.radians(q1)) * l1
    x3 = x2 + math.cos(math.radians(q2)) * l2
    y3 = y2 + math.sin(math.radians(q2)) * l2

    coord1 = to_pygame((x1,y1), height)
    coord2 = to_pygame((x2,y2), height) 
    coord3 = to_pygame((x3,y3), height)
    coord_eq = to_pygame((x_neutral,y_neutral), height)

    if drag == False:
        trajectory.append(coord3)
    pygame.time.delay(speed)
    for event_ in pygame.event.get():
        if event_.type == pygame.QUIT:
            run =False
        elif event_.type == pygame.MOUSEBUTTONDOWN:
            if drag == True:
                drag = False
            elif drag == False:
                drag = True

    window.fill(backGroundColor)
    pygame.draw.line(window, colour1, coord1, coord2, border_width)
    pygame.draw.line(window, colour2, coord2, coord3, border_width)
    pygame.draw.circle(window, colour3, coord1, circle_radius, border_width)
    pygame.draw.circle(window, colour3, coord2, circle_radius, border_width)
    pygame.draw.circle(window, colour3, coord3, circle_radius, border_width)
    pygame.draw.circle(window, colour3, coord_eq, 5, 0)
    pygame.draw.circle(window, colour2, pygame.mouse.get_pos(), 10, 1)
    for i in trajectory:
        pygame.draw.circle(window, colour1, i, 3, 3)
    pygame.display.update()
pygame.quit()
