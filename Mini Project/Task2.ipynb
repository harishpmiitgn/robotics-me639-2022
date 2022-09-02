import numpy as np
import pygame
import math
from scipy.integrate import odeint

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

white = (255, 255, 255)
green = (0, 255, 0)
blue = (0, 0, 128)
tor1 = 0
tor2 = 0 
pygame.init()
font = pygame.font.Font('freesansbold.ttf', 32)



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
x1,y1 = 0,0

F = 20

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

def find_force(coor1,coor2, coor3, coor_j1):
    if coor1[0] != coor2[0]:
        slope = (coor2[1] - coor1[1])/(coor2[0] - coor1[0])
        const = ((coor1[1])*(coor2[0]) - (coor1[0])*(coor2[1]))/(coor2[0] - coor1[0])
    else:
        slope = math.inf
        const = math.inf
    sign_base = np.sign(coor_j1[1] - (slope*coor_j1[0] + const))
    const2 = coor3[1] + coor3[0]*(1/slope)
    coor_c1x = coor3[0] + 35*abs(math.sin(math.atan(slope)))
    coor_c1y = -coor_c1x*(1/slope) + const2
    coor_c2x = coor3[0] - 35*abs(math.sin(math.atan(slope)))
    coor_c2y = -coor_c2x*(1/slope) + const2
    sign1 = np.sign(coor_c1y - (slope*coor_c1x + const))
    sign2 = np.sign(coor_c2y - (slope*coor_c2x + const))
    if sign_base == sign1:
        Fx = F*abs(math.sin(math.atan(slope)))*np.sign(coor_c2x - coor3[0])
        Fy = F*abs(math.cos(math.atan(slope)))*np.sign(coor_c2y - coor3[1])
        return (Fx,Fy,coor_c2x,coor_c2y)
    elif sign_base == sign2:
        Fx = F*abs(math.sin(math.atan(slope)))*np.sign(coor_c1x - coor3[0])
        Fy = F*abs(math.cos(math.atan(slope)))*np.sign(coor_c1y - coor3[1])
        return (Fx,Fy,coor_c1x,coor_c1y)
    else:
        Fx = F*abs(math.sin(math.atan(slope)))*np.sign(coor_c2x - coor3[0])
        Fy = F*abs(math.cos(math.atan(slope)))*np.sign(coor_c2y - coor3[1])  
        return (Fx,Fy,coor_c2x,coor_c2y)   

def T1f(Fx_,Fy_,q_1): 
    return -Fx_*l1*math.sin(math.radians(q_1)) + Fy_*l1*math.cos(math.radians(q_1))

def T2f(Fx_,Fy_,q_2): 
    return -Fx_*l2*math.sin(math.radians(q_2)) + Fy_*l2*math.cos(math.radians(q_2))


def T1(q1,q1_,q1__,q2,q2_,q2__,m1,m2,l1,l2):
  return ((1/3)*m1*(l1**2)*q1__ + m2*(l1**2)*q1__ + (m2/2)*q2__*l1*l2*math.cos(math.radians(q2-q1)) - (m2/2)*(q2_**2)*l1*l2*math.sin(math.radians(q2-q1)) + m1*g*(l1/2)*math.cos(math.radians(q1)) + m2*g*l1*math.cos(math.radians(q1)))

def T2(q1,q1_,q1__,q2,q2_,q2__,m1,m2,l1,l2):
  return ((1/3)*m2*(l1**2)*q1__ + (m2/2)*q1__*l1*l2*math.cos(math.radians(q2-q1)) + (m2/2)*(q1_**2)*l1*l2*math.sin(math.radians(q2-q1)) + m2*g*(l2/2)*math.cos(math.radians(q2)))

# q : [q1,q2,q1_,q2_]
def model(q,t):

  Tor1 = T1(q1d,q1d_,q1d__,q2d,q2d_,q2d__,m1,m2,l1,l2)
  Tor2 = T1(q1d,q1d_,q1d__,q2d,q2d_,q2d__,m1,m2,l1,l2)

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

def find_theta(x,y,l1,l2):
    D = ((x-x1)**2 + (y-y1)**2 - l1**2 - l2**2)/(2*l1*l2)
    if x-x1 > y-y1:
        return math.degrees(math.atan2(-abs((1-D**2)**0.5),D))
    else:
        return math.degrees(math.atan2(abs((1-D**2)**0.5),D))
    #return math.degrees(math.atan2(-abs((1-D**2)**0.5),D))

def find_q1(x,y,l1,l2):
    theta = math.radians(find_theta(x,y,l1,l2))
    return math.degrees(math.atan2((y-y1),(x-x1)) - math.atan2((l2*math.sin(theta)),(l1 + l2*math.cos(theta))))

t0 = 0
tf = 1

q10 = q1
q10_ = 0
q1f = q1
q1f_ = 0 

q20 = q2
q20_ = 60
q2f = q2
q2f_ = 0

T = [[1,t0,t0**2,t0**3],[0,1,2*t0,3*(t0**2)],[1,tf,tf**2,tf**3],[0,1,2*tf,3*(tf**2)]]
T = np.array(T)
Q1 = [q10,q10_,q1f,q1f_]
Q1 = np.array(Q1)
Q2 = [q20,q20_,q2f,q2f_]
Q2 = np.array(Q2)

A = np.linalg.inv(T).dot(Q1)
B = np.linalg.inv(T).dot(Q2)

def fq1(t,A) :
  q = A[0] + t*A[1] + (t**2)*A[2] + (t**3)*A[3]
  q_dot = A[1] + (2*t)*A[2] + (3*(t**2))*A[3]
  q_ddot = 2*A[2] + 6*t*A[3]
  return [q,q_dot,q_ddot] 

def fq2(t,B) :
  q = B[0] + t*B[1] + (t**2)*B[2] + (t**3)*B[3]
  q_dot = B[1] + (2*t)*B[2] + (3*(t**2))*B[3]
  q_ddot = 2*B[2] + 6*t*B[3]
  return [q,q_dot,q_ddot]

del_t = tf - t0
N = del_t*(fps)
t_ = t0
q0 = [q10,q20,q10_,q20_]

run = True
a = N +1
speed = int((1/fps)*1000)
trajectory = []
num =0
drag = False
draw_arrow = False
Fx = 0
Fy = 0
while run:

    text = font.render('Torque1: ' + str(tor1) + '  Torque2: ' + str(tor2), True, green)
    textRect = text.get_rect()
    textRect.center = (350, 30)
    text1 = font.render('Fx: ' + str(Fx) + '  Fy: ' + str(Fy), True, green)
    text1Rect = text1.get_rect()
    text1Rect.center = (350, 70)

    if drag == 0 or drag == 1 or drag == 2:
        if drag == 0:
            mouse_x, mouse_y = pygame.mouse.get_pos()
            coord4 = (mouse_x,mouse_y)
            coord5 = coord4
            coord6 = coord5
        if drag == 1:
            mouse_x, mouse_y = pygame.mouse.get_pos()
            coord5 = (mouse_x,mouse_y)
            coord6 = coord5
        if drag == 2:
            mouse_x, mouse_y = pygame.mouse.get_pos()
            coord6 = (mouse_x,mouse_y)
        draw_arrow = False
    else:
        if a <= N:
            q0 = [fq1(t_,A)[0], fq2(t_,B)[0], fq1(t_,A)[1], fq2(t_,B)[1]]
            t_ += dt
            [q1d, q1d_, q1d__] = [fq1(t_,A)[0], fq1(t_,A)[1], fq1(t_,A)[2]]
            [q2d, q2d_, q2d__] = [fq2(t_,B)[0], fq2(t_,B)[1], fq2(t_,B)[2]]
            q_v = odeint(model,q0,[0,dt])
            q1 = q_v[1][0]
            q2 = q_v[1][1]
            a += 1 
        else:
            trajectory = []
            coord4_ = to_pygame(coord4, height)
            coord5_ = to_pygame(coord5, height)
            coord6_ = to_pygame(coord6, height)
            coord2_ = to_pygame(coord2, height)
            if ((((coord6[0] - coord3[0] )**2) + ((coord6[1]-coord3[1])**2) )**0.5) < 10:
                draw_arrow = True
                tor1 = round(Torque1, 2)
                tor2 = round(Torque2, 2)
                Fx = round(Force[0], 2)
                Fy = round(Force[1], 2)
            else:
                tor1 = 0
                tor2 = 0
                Fx = 0
                Fy = 0
            Force = find_force(coord4_,coord5_, coord6_, coord2_)
            Torque1 = T1f(Force[0],Force[1],q1)
            Torque2 = T1f(Force[0],Force[1],q2)
            arrow_head = to_pygame((Force[2], Force[3]), height)
            x_target_, y_target_ = coord6
            y_target_ = height - y_target_
            a = 0
            try:
                q1_target = int(find_q1(x_target_,y_target_,l1,l2))
                q2_target = int(find_q1(x_target_,y_target_,l1,l2) + find_theta(x_target_,y_target_,l1,l2))
                t0 = t0
                tf = tf

                q10 = q1
                q10_ = 0
                q1f = q1_target 
                q1f_ = 0 

                q20 = q2
                q20_ = 0
                q2f = q2_target
                q2f_ = 0

                T = [[1,t0,t0**2,t0**3],[0,1,2*t0,3*(t0**2)],[1,tf,tf**2,tf**3],[0,1,2*tf,3*(tf**2)]]
                T = np.array(T)
                Q1 = [q10,q10_,q1f,q1f_]
                Q1 = np.array(Q1)
                Q2 = [q20,q20_,q2f,q2f_]
                Q2 = np.array(Q2)

                A = np.linalg.inv(T).dot(Q1)
                B = np.linalg.inv(T).dot(Q2)

                del_t = tf - t0
                N = del_t*(fps)
                t_ = t0
                q0 = [q10,q20,q10_,q20_]

                a += 1
            except:
                pass

    x2 = x1 + math.cos(math.radians(q1)) * l1
    y2 = y1 + math.sin(math.radians(q1)) * l1
    x3 = x2 + math.cos(math.radians(q2)) * l2
    y3 = y2 + math.sin(math.radians(q2)) * l2
    coord1 = to_pygame((x1,y1), height)
    coord2 = to_pygame((x2,y2), height) 
    coord3 = to_pygame((x3,y3), height)
    trajectory.append(coord3)
    pygame.time.delay(speed)
    for event_ in pygame.event.get():
        if event_.type == pygame.QUIT:
            run =False
        elif event_.type == pygame.MOUSEBUTTONDOWN:
            if drag == 3:
                drag = 0
            else:
                drag += 1

    window.fill(backGroundColor)
    pygame.draw.line(window, colour1, coord1, coord2, border_width)
    pygame.draw.line(window, colour2, coord2, coord3, border_width)
    pygame.draw.circle(window, colour3, coord1, circle_radius, border_width)
    pygame.draw.circle(window, colour3, coord2, circle_radius, border_width)
    pygame.draw.circle(window, colour3, coord3, circle_radius, border_width)
    pygame.draw.circle(window, colour2, pygame.mouse.get_pos(), 10, 1)
    pygame.draw.circle(window, colour3, coord4, 5, border_width)
    pygame.draw.circle(window, colour3, coord5, 5, border_width)
    pygame.draw.line(window, colour2, coord4, coord5, 5)
    pygame.draw.circle(window, colour3, coord6, 5, border_width)
    if draw_arrow:
        pygame.draw.line(window, colour1, coord6, arrow_head, 5)
        pygame.draw.circle(window, colour2, arrow_head, 5, border_width)
    for i in trajectory:
        pygame.draw.circle(window, colour1, i, 3, 3)
    window.blit(text, textRect)
    window.blit(text1, text1Rect)
    pygame.display.update()
    num+= 1
pygame.quit()
