import numpy as np
import pygame
import math


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
x1,y1 = 0,0

x2 = x1 + math.cos(math.radians(q1)) * l1
y2 = y1 + math.sin(math.radians(q1)) * l1
x3 = x2 + math.cos(math.radians(q2)) * l2
y3 = y2 + math.sin(math.radians(q2)) * l2

workspace = []
workspace_pygame = []
for i in range(360):
    for j in range(360):
        coordi = fwd_kinematics(l1,l2,i,j)
        coordi_pygame = to_pygame(coordi, height)
        workspace.append(coordi)
        workspace_pygame.append(coordi_pygame)

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
    return math.degrees(math.atan2(-abs((1-D**2)**0.5),D))

def find_q1(x,y,l1,l2):
    theta = math.radians(find_theta(x,y,l1,l2))
    return math.degrees(math.atan2((y-y1),(x-x1)) - math.atan2((l2*math.sin(theta)),(l1 + l2*math.cos(theta))))

x_target_,y_target_ = workspace[1][0], workspace[1][1]
x_target_,y_target_ = Closest_pt(x_target_,y_target_,workspace)
print(x_target_,y_target_)

q1_target = int(find_q1(x_target_,y_target_,l1,l2))
q2_target = int(find_q1(x_target_,y_target_,l1,l2) + find_theta(x_target_,y_target_,l1,l2))
q1_ = q1_target - q1
q2_ = q2_target - q2
run = True
N = 500
a = N+1
speed = 1
while run:
    if a <= N:
        q1 += q1_/N
        q2 += q2_/N
        a += 1 
    else:
        x_target_, y_target_ = pygame.mouse.get_pos()
        y_target_ = height - y_target_
        a = 0
        try:
            q1_target = int(find_q1(x_target_,y_target_,l1,l2))
            q2_target = int(find_q1(x_target_,y_target_,l1,l2) + find_theta(x_target_,y_target_,l1,l2))
            q1_ = q1_target - q1
            q2_ = q2_target - q2
            a += 1
        except:
            q1_ = 0
            q2_ = 0 
    x2 = x1 + math.cos(math.radians(q1)) * l1
    y2 = y1 + math.sin(math.radians(q1)) * l1
    x3 = x2 + math.cos(math.radians(q2)) * l2
    y3 = y2 + math.sin(math.radians(q2)) * l2
    coord1 = to_pygame((x1,y1), height)
    coord2 = to_pygame((x2,y2), height) 
    coord3 = to_pygame((x3,y3), height)
    #print(coord2,coord3)
    pygame.time.delay(speed)
    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            run =False
    window.fill(backGroundColor)
    pygame.draw.line(window, colour1, coord1, coord2, border_width)
    pygame.draw.line(window, colour2, coord2, coord3, border_width)
    pygame.draw.circle(window, colour3, coord1, circle_radius, border_width)
    pygame.draw.circle(window, colour3, coord2, circle_radius, border_width)
    pygame.draw.circle(window, colour3, coord3, circle_radius, border_width)
    pygame.draw.circle(window, colour2, pygame.mouse.get_pos(), 10, 1) 
    pygame.display.update()
pygame.quit()