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
    #x,y = to_pygame((x,y),height)
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


workspace_pygame = []

q1_ = np.linspace(35,145,100)
q2_ = np.linspace(35,145,100)
for i in q1_:
    for j in q2_:
        coordi = fwd_kinematics(l1,l2,i,j)
        coordi_pygame = to_pygame(coordi, height)
        workspace_pygame.append(coordi_pygame)
run = True
while run:
    x2 = x1 + math.cos(math.radians(q1)) * l1
    y2 = y1 + math.sin(math.radians(q1)) * l1
    x3 = x2 + math.cos(math.radians(q2)) * l2
    y3 = y2 + math.sin(math.radians(q2)) * l2

    coord1 = to_pygame((x1,y1), height)
    coord2 = to_pygame((x2,y2), height) 
    coord3 = to_pygame((x3,y3), height)

    pygame.time.delay(1)
    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            run =False
    window.fill(backGroundColor)
    pygame.draw.line(window, colour1, coord1, coord2, border_width)
    pygame.draw.line(window, colour2, coord2, coord3, border_width)
    pygame.draw.circle(window, colour3, coord1, circle_radius, border_width)
    pygame.draw.circle(window, colour3, coord2, circle_radius, border_width)
    pygame.draw.circle(window, colour3, coord3, circle_radius, border_width)
    for i in workspace_pygame:
        pygame.draw.circle(window, colour3, i, 1, border_width)  
    pygame.display.update()
pygame.quit()