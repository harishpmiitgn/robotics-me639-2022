import math
import numpy as np
import matplotlib.pyplot as plt
import cv2
import scipy
from render import Renderer
import pandas as pd
import xlsxwriter


# Here, the given trajectory is circle. So manipulator will trace a circle.

w=[] # list contains the x-coordinates of end-effector
u=[] # list contains the y-coordinates of end-effector
total_points=int(input('Enter the number of points required to create the circle: '))


for z_1 in range(0,total_points):
    w.append((math.cos((6.28/total_points)*z_1))*100)
    u.append((math.sin((6.28/total_points)*z_1))*100)


class object(Renderer):
    # Initializing all the required variables.
    def __init__(self, recordLocation=None):
        super().__init__(recordLocation=recordLocation)
        self.its=0


        self.l1=225 # Length of link 1
        self.l2=226  # Length of link 2
        self.theta=0
        self.q1=0    # Angle made by link 1 with horizontal
    
        self.q2=0     # Angle made by link 2 with horizontal
        self.x0=200  # x-coordiante of the center
        self.y0=200  # y-coordinate of the center
    
        self.points=[]

    def getInfo(self):
        # Printing the required variables during simulation.
        info = {
            'q1' : round(self.q1, 4),
            'q2' : round(self.q2, 4),
            'x'  : round(self.x,  4),
            'y'  : round(self.y,  4),
         }
        return info

    def plot(self):
        # Plotting the desired end-effector points.
        data=np.array(list(self.points))
    
        plt.scatter(data[:,2],data[:,3]) 
    
        plt.show()
        return data


    def step(self,w,u,i):
        # Incrementing the variables according to provided excel file.
        
        self.x = self.x0 + w[i]
        self.y = self.y0 + u[i]
        self.theta=math.acos(((self.x-50)**2+(self.y-500)**2-(self.l1)**2-(self.l2)**2)/(2*self.l1*self.l2))
        
        self.q1=math.atan((self.y-500)/(self.x-50))-math.atan((self.l2*math.sin(self.theta))/(self.l1+(self.l2*math.cos(self.theta))))
        self.q2=(self.q1+self.theta)
        self.a=50 # Starting point of the link 1, x-coordinate
        self.b=500 # Starting point of the link 1, y-coordinate
        self.c=(self.a)+(self.l1*math.cos(self.q1))   # Starting point of the link 2, x-coordinate
        self.d=(self.b)+(self.l1*math.sin(self.q1))    # Starting point of the link 2, y-coordinate
        self.x1=(self.c)+(self.l2*math.cos(self.q2))    # ending point of the link 2, x-coordinate
        self.y1=(self.d)+(self.l2*math.sin(self.q2))      # ending point of the link 2, y-coordinate
  
        
        self.its += 1
        
        self.points.append((self.c,self.d,int(self.x),int(self.y),int(self.x1),(self.y1)))
        


    def draw(self,image):
        # Showing the animation.
        for c,d,x,y,x1,y1 in self.points:
            
            cv2.line(image,(50,500),(int(self.c),int(self.d)),(0,255,0),1)
            cv2.line(image,(int(self.c),int(self.d)),(int(self.x1),int(self.y1)),(0,0,255),1)
            cv2.circle(image,(int(x1),int(y1)),3,(255,0,),-1)
            
        return image


anim=object(recordLocation='anim.mp4')
for i in range(len(w)):
    anim.step(w,u,i)
    anim.render(height=600,pause=1)

# Creating the excel file
df=pd.DataFrame(anim.plot(),columns=['c','d', 'X', 'Y', 'X1','Y1'])
writer=pd.ExcelWriter('circle3.xlsx',engine='xlsxwriter')
df.to_excel(writer,sheet_name='Sheet1',index=False)
writer.save()


