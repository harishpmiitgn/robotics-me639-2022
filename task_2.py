import math
import matplotlib.pyplot as plt
import numpy as np
from render import Renderer
import cv2
import time


class object(Renderer):
    def __init__(self, recordLocation=None):
        super().__init__(recordLocation=recordLocation)
        self.x=0
        self.y=0
        self.x0=200
        self.y0=400
        self.q1=0
        self.q2=0
        self.l1=100
        self.l2=100
        self.theta=0
        self.its = 0
        self.m1=1
        self.m2=1
        self.g= 9.81

        self.traj=[]
        self.angle=[]
        
        
        self.tau1=0
        self.tau2=0
        self.Fx=0
        self.Fy=0

        #Wall as a line joining points (100,100) and (500,200)
        #x-4y+300
        self.wx1= 100
        self.wx2= 500
        self.wy1=100
        self.wy2=200

        #Point lying on line
        Px=300
        Py=150

        self.tau_ext_1 =0 #
        self.tau_ext_2 =0 #

        self.its =1
        self.traj=[]
        self.angle=[]

    def getInfo(self):
        info = {
            'Fx(N)' : round(self.Fx),
            'Fy(N)' : round(self.Fy),
            'Torque1(N-m)'    : round(self.tau1),
            'Torque2(N-m)'    : round(self.tau2)
        }
        return info


    def calculations(self):
       
        #trajectory to follow from initial point to wall
        X=np.linspace(100,300,num = 100,endpoint = True,retstep = False,dtype = None)
        Y=np.linspace(100,150,num = 100,endpoint = True,retstep = False,dtype = None)

        if self.its>=1:

            self.Fx=100
            self.Fy=100

            self.tau1_ext = -self.l1*math.sin(self.q1)*self.Fx + self.l1*math.cos(self.q1)*self.Fx
            self.tau2_ext = -self.l2*math.sin(self.q2)*self.Fx + self.l2*math.cos(self.q2)*self.Fy

        X=np.linspace(100,300,num = 100,endpoint = True,retstep = False,dtype = None)
        Y=np.linspace(100,150,num = 100,endpoint = True,retstep = False,dtype = None)

        #end effector following normal
        for i in range(100):
            x= X[i]
            y= Y[i]

        self.x= x-400
        self.y= y-200

        #Calculating angles
        theta= np.arccos((self.x**2+self.y**2-self.l1**2-self.l2**2)/(2*self.l1*self.l2))
        self.q1 = np.arctan(self.y/self.x) - np.arctan((self.l2*np.sin(theta))/(self.l1+self.l2*np.cos(theta)))
        self.q2 = self.q1+theta


        if x<400:
            self.q1= -np.pi+self.q1
            self.q2= -np.pi+self.q2

        
        self.its+=0.01

        self.traj.append((int(x), int(y), time.time()))
        self.angle.append((self.q1, self.theta, time.time()))


        #Langrangian 
        for i in range(len(self.angle)):
            if i>2:
                q1_1=self.angle[i-2][0]
                q1_2=self.angle[i-1][0]
                q1_3=self.angle[i][0]

                q2_1=self.angle[i-2][1]
                q2_2=self.angle[i-1][1]
                q2_3=self.angle[i][1]
                
                t1=self.angle[i-2][2]
                t2=self.angle[i-1][2]
                
                t_c=self.angle[i][2]


                dt21=(t2-t1)/2
                dt32=(t_c-t2)/2

                dq1_last= (q1_2-q1_1)/dt21
                dq2_last=(q2_2-q2_1)/dt21

                dq1=(q1_3-q1_2)/dt32
                dq2=(q2_3-q2_2)/dt32

                ddq1= (2*(dq1-dq1_last))/(t_c-t1)
                ddq2=(2*(dq2-dq2_last))/(t_c-t1)


                self.tau1_1=  (self.m1*self.l1**2 + self.m2*(self.l1**2+2*self.l1*self.l2*math.cos(q2_3)+self.l2**2))*ddq1 + self.m2*(self.l1*self.l2*math.cos(q2_3)+self.l2**2)*ddq2 -self.m2*self.l1*self.l2*math.sin(q2_3)*(2*dq1*dq2+ddq2**2) + (self.m1+self.m2)*self.l1*self.g*math.cos(q1_3) + self.m2*self.g*self.l2*math.cos(q1_3+q2_3)
                self.tau2_1 = self.m2*(self.l1*self.l2*math.cos(q2_3)+self.l2**2)*ddq1 + self.m2*self.l2**2*q2_3**2 +  self.m2*self.l1*self.l2*dq1**2*math.sin(q2_3) + self.m2*self.g*self.l2*math.cos(q1_3+q2_3)
        
                #Required taus
                self.tau1 = self.tau1_1+ self.tau1_ext
                self.tau2 = self.tau2_1+ self.tau2_ext
                    
            
        

        


    def draw(self, image):
        #Wall
        cv2.line(image, (self.wx1,self.wy1), (self.wx2,self.wy2), (120, 0, 0), 5)
        

        #Joint and end-effector pointa
        self.x1=self.x0+int(self.l1*np.cos(self.q1))
        self.y1=self.y0+int(self.l1*np.sin(self.q1))
        self.x2=self.x1+int(self.l1*np.cos(self.q1))+int(self.l2*np.cos(self.q2))
        self.y2=self.y1+int(self.l1*np.sin(self.q1))+int(self.l2*np.sin(self.q2))

         #Drawing the links of the robot
        cv2.line(image, (self.x0, self.y0),(self.x1,self.y1), (0, 0, 255), 1)      
        cv2.line(image, (self.x1,self.y1) , (int(self.x2),int(self.y2)), (0, 255, 0), 1)


        X=np.linspace(100,300,num = 100,endpoint = True,retstep = False,dtype = None)
        Y=np.linspace(100,150,num = 100,endpoint = True,retstep = False,dtype = None)

        #end effector following normal
        for i in range(100):
            x= X[i]
            y= Y[i]
            cv2.circle(image, (x, y), 1, (0, 0, 120), -1)

        return image




obj = object()    

for i in range(1000):
    obj.calculations() 
    if i % 1 == 0:
       obj.render(height= 600, pause = 10)

    

    
    
