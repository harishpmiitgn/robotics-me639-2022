from cProfile import label
import numpy as np
from matplotlib import pyplot as plt

def fk_2R(q1,q2,l1,l2):
    x1 = l1*np.cos(q1)
    y1 = l2*np.sin(q1)
    x2 = x1+l2*np.cos(q2)
    y2 = y1+l2*np.sin(q2)
    return [x1,y1,x2,y2]

def ik_2R(x,y,elbow,l1,l2):
    
    D = (x**2+y**2-l1**2-l2**2)/(2*l1*l2)
    if D>1 or D<-1:
        q1 = 0
        q2 = 0
        ws = 0 # Outside workspace
    else:
        ws = 1 # Inside workspace
        theta2 = np.arctan2(elbow*np.sqrt(1-D**2),D)
        q1 = np.arctan2(y,x)-np.arctan2(l2*np.sin(theta2),(l1+l2*np.cos(theta2)))
        q2 = q1+theta2
        
        

    return [ws,q1,q2]

def invstatics2R(Fx,Fy,l1,l2,q1,q2):
    tau1 = Fx*l1*np.sin(q1)-Fy*l1*np.cos(q1)
    tau2 = Fx*l2*np.sin(q2)-Fy*l2*np.cos(q2)
    return np.array([[tau1],[tau2]])

def rungekutta4(f,y0,t,tau,l1,l2,m1,m2):
        n = len(t)
        print("n",n-1)
        y = np.zeros((n,len(y0)))
        y[0]=y0
        
        for i in range(n-1):
            taui = np.array([[tau[i][0]],[tau[i][1]]])
            h = t[i+1] - t[i]
            
            k1 = f(y[i], t[i], taui,l1,l2,m1,m2)
            k2 = f(y[i] + k1 * h / 2., t[i] + h / 2., taui,l1,l2,m1,m2)
            k3 = f(y[i] + k2 * h / 2., t[i] + h / 2., taui,l1,l2,m1,m2)
            k4 = f(y[i] + k3 * h, t[i] + h,taui,l1,l2,m1,m2)
            y[i+1] = y[i] + (h / 6.) * (k1 + 2*k2 + 2*k3 + k4)
            #y[i+1] = y[i]+(t[i+1] - t[i])*f(y[i],t[i],tau,l1,l2,m1,m2)
        #y = np.round(y,3)
        return y    
def f(y,t,tau,l1,l2,m1,m2): 
            g = 9.81
            q1=y[0]
            q2=y[1]
            q1dot=y[2]
            q2dot=y[3]
            
            #print(qdot)
            
            detD = (1/3*m2*l2**2)*(1/3*m1*l1**2+m2*l1**2)-1/4*m2**2*l1**2*l2**2*(np.cos(q1-q2))**2
            
            Dinv = (1/detD)*np.array([[1/3*(m2*l2**2), -1/2*(m2*l1*l2*np.cos(q1-q2))],[-1/2*(m2)*l1*l2*np.cos(q1-q2),1/3*(m1)*l1**2+m2*l1**2]])
            
            D  = np.array([[m1*(l1/2)**2+m2*l1**2+m1*(l1**2)/12, m2*l1*(l2/2)*np.cos(q2-q1) ],[m2*l1*(l2/2)*np.cos(q2-q1),m2*(l2/2)**2+m2*(l2**2)/12]])
            #Dinv = np.round(np.linalg.inv(D),3)
            phi = np.array([[1/2*m1*l1*g*np.cos(q1)+m2*l1*g*np.cos(q1)],[m2*(l2/2)*g*np.cos(q2)]])
            #print(V)
            #C = np.array([[0,1/2*m2*l1*l2*np.sin(q1-q2)*q2dot],[-1/2*m2*l1*l2*np.sin(q1-q2)*q1dot,0]])
            Cd = np.array([[-m2*l1*l2/2*np.sin(q2-q1)*q2dot**2],[m2*l1*l2/2*np.sin(q2-q1)*q1dot**2]])
            #print(C)
            #-np.dot(C,qdot)
            qddot = np.dot(Dinv,(-Cd-phi+tau))
            print(qddot)
            print(qddot[1,0])
            ydot = np.array([q1dot,q2dot,qddot[0,0],qddot[1,0]])
            #print(ydot)
            #ydot = np.round(ydot,3) 
            
            return ydot


def invDi(qddota,qdota,qa,l1,l2,m1,m2):
        tau = np.zeros((len(qdota),2))
        for i in range(len(qddota)):    
            qddot = np.array([[qddota[i][0]],[qddota[i][1]]])
            qdot = qdota[i].T
            q = qa[i].T
            g = 9.81
            q1=q[0]
            q2=q[1]
            q1dot=qdot[0]
            q2dot=qdot[1]
            
            D  = np.array([[m1*(l1/2)**2+m2*l1**2+m1*(l1**2)/12, m2*l1*(l2/2)*np.cos(q2-q1) ],[m2*l1*(l2/2)*np.cos(q2-q1),m2*(l2/2)**2+m2*(l2**2)/12]])
            phi = np.array([[1/2*m1*l1*g*np.cos(q1)+m2*l1*g*np.cos(q1)],[m2*(l2/2)*g*np.cos(q2)]])
            Cd = np.array([[-m2*l1*l2/2*np.sin(q2-q1)*q2dot**2],[m2*l1*l2/2*np.sin(q2-q1)*q1dot**2]])
            taui = np.dot(D,qddot)+Cd+phi
            #print(taui)
            tau[i] = taui.T
        return tau
def ik_2R_array(xa,ya,elbow,l1,l2):
    
    q = np.zeros((len(xa),2))
    for i in range(len(xa)):
        x = xa[i]
        y = ya[i]
        print(x)
        D = (x**2+y**2-l1**2-l2**2)/(2*l1*l2)
        if D>1 or D<-1:
            q1 = 0
            q2 = 0
            ws = 0 # Outside workspace
        else:
            ws = 1 # Inside workspace
            theta2 = np.arctan2(elbow*np.sqrt(1-D**2),D)
            q1 = np.arctan2(y,x)-np.arctan2(l2*np.sin(theta2),(l1+l2*np.cos(theta2)))
            q2 = q1+theta2
            q[i] = np.array([q1,q2])
    return q
# def desired_tau(qddot,qdot,q,m1,m2,l1,l2):
#    [1/3*(m1)*(l1)**2+m2*l1**2,0.5*m2*l1*l2*cos(q1-q2)] 
    

    
#     return q,qdot,qddot

### Visualization Functions #####

def plot_joint(q1,q2,l1,l2):
        [x1,y1,x2,y2]=fk_2R(q1,q2,l1,l2)
        plt.plot([0,x1],[0,y1],'-r')
        plt.plot([x1,x2],[y1,y2],'-b')
        plt.plot([0,x1,x2],[0,y1,y2],'ok')
        plt.xlim([-1.1*(l1+l2),1.1*(l1+l2)])
        plt.ylim([-1.1*(l1+l2),1.1*(l1+l2)])
        

def plot_endpt(x,y,elbow,l1,l2):
    [ws,q1,q2]=ik_2R(x,y,elbow,l1,l2)
    
    if ws == 1:
        [x1,y1,x2,y2]=fk_2R(q1,q2,l1,l2)
        print(x1)
        print(x2)
        plt.plot([0,x1],[0,y1],'-r')
        plt.plot([x1,x2],[y1,y2],'-b')
        plt.plot([0,x1,x2],[0,y1,y2],'ok')
        plt.xlim([-1.1*(l1+l2),1.1*(l1+l2)])
        plt.ylim([-1.1*(l1+l2),1.1*(l1+l2)])
    else :
        print("outside workspace")
    

def animate_xy(x,y,elbow,l1,l2):
    n = np.size(x)
    
    plt.show()
    
    for i in range(n):
        print(i)
        plot_endpt(x[i-1],y[i-1],elbow,l1,l2)
        plt.plot(x,y,'-g')
        plt.pause(0.0001)
        plt.clf()

def animate_q1q2(q1,q2,l1,l2):
    n = np.size(q1)
    plt.show()
    for i in range(n):
        print(i)
        plot_joint(q1[i],q2[i],l1,l2)
        plt.plot(x,y,'-g')
        plt.pause(0.0001)
        plt.clf()

def cubic(t,q1f,q2f,q10,q20):
    
    a10 = q10
    a11 = 0
    a12 = 3*(q1f-q10)/tf**2
    a13 = -2*(q1f-q10)/tf**3

    a20 = q20
    a21 = 0
    a22 = 3*(q2f-q20)/tf**2
    a23 = -2*(q2f-q20)/tf**3

    q1d = a10+a11*t+a12*t**2+a13*t**3
    q2d = a20+a21*t+a22*t**2+a23*t**3
    return q1d,q2d
#################################################
l1 = 1
l2 = 1
m1 = 1
m2 = 1

elbow = -1
tf = 1
t = np.linspace(0,tf,1001)
dt = tf/1001

### Task 1a without Dynamics ####

# Circular Trajectory
# x = 1.3*np.cos(100*t)
# y = 1.3*np.sin(100*t)

# cubic traj between x0 y0 and x1 y1
x0 = 0.5
y0 = 0.5
x1 = 1.3
y1 = 1.3
ws,q10,q20 = ik_2R(x0,y0,elbow,l1,l2)
ws,q1f,q2f = ik_2R(x1,y1,elbow,l1,l2)
q1d,q2d = cubic(t,q1f,q2f,q10,q20)
#Line
# x = 0.3+1*t
# y = 0.3+1*t

#Circular Trajectory
x = 0.7+0.3*np.cos(10*t)

y = 0.7+0.3*np.sin(10*t)

###
#n = np.size(x)
#animate_xy(x,y,elbow,l1,l2)
#animate_q1q2(q1d,q2d,l1,l2)
###  Task 1b with dynamics run ###



print("hi")
#Line
# x = 0.3+1*t
# y = 0.3+1*t
# xdot = -1.3*10*np.sin(10*t)
# ydot = 1.3*10*np.cos(10*t) 
# xddot = -1.3*10*10*np.cos(10*t)
# yddot = 1.3*10*10*np.sin(10*t)

q = ik_2R_array(x,y,elbow,l1,l2)


# q[:,0] = q1d.T    Uncomment for cubic
# q[:,1] = q2d.T
qdot = np.diff(q,axis=0)
qdot = np.vstack((qdot,qdot[-1]))/dt
qddot = np.diff(qdot,axis=0)
qddot = np.vstack((qddot,qddot[-1]))/dt
plt.plot(t,qdot)
# plt.plot(t,qdot)

t2 = np.linspace(0, tf, 1001)
y0 = np.round(np.array([q[0][0],q[0][1],qdot[0][0],qdot[0][1]]),3)
tau = invDi(qddot,qdot,q,l1,l2,m1,m2)
sol2 = rungekutta4(f, y0, t2, tau,l1,l2,m1,m2)
print(sol2)
q1 = sol2[:,0].T
q2 = sol2[:,1].T
fig1 = plt.figure()
plt.plot(t,q[:,0],label='q1d')
plt.plot(t,q[:,1],label='q2d')
plt.legend()
fig2 = plt.figure()
plt.plot(t,q1,label='q1')
plt.plot(t,q2,label='q2')
plt.legend()
plt.show()
animate_q1q2(q1,q2,l1,l2)

### Task 3 ###




