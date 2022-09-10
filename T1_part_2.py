import numpy as np
import math
import matplotlib.pyplot as plt
from sympy import *
x,y,t,theta_fn=symbols('x y t theta_fn')

L1=3 #m for first robotic arm
L2=2.5 #m for second robotic arm
m1=0.2 
m2=0.3
g=9.81

theta_fn=acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2))
q1_fn=atan(y/x)-atan((L2*sin(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2))))/(L1+L2*cos(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2)))))
dq1_fn_dt=diff(atan(y/x)-atan((L2*sin(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2))))/(L1+L2*cos(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2))))),x)*diff(t,t)+diff(atan(y/x)-atan((L2*sin(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2))))/(L1+L2*cos(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2))))),y)*diff(sin(t),t)
#q2_fn=q1_fn+theta_fn
q2_fn=atan(y/x)-atan((L2*sin(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2))))/(L1+L2*cos(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2)))))+acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2))
dq2_fn_dt=diff(atan(y/x)-atan((L2*sin(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2))))/(L1+L2*cos(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2)))))+acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2)),x)*diff(t,t)+diff(atan(y/x)-atan((L2*sin(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2))))/(L1+L2*cos(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2)))))+acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2)),y)*diff(sin(t),t)
#print(dq1_fn_dt)
#print(dq2_fn_dt)
#d2q1_fn_dt2=diff(dq1_fn_dt,x)*diff(x,t)+diff(dq1_fn_dt,t)*diff(y,t)
d2q1_fn_dt2=diff(diff(atan(y/x)-atan((L2*sin(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2))))/(L1+L2*cos(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2))))),x)*diff(t,t)+diff(atan(y/x)-atan((L2*sin(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2))))/(L1+L2*cos(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2))))),y)*diff(sin(t),t),x)*diff(t,t)+diff(diff(atan(y/x)-atan((L2*sin(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2))))/(L1+L2*cos(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2))))),x)*diff(t,t)+diff(atan(y/x)-atan((L2*sin(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2))))/(L1+L2*cos(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2))))),y)*diff(sin(t),t),y)*diff(sin(t),t)
#d2q2_fn_dt2=diff(dq2_fn_dt,x)*diff(x,t)+diff(dq2_fn_dt,t)*diff(y,t)
d2q2_fn_dt2=diff(diff(atan(y/x)-atan((L2*sin(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2))))/(L1+L2*cos(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2)))))+acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2)),x)*diff(t,t)+diff(atan(y/x)-atan((L2*sin(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2))))/(L1+L2*cos(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2)))))+acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2)),y)*diff(sin(t),t),x)*diff(t,t)+diff(diff(atan(y/x)-atan((L2*sin(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2))))/(L1+L2*cos(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2)))))+acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2)),x)*diff(t,t)+diff(atan(y/x)-atan((L2*sin(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2))))/(L1+L2*cos(acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2)))))+acos((x**2+y**2-L1**2-L2**2)/(2*L1*L2)),y)*diff(sin(t),t),y)*diff(sin(t),t)
#print(d2q1_fn_dt2)

N_x=5 #no of divisions in x for simulation

x_start=1
x_end=6

x0=0
y0=0
theta=0

for n in range(x_start,x_end+1,1):
    y_fn=math.sin(n)
    theta=math.acos((n**2+y_fn**2-L1**2-L2**2)/(2*L1*L2))
    q1=math.atan(y_fn/n)-math.atan((L2*math.sin(theta))/(L1+L2*math.cos(theta)))
    q2=q1+theta
    #T1 and T2 are the torques in each iteration
    T1=(1/3)*m1*(L1**2)*(d2q1_fn_dt2.subs([(x,n),(y,y_fn)]))+m2*(L1**2)*(d2q1_fn_dt2.subs([(x,n),(y,y_fn)]))+(1/2)*m2*L1*L2*(d2q2_fn_dt2.subs([(x,n),(y,y_fn)]))*math.cos(q2_fn.subs([(x,n),(y,y_fn)])-q1_fn.subs([(x,n),(y,y_fn)]))+(1/2)*L1*L2*m2*dq2_fn_dt.subs([(x,n),(y,y_fn)])*(dq2_fn_dt.subs([(x,n),(y,y_fn)])-dq1_fn_dt.subs([(x,n),(y,y_fn)]))*math.sin(q2_fn.subs([(x,n),(y,y_fn)])-q1_fn.subs([(x,n),(y,y_fn)]))+(1/2)*m1*g*L1*math.cos(q1)+m2*g*L1*math.cos(q1)
    T2=(1/3)*m2*(L2**2)*(d2q2_fn_dt2.subs([(x,n),(y,y_fn)]))+(1/4)*m2*(L2**2)*(d2q2_fn_dt2.subs([(x,n),(y,y_fn)]))+(1/2)*m2*L1*L2*(d2q1_fn_dt2.subs([(x,n),(y,y_fn)]))*math.cos(q2_fn.subs([(x,n),(y,y_fn)])-q1_fn.subs([(x,n),(y,y_fn)]))+(1/2)*L1*L2*m2*dq1_fn_dt.subs([(x,n),(y,y_fn)])*(dq2_fn_dt.subs([(x,n),(y,y_fn)])-dq1_fn_dt.subs([(x,n),(y,y_fn)]))*math.sin(q2_fn.subs([(x,n),(y,y_fn)])-q1_fn.subs([(x,n),(y,y_fn)]))+(1/2)*m2*g*L2*math.sin(q1)
    print(T1)
    #print(T2)


