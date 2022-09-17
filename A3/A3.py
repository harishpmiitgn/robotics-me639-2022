
import numpy as np
import matplotlib as plt

def homogenous_from_DH_row(DHrow): 
    d = DHrow[0]
    a = DHrow[1]
    alpha = DHrow[2]
    theta = DHrow[3]
    Atheta = np.array([[np.cos(theta),-np.sin(theta),0,0],[np.sin(theta),np.cos(theta),0,0],[0,0,1,0],[0,0,0,1]])
    Ad = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,d],[0,0,0,1]])
    Aa = np.array([[1,0,0,a],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    Aalpha = np.array([[1,0,0,0],[0,np.cos(alpha),-np.sin(alpha),0],[0,np.sin(alpha),np.cos(alpha),0],[0,0,0,1]])
    A = Atheta @ Ad @ Aa @ Aalpha
    return A

def HM_from_DHTable(DH,i):
    n=np.shape(DH)[0]
    A = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    for j in range(i):
        Aj = homogenous_from_DH_row(DH[j][:])
        A = A@Aj
    Ri = A[0:3,0:3]
    oi = A[3,0:3]
    # print(A)
    return A

def Jacobian(DH,type=[11]):
    n=np.shape(DH)[0]
    if type[0] == 11:
        notype = True
        print("since no type mentioned all joints assumed revolute")
    else:
        notype=False
    An = HM_from_DHTable(DH,n)
    on = An[0:3,3]
    o0 = np.array([0,0,0]).T
    z0 = np.array([0,0,1]).T
    Jv = np.zeros((3,n))
    Jw = np.zeros((3,n))
    if notype == True:
        Jv[:,0] = np.cross(z0,on-o0)
        Jw[:,0] = z0
    else:
        if type[0] == 0:
            Jv[:,0] = np.cross(z0,on-o0)
            Jw[:,0] = z0
        else:
            Jv[:,0] = z0
            Jw[:,0] = 0*z0
    for i in range(n-1):
        Ai = HM_from_DHTable(DH,i+1)
        zi = Ai[0:3,2]
        oi = Ai[0:3,3]
        if notype == False:
            if type[i+1] == 0:
                Jvi = np.cross(zi,on-oi)
                Jwi = zi
            else:
                Jvi = zi
                Jwi = np.array([0,0,0]).T

        else:
            Jvi = np.cross(zi,on-oi)
            Jwi = zi

        Jv[:,i+1] = Jvi
        Jw[:,i+1] = Jwi
    J = np.vstack((Jv,Jw))
    notype = False
    return J

### Results:

## No need to mention number of joints that can be inferred from no of rows in DH table
DH = np.array([[0,1,0,0],[0,1,np.radians(180),0],[0,0,0,0],[1,0,0,0]]) 
DH3SCARA = np.array([[0,1,0,0],[0,1,0,0],[0,0,np.radians(-180),0]]) # RRP SCARA
DH3STANFORD = np.array([[0,0,np.radians(-90),0],[1,0,np.radians(90),0],[1,0,0,0]])
Q37 = np.array([[1, 0, np.radians(90), 0 ],[1, 0, np.radians(-90), np.radians(90)],[1,0,0,0]])
Q38 = np.array([[1,0,np.radians(-90),0],[0,1,0,0],[0,1,0,0],[0,0,np.radians(90),0],[0,0,np.radians(90),0],[1,0,0,0]])
DH2 = np.array([[0,1,0,0],[0,1,0,0]]) # 2R
# 3a complete jacobian
print("Q3a")
jSC = Jacobian(DH3SCARA,[0,0,1]) # DH order [d,a,alpha,theta)] Type(nx1 array): 0 for Revolute 1 for prismatic
jST = Jacobian(DH3STANFORD,[0,0,1])
print("jST")
print(jST)
print("jSC")
print(jSC)
# 3b
print("Q 3b")
HSC = HM_from_DHTable(DH3SCARA,3)
HST = HM_from_DHTable(DH3STANFORD,3)
print("SCARA_endeffector_pos")
print(HSC[0:3,3])
print("STANFORD_endeffector_pos")
print(HST[0:3,3])
# 3c
print("Q 3c endeffector velocity")
qdot = np.array([[0],[0],[1]])
vscara = np.dot(jSC,qdot)
print("vscara",vscara)
vstanford = np.dot(jST,qdot)
print("vstanford",vstanford)
print("Q5 and Q6")
HQ37 = HM_from_DHTable(Q37,3)
HQ38 = HM_from_DHTable(Q38,6)
print("HQ37")
print(HQ37)
print("HQ38")
print(HQ38)
