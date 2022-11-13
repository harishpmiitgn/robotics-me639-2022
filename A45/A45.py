import numpy as np

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

def IK_SCARA(x,y,z,a1,a2,solno):
    D = (x**2+y**2-a1**2-a2**2)/(2*a1*a2)
    theta21  = np.arctan2(np.sqrt(1-D**2),D)
    theta22 = np.arctan2(-np.sqrt(1-D**2),D)
    theta11 = np.arctan2(y,x)-np.arctan2(a2*np.sin(theta21),(a1+a2*np.cos(theta21)))
    theta12 = np.arctan2(y,x)-np.arctan2(a2*np.sin(theta22),(a1+a2*np.cos(theta22)))
    d = z
    if solno == 1:
        return [theta11, theta21, d]
    elif solno ==2:
        return [theta11, theta21, d]

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

def JvelfromCvel(cvel,DH):
    J = Jacobian(DH,type = [0,0,1])
    Jv = J[0:3][0:]
    print(Jv)
    Jinv = np.linalg.inv(Jv)
    Jvel=np.dot(Jinv,cvel)
    return Jvel

def IK_Spherical_Robot(px,py,pz,d1,a2):
    theta1 = np.arctan2(px,py)
    r = np.sqrt(px**2+py**2)
    s = pz-d1
    theta2 = np.arctan2(r,s)
    d3 = np.sqrt(r**2+s**2)-a2
    return [theta1,theta2,d3]

def IK_Spherical_Wrist(M,choice=1):
    u33 = M[2][2]
    u13 = M[0][2]
    u23 = M[1][2]
    u31 = M[2][1]
    u32 = M[2][1]
    # for solution 2 enter choice =-1
    theta = np.arctan2(u33,choice*np.sqrt(1-u33**2))
    phi = np.arctan2(u13,u23)
    psi = np.arctan2(-u31,u32)
    return [theta,phi,psi]

# def IK_Spherical_Wrist():
#     theta = np.arctan2()

# Q1
# stanford was replaced with spherical by Sir
print("spherical IK")
print(IK_Spherical_Robot(np.sqrt(2),np.sqrt(2),1,1,1))

# Q2
theta1 = 0
theta2 = 0
d = 0
DH3SCARA = np.array([[0,1,0,theta1],[0,1,0,theta2],[0,0,np.radians(-180),d]]) # RRP SCARA
HSC = HM_from_DHTable(DH3SCARA,3)
print("SCARA_endeffector_pos")
print(HSC[0:3,3])
[x,y,z]=HSC[0:3,3] # forward kinematics
print("joint values are")
print(IK_SCARA(x,y,z,1,1,1))   # inverse kinematics verified

# Q3 find Joint velocity from cartesian vel, example scara
DH3SCARA = np.array([[0,1,0,np.pi/4],[0,1,0,np.pi/4],[0,0,np.radians(-180),0]]) # give non singular
cvel = np.array([[0],[0],[1]])
Jvel = JvelfromCvel(cvel,DH3SCARA)
print("Joint velocity is")
print(Jvel)

# Q6 spherical wrist ik
print("wrist")
M = np.array([[1, 0, 0],[0,1,0],[0,0,1]])
wrist = IK_Spherical_Wrist(M,choice=1)
print(wrist)





