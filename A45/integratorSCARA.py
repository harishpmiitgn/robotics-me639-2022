
import roboticstoolbox as rtb
from spatialmath import SE3
import cv2
from mpl_toolkits import mplot3d
from matplotlib import pyplot as plt
import spatialmath.base as base
import numpy as np

def modelscara(q, t, i, I1, I2, l1, l2, l3, lc1, lc2, lc3, m1, m2, K1, K2, K3,q1_poly,q2_poly,q3_poly):

    dt = 0.01 # step size change accordingly
    t1 = q[0]
    t2 = q[1]
    t3 = q[2]
    #joint rates
    dt1 = q[3]
    dt2 = q[4]
    dt3 = q[5]

    # cubic desired trajectory
    qd1 = q1_poly[0]+q1_poly[1]*t+q1_poly[2]*t**2+q1_poly[3]*t**3
    qd2 = q2_poly[0] + q2_poly[1] * t + q2_poly[2] * t ** 2 + q2_poly[3] * t ** 3
    qd3 = q3_poly[0] + q3_poly[1] * t + q3_poly[2] * t ** 2 + q3_poly[3] * t ** 3
    # desired velocity
    dqd1 = q1_poly[1]  + 2*q1_poly[2] * t+ 3*q1_poly[3] * t **2
    dqd2 = q2_poly[1]  + 2*q2_poly[2] * t+ 3*q2_poly[3] * t **2
    dqd3 = q3_poly[1]  + 2*q3_poly[2] * t + 3*q3_poly[3] * t **2
    # desired acceleration
    ddqd1 =  2*q1_poly[2]+ 6*q1_poly[3] * t
    ddqd2 =  2*q2_poly[2]+ 6*q2_poly[3] * t
    ddqd3 =  2*q3_poly[2] + 6*q3_poly[3] * t

    # desired accl vector
    tdd = np.array([[ddqd1],[ddqd2],[ddqd2]])

    tdd = np.reshape(tdd,(3,1))
    # actual velocity vector
    td = np.array([[dt1],[dt2],[dt3]])

    tau = np.transpose(np.zeros(3))
    # Define Robot using PeterCorke toolbox just substitute non variable DH parameters Inertia and mass
    robot1 = rtb.DHRobot(
        [
            rtb.RevoluteDH(alpha=0, a=0.5, d=0.2, I=np.array([[0,0,0],[0,0,0],[0,0,1/12]]), m=1),
            rtb.RevoluteDH(alpha=np.pi, a=0.5, d=0, I=np.array([[0,0,0],[0,0,0],[0,0,1/12]]), m=1),
            rtb.PrismaticDH(theta=0, alpha=0, a=0, I=np.array([[0,0,0],[0,0,0],[0,0,1/12]]), m=1),
        ], name="scararob", gravity=[0, 0, -9.81])

    # Get D,C,phi
    D = robot1.inertia([t1,t2,t3])
    C = robot1.coriolis([t1,t2,t3],[dt1,dt2,dt3])
    phi = robot1.gravload([t1,t2,t3])
    phi = np.reshape(phi, (3, 1))

    #diagonal terms D to dkk
    dkk = np.array([[D[0][0], 0, 0],[0, D[1][1],0],[0, 0,D[2][2]]])
    # modelling disturbance, you can more disturbances here
    dk =   np.matmul(C, td) +np.matmul((D-dkk), tdd) + phi

    K = np.array([[K1,0,0],[0,K2,0],[0,0,K3]])/dt

    Kinv = np.linalg.inv(K)

    # compute errors
    e = np.array([[qd1-q[0]],[qd2-q[1]],[qd3-q[2]]])
    e = np.reshape(e,(3,1))
    ed = np.array([[0 - q[3]], [0 - q[4]], [0 - q[5]]])
    ed = np.reshape(e, (3, 1))

    Kp = np.array([[200, 0, 0],[0,200,0],[0,0,1000]])
    Kd =  np.array([[0.1, 0, 0],[0,0.1,0],[0,0,20]])
    Ki = np.array([[0.01, 0, 0],[0,0.01,0],[0,0,0]])
    dt = 0.01
    if i == 0:
        tprev = 0
        dt = t - tprev
    else:
        dt = t - tprev
    # disp(size(dt));
    if i ==0:
        sum =  np.matmul(Ki, e) * dt
    else:
        sum += sum + np.matmul(Ki, e) * dt

    V = np.matmul(Kp,e) + np.matmul(Kd,ed) + sum # voltage control input
    print(e)
    # dynamic equation in joint varaibles
    # ddq = np.matmul(np.linalg.inv(Jeff),(r1*np.matmul(K ,V) - (r1**2)*(dk)- np.matmul(Beff , td))) # with motor dynamics
    qdot = np.array([[q[3]],[q[4]],[q[5]]])
    ddq = np.matmul(np.linalg.inv(D),(V - (np.matmul(C, qdot))-phi)) 
    # find statedot
    dq = [q[3],q[4],q[5],ddq[0][0],ddq[1][0],ddq[2][0]]

    print('t= '+str(t))
    tprev = t   # used for Integral
    i = i+1
    return dq



