from render import Renderer
import cv2
import numpy as np
from scipy.optimize import fsolve
from scipy.integrate import ode


class RR(Renderer):
    def __init__(self):
        super().__init__()
        self.ode = ode(self.function).set_integrator('vode', nsteps=500, method='bdf')
        self.m1 = 1
        self.m2 = 1
        self.l1 = 100
        self.l2 = 100
        self.xe = 10
        self.ye = 110
        self.xi = 48
        self.yi = 52
        self.xc, self.yc = self.xi, self.yi
        self.q1 = (np.pi/2+np.pi/3)
        self.q2 = np.pi / 5
        self.tau1 = 0
        self.tau2 = 0
        self.K = 1
        self.q1_1 = 0
        self.q2_1 = 0
        self.g = 9.8
        self.q1_2 = 0
        self.q2_2 = 0
        self.ode.set_initial_value([self.q1, self.q1_1, self.q2, self.q2_1], 0)

    def function(self, t, y):
        q1, q1_1, q2, q2_1 = y

        coeff1 = -0.5 * self.m2 * self.l1 * self.l2 * q2_1 * (q2_1 - q1_1) * np.sin(q2 - q1) - 0.5 * self.m2 * self.l1 * self.l2 * q1_1 * q2_1 * np.sin(q2 - q1) + 0.5 * self.m1 * self.g * self.l1 * np.cos(q1) + self.m2 * self.g * self.l1 * np.cos(q1) - self.tau1
        coeff2 = -0.5 * self.m2 * self.l1 * self.l2 * q1_1 * (q2_1 - q1_1) * np.sin(q2 - q1) + 0.5 * self.m2 * self.l1 * self.l2 * q1_1 * q2_1 * np.sin(q2 - q1) + 0.5 * self.m2 * self.g * self.l2 * np.cos(q2) - self.tau2
        coeff3 = 1 / 3 * self.m1 * self.l1 ** 2 + self.m2 * self.l1 ** 2
        coeff4 = 0.5 * self.m2 * self.l1 * self.l2 * np.cos(q2 - q1)
        coeff5 = 0.5 * self.m2 * self.l1 * self.l2 * np.cos(q2 - q1)
        coeff6 = 1 / 3 * self.m2 * self.l1 ** 2 + 1 / 4 * self.m2 * self.l2 ** 2

        q2_2 = (coeff1 / coeff3 - coeff2 / coeff5) / (coeff6 / coeff5 - coeff4 / coeff3)
        q1_2 = -coeff1 / coeff3 - coeff4 * q2_2 / coeff3

        return [q1_1, q1_2, q2_1, q2_2]

    def update(self):
        '''theta = np.arccos((self.xc ** 2 + self.yc ** 2 - self.l1 ** 2 - self.l2 ** 2) / (2 * self.l1 * self.l2))
        self.q1 = np.arctan(self.yc / self.xc) - np.arctan(
            (self.l1 * np.sin(theta)) / (self.l2 * np.cos(theta) + self.l1))
        self.q2 = theta + self.q1'''

        fx = -self.K * (self.xc - self.xe)
        fy = -self.K * (self.yc - self.ye)

        self.tau1 = fy * self.l1 * np.cos(self.q1) - fx * self.l1 * np.sin(
            self.q1) + 0.5 * self.m2 * self.g * self.l2 * np.cos(self.q2)
        self.tau2 = fy * self.l2 * np.cos(self.q2) - fx * self.l2 * np.sin(
            self.q2) + 0.5 * self.m1 * self.g * self.l1 * np.cos(self.q1)

    def getInfo(self):
        info = {"x": round(self.xc, 3), "y": round(self.yc, 3)}
        return info

    def step(self, dt=0.01):
        self.update()
        self.q1, q1_1, self.q2, q2_1 = self.ode.integrate(self.ode.t + dt)
        self.xc = self.l1 * np.cos(self.q1) + self.l2 * np.cos(self.q2)
        self.yc = self.l1 * np.sin(self.q1) + self.l2 * np.sin(self.q2)

    def draw(self, image):
        line1 = (int(300 + self.l1 * np.cos(-self.q1)), int(300 + self.l1 * np.sin(-self.q1)))
        cv2.line(image, (300, 300), line1, (255, 0, 0), 1)
        line2 = (int(line1[0] + self.l2 * np.cos(-self.q2)), int(line1[1] + self.l2 * np.sin(-self.q2)))
        cv2.line(image, line1, line2, (0, 255, 0), 1)

        cv2.circle(image, (285 + int(self.xe), -int(self.ye) + 315), 2, (25, 25, 25), -1)

        return image


mybot = RR()
i = 0
while (True):
    mybot.step(0.01)
    print(mybot.q1, mybot.q2)
    mybot.render()
    i+=1
    if i>1000000:
        break
