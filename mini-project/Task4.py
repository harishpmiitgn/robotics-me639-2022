from render import Renderer
import cv2
import numpy as np
from scipy.optimize import fsolve

class RR(Renderer):
    def __init__(self):
        super(RR, self).__init__()
        self.min_angle = (7/36)*np.pi
        self.max_angle = (29/36)*np.pi
        self.l1 = 100
        self.l2 = 100
        self.xc = 0
        self.yc = 0
        self.q1 = (7/36)*np.pi
        self.q2 = (7/36)*np.pi
        self.info = {}
        self.path = []
        self.done = False

    def step(self, freq = 5):
        for i in range(int((145 - 35) / freq)):
            self.q1 += np.pi * freq / 180
            for j in range(int((145 - 35) / freq)):
                self.q2 = (7/36)*np.pi
                self.q2 += j * np.pi * freq / 180
                #self.xc = self.l1 * np.cos(self.q1) + self.l2 * np.cos(self.q2)
                #self.yc = self.l1 * np.sin(self.q1) + self.l2 * np.sin(self.q2)
                #self.path.append((self.xc, self.yc))
                self.info = {"q1: ": self.q1*180/np.pi, "q2: ":self.q2*180/np.pi}
                self.render()
        self.done = True

    def getInfo(self):
        return self.info

    def draw(self, image):
        line1 = (int(300 + self.l1 * np.cos(-self.q1)), int(300 + self.l1 * np.sin(-self.q1)))
        cv2.line(image, (300, 300), line1, (255, 0, 0), 1)
        line2 = (int(line1[0] + self.l2 * np.cos(-self.q2)), int(line1[1] + self.l2 * np.sin(-self.q2)))
        cv2.line(image, line1, line2, (0, 255, 0), 1)
        self.path.append(line2)
        for i in self.path:
            cv2.circle(image, i, 1, (0, 0, 0), 1)
        return image

mybot=RR()
while(True):
    cv2.namedWindow("window")
    mybot.step(5)
    mybot.render()
    if (mybot.done):
        cv2.waitKey(0)
        break