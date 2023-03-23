import sys
#sys.path.append('/home/tumu/anaconda3/pkgs')
import numpy as np
import math
import PyKDL
import matplotlib.pyplot as plt

from kinematics import KINEMATICS

def main():
    #jointLimitLow = np.array([-math.pi/2, 0.00, -2.96, -math.pi, -2.9, 0.00, -2.9])
    #jointLimitHigh = np.array([math.pi/2, math.pi/2, 2.96, 0.00, 2.9, 3.8, 2.9]) 
    jointLimitLow = np.array([-math.pi/2, 0.00, 0.00, -1.85, 0.00, 2.26,0.79])
    jointLimitHigh = np.array([math.pi/2, math.pi/2, 0.00, -1.85, 0.00, 2.26,0.79])

    kinematics = KINEMATICS('panda.urdf')
    xPoints = []
    yPoints = []
    zPoints = []
    for i in range(5000):
        goal = np.empty(3)
        sampledAngles = np.random.uniform(jointLimitLow, jointLimitHigh)
        #print("sampled Angles:",sampledAngles)
        q_in = PyKDL.JntArray(kinematics.numbOfJoints)
        q_in[0], q_in[1], q_in[2], q_in[3] =sampledAngles[0], sampledAngles[1], sampledAngles[2], sampledAngles[3]
        q_in[4], q_in[5], q_in[6] = sampledAngles[4], sampledAngles[5], sampledAngles[6]
        goalFrame = kinematics.forwardKinematicsPoseSolv(q_in)

        xPoints.append(goalFrame.p[0])
        yPoints.append(goalFrame.p[1])
        zPoints.append(goalFrame.p[2])

    plt.subplot(1,2,1)
    plt.plot(xPoints, yPoints, 'o')
    plt.subplot(1,2,2)
    plt.plot(xPoints, zPoints, 'o')
    plt.show()

if __name__ == '__main__':
    main()
