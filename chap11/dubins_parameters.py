# dubins_parameters
#   - Dubins parameters that define path between two configurations
#
# mavsim_matlab 
#     - Beard & McLain, PUP, 2012
#     - Update history:  
#         3/26/2019 - RWB
#         4/2/2020 - RWB

import numpy as np
import sys
sys.path.append('..')


class DubinsParameters:
    def __init__(self, ps=9999*np.ones((3,1)), chis=9999,
                 pe=9999*np.ones((3,1)), chie=9999, R=9999):
        if R == 9999:
            L = R
            cs = ps
            lams = R
            ce = ps
            lame = R
            w1 = ps
            q1 = ps
            w2 = ps
            w3 = ps
            q3 = ps
        else:
            L, cs, lams, ce, lame, w1, q1, w2, w3, q3 \
                = compute_parameters(ps, chis, pe, chie, R)
        self.p_s = ps
        self.chi_s = chis
        self.p_e = pe
        self.chi_e = chie
        self.radius = R
        self.length = L
        self.center_s = cs
        self.dir_s = lams
        self.center_e = ce
        self.dir_e = lame
        self.r1 = w1
        self.n1 = q1
        self.r2 = w2
        self.r3 = w3
        self.n3 = q3

    def update(self, ps, chis, pe, chie, R):
         L, cs, lams, ce, lame, w1, q1, w2, w3, q3 \
            = compute_parameters(ps, chis, pe, chie, R)
         self.p_s = ps
         self.chi_s = chis
         self.p_e = pe
         self.chi_e = chie
         self.radius = R
         self.length = L
         self.center_s = cs
         self.dir_s = lams
         self.center_e = ce
         self.dir_e = lame
         self.r1 = w1
         self.n1 = q1
         self.r2 = w2
         self.r3 = w3
         self.n3 = q3


def compute_parameters(ps, chis, pe, chie, R):
    ell = np.linalg.norm(ps-pe)
    if ell < 2 * R:
        print('Error in Dubins Parameters: The distance between nodes must be larger than 2R.')
    else:
        # compute start and end circles

        crs = ps + R*rotz(chis + np.pi/2)[0]
        cls = ps + R*rotz(chis - np.pi/2)[0]
        cre = pe + R*rotz(chie + np.pi/2)[0]
        cle = pe + R*rotz(chie - np.pi/2)[0]

        # compute L1
        theta = np.arctan2((cre.item(1) - crs.item(1)), (cre.item(0) - crs.item(0)))
        L1 = np.linalg.norm(crs - cre) + R*(2*np.pi + (theta - np.pi/2) - (chis - np.pi/2)) + R*(2*np.pi + (chie - np.pi/2) - (theta - np.pi/2))
        # compute L2

        ell = np.linalg.norm(crs - cle)
        theta = np.arctan2((cre.item(1) - crs.item(1)), (cre.item(0) - crs.item(0)))
        theta2 = 0
        if not np.isreal(theta2):
            L2 = 0
        else:
            L2 = 0

        # compute L3
        ell = 0
        theta = 0 
        theta2 = 0 
        if not np.isreal(theta2):
            L3 = 0
        else:
            L3 = 0
        # compute L4
        theta = 0
        L4 = 0
        # L is the minimum distance
        L = np.min([L1, L2, L3, L4])
        idx = np.argmin([L1, L2, L3, L4])
        if idx == 0:
            cs = 0
            lams = 0
            ce = 0
            lame = 0
            q1 = 0
            w1 = 0
            w2 = 0
        elif idx == 1:
            cs = 0
            lams = 0
            ce = 0
            lame = 0
            q1 = 0
            w1 = 0
            w2 =  0
        elif idx == 2:
            cs = 0
            lams = 0
            ce = 0
            lame = 0
            q1 = 0
            w1 = 0
            w2 = 0
        elif idx == 3:
            cs = 0
            lams = 0
            ce = 0
            lame = 0
            q1 = 0
            w1 = 0
            w2 =  0
        w3 = 0
        q3 = 0

        return L, cs, lams, ce, lame, w1, q1, w2, w3, q3


def rotz(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1]])


def mod(x):
    while x < 0:
        x += 2*np.pi
    while x > 2*np.pi:
        x -= 2*np.pi
    return x


