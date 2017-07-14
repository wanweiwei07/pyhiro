# see Model Globally, Match Locally: Efficient and Robust 3D Object Recognition, CVPR2010
# implemented by weiwei, 20170714, tsukuba

import os

import numpy as np
from pykinect2 import PyKinectRuntime
from pykinect2 import PyKinectV2
from sklearn.cluster import DBSCAN

import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pg
import icp
from vision import CADTemp, icp
import tools
from utils import robotmath as rm
import math

class PPFMatch(object):

    def __init__(self, ompath, npntsonverts = 200):
        """

        :param ompath: path of the mesh template

        author: weiwei
        date: 20170711
        """

        cadtemp = CADTemp.CADTemp(ompath = ompath, numpointsoververts = npntsonverts)

        self.objnp = pg.packpandanp(cadtemp.objtrimesh.vertices,
                               cadtemp.objtrimesh.face_normals,
                               cadtemp.objtrimesh.faces,
                               name='')
        self.temppnt = cadtemp.pcdtemp
        self.normals = tools.estimatenormals(self.temppnt, npoints = 10)

    def computePPFwithAlpha(self, ddist = 5.0, dangle = 2*math.pi/12.0):
        """
        compute the point pair feature f1, f2, f3, f4,
        and the alpha_m

        :return: a dictionary

        author: weiwei
        date: 20170714
        """

        # global model descriptor, gmd
        gmd = {}

        ntemppoint = self.temppnt.shape[0]
        for i in range(ntemppoint):
            for j in range(ntemppoint):
                m_0 = np.asrray(self.temppnt[i])
                m_1 = np.asrray(self.temppnt[j])
                v_m0m1 = m_0-m_1
                v_m1m0 = m_1-m_0
                n_m0 = self.normals[i]
                n_m1 = self.normals[j]
                # f1, namely ||d||2
                f1 = np.linalg.norm(m_0-m_1)
                # f2, namely angle between n_m0 and v_m1m0
                f2 = rm.angle_between(n_m0, v_m1m0)
                # f3, namely angle between n_m1 and v_m0m1
                f3 = rm.angle_between(n_m1, v_m0m1)
                # f4, namely angle between n_m0 and n_m1
                f4 = rm.angle_between(n_m0, n_m1)
                # discretize the values
                f1d = math.floor(f1/ddist)*ddist+ddist
                f2d = math.floor(f2/dangle)*dangle+dangle
                f3d = math.floor(f3/dangle)*dangle+dangle
                f4d = math.floor(f4/dangle)*dangle+dangle
                key = (f1d, f2d, f3d, f4d)
                if key in gmd.keys():
                    gmd[key].append([m_0, m_1])
                else:
                    gmd[key] = [[m_0, m_1]]


if __name__=='__main__':

    base = pandactrl.World(camp=[0,0,300], lookatp=[0,0,0])

    this_dir, this_filename = os.path.split(__file__)
    objpath = os.path.join(this_dir, "models", "ttube.stl")

    caliber = Calibrate(objpath)