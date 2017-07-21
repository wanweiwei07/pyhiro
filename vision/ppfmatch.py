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
import kinectinterface as kif
from panda3d.core import *

class PPFMatch(object):

    def __init__(self, ompath):
        """

        :param ompath: path of the mesh template

        author: weiwei
        date: 20170711
        """

        cadtemp = CADTemp.CADTemp(ompath = ompath)

        self.objnp = pg.packpandanp(cadtemp.objtrimesh.vertices,
                               cadtemp.objtrimesh.face_normals,
                               cadtemp.objtrimesh.faces,
                               name='')
        self.temppnts = cadtemp.pcdtempnoverts
        self.tempnormals = cadtemp.pcdtempnovertsnormals
        self.kif = kif.KinectInterface()

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

        ntemppoint = self.temppnts.shape[0]
        for i in range(ntemppoint):
            print i, ntemppoint
            for j in range(ntemppoint):
        # for i in range(0,1):
        #     for j in range(3,4):
                m_0 = np.asarray(self.temppnts[i])
                m_1 = np.asarray(self.temppnts[j])
                v_m0m1 = m_0-m_1
                v_m1m0 = m_1-m_0
                n_m0 = self.tempnormals[i]
                n_m1 = self.tempnormals[j]
                # f1, namely ||d||2
                f1 = np.linalg.norm(m_0-m_1)
                # f2, namely angle between n_m0 and v_m1m0
                f2 = rm.radian_between(n_m0, v_m1m0)
                # f3, namely angle between n_m1 and v_m0m1
                f3 = rm.radian_between(n_m1, v_m0m1)
                # f4, namely angle between n_m0 and n_m1
                f4 = rm.radian_between(n_m0, n_m1)
                # discretize the values
                f1d = math.floor(f1/ddist)*ddist+ddist
                f2d = math.floor(f2/dangle)*dangle+dangle
                f3d = math.floor(f3/dangle)*dangle+dangle
                f4d = math.floor(f4/dangle)*dangle+dangle
                key = (f1d, f2d, f3d, f4d)
                # angle between n_m0 and x+
                xplus = np.asarray([1,0,0])
                yplus = np.asarray([0,1,0])
                nm0xangle = math.degrees(rm.radian_between(n_m0, xplus))
                rotax = np.cross(xplus, n_m0)
                if np.isnan(rotax).any() or not rotax.any():
                    continue
                rotmat = rm.rodrigues(rotax, nm0xangle)
                v_m1m0onxplus = np.dot(v_m1m0, rotmat)
                v_m1m0onxplusyzproj = np.asarray([0, v_m1m0onxplus[1], v_m1m0onxplus[2]])
                alpha_m0 = rm.radian_between(v_m1m0onxplusyzproj, yplus)
                if v_m1m0onxplus[2] < 0:
                    alpha_m0 = 2*math.pi - alpha_m0
                # debug
                # before transform
                pg.plotArrow(base.render, spos = m_0, epos = m_1, rgba=Vec4(0,1,0,1))
                pg.plotArrow(base.render, spos = m_0, epos = m_0+n_m0, rgba = Vec4(1,0,0,1))
                # after transform
                # print v_m1m0onxplus
                # print v_m1m0onxplusyzproj
                pg.plotArrow(base.render, spos = m_0, epos = v_m1m0onxplus+m_0, rgba=Vec4(0,.7,.7,1))
                pg.plotArrow(base.render, spos = m_0, epos = v_m1m0onxplusyzproj+m_0, rgba=Vec4(.70,.7,.7,1))
                pg.plotArrow(base.render, spos = m_0, epos = m_0+xplus, rgba = Vec4(.7,0,.7,1))
                # alpha_m0
                # print np.degrees(alpha_m0)
                # plot aixs
                zplus = np.asarray([0,0,1])
                pg.plotArrow(base.render, spos = m_0, epos = m_0+xplus*10, rgba = Vec4(.3,0,0,.3))
                pg.plotArrow(base.render, spos = m_0, epos = m_0+yplus*10, rgba = Vec4(0,.3,0,.3))
                pg.plotArrow(base.render, spos = m_0, epos = m_0+zplus*10, rgba = Vec4(0,0,.3,.3))

                if key in gmd.keys():
                    gmd[key].append([m_0, m_1, alpha_m0])
                else:
                    gmd[key] = [[m_0, m_1, alpha_m0]]

    def newmatch(self):
        """
        capture a new image and match
        :return:
        """

        objectpnts = self.kif.getObjectPcd()
        # normals
        objectnormals = tools.estimatenormals(objectpnts)


if __name__=='__main__':

    base = pandactrl.World(camp=[0,0,700], lookatp=[0,0,0])

    this_dir, this_filename = os.path.split(__file__)
    objpath = os.path.join(this_dir, "models", "ttube.stl")

    ppfmatch = PPFMatch(objpath)
    # ppfmatch.computePPFwithAlpha()

    colors = []
    color = [np.random.rand(), np.random.rand(), np.random.rand(), np.random.rand()]
    for pnt in ppfmatch.temppnts:
        colors.append(color)
    pntsnp = pg.genPntsnp(ppfmatch.temppnts, colors=colors)
    for i, normal in enumerate(ppfmatch.tempnormals):
        pg.plotArrow(base.render, spos = ppfmatch.temppnts[i], epos = normal+ppfmatch.temppnts[i], thickness = .2, length =2)

    # pg.plotAxisSelf(base.render)
    base.run()