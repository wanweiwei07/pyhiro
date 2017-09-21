# see Model Globally, Match Locally: Efficient and Robust 3D Object Recognition, CVPR2010
# implemented by weiwei, 20170714, tsukuba

import os

import operator
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
import random
from panda3d.core import *
import pickle
from plyfile import PlyData, PlyElement

class PPFMatch(object):

    def __init__(self, ompath, ppfpath = None, bsave = False):
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
        self.temppnts = cadtemp.pcdtempnovertsinner
        self.tempnormals = cadtemp.pcdtempnovertsnormalsinner
        self.kif = kif.KinectInterface()

        # global model descriptor, gmd
        if ppfpath:
            self.gmd = self.loadGMD(ppfpath)
            if bsave is True:
                bsave = False
                print "Warning: bsave must be false to load ppfpath"
        else:
            self.gmd = self.computePPFwithAlpha(self.temppnts, self.tempnormals)
        if bsave:
            pickle.dump(self.gmd, open("tmp.pickle", mode="wb"))

    def loadGMD(self, ppfpath):
        return pickle.load(open(ppfpath, mode = "rb"))

    def computePPFwithAlpha(self, points, normals, ddist = 5.0, dangle = 30.0):
        """
        compute the point pair feature f1, f2, f3, f4,
        and the alpha_m
        dangle in degree

        :return: a dictionary

        author: weiwei
        date: 20170714
        """

        gmd = {}

        ntemppoint = points.shape[0]
        for i in range(ntemppoint):
            print i, ntemppoint
            for j in range(ntemppoint):
        # for i in range(0,1):
        #     for j in range(3,4):
                m_0 = np.asarray(points[i])
                m_1 = np.asarray(points[j])
                v_m0m1 = m_0-m_1
                v_m1m0 = m_1-m_0
                n_m0 = normals[i]
                n_m1 = normals[j]
                # f1, namely ||d||2
                f1 = np.linalg.norm(m_0-m_1)
                # f2, namely angle between n_m0 and v_m1m0
                f2 = rm.degree_between(n_m0, v_m1m0)
                # f3, namely angle between n_m1 and v_m0m1
                f3 = rm.degree_between(n_m1, v_m0m1)
                # f4, namely angle between n_m0 and n_m1
                f4 = rm.degree_between(n_m0, n_m1)
                # discretize the values
                try:
                    f1d = int(math.floor(f1/ddist)*ddist+ddist)
                    f2d = int(math.floor(f2/dangle)*dangle+dangle)
                    f3d = int(math.floor(f3/dangle)*dangle+dangle)
                    f4d = int(math.floor(f4/dangle)*dangle+dangle)
                except:
                    continue
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
                # # debug
                # # before transform
                # pg.plotArrow(base.render, spos = m_0, epos = m_1, rgba=Vec4(0,1,0,1))
                # pg.plotArrow(base.render, spos = m_0, epos = m_0+n_m0, rgba = Vec4(1,0,0,1))
                # # after transform
                # # print v_m1m0onxplus
                # # print v_m1m0onxplusyzproj
                # pg.plotArrow(base.render, spos = m_0, epos = v_m1m0onxplus+m_0, rgba=Vec4(0,.7,.7,1))
                # pg.plotArrow(base.render, spos = m_0, epos = v_m1m0onxplusyzproj+m_0, rgba=Vec4(.70,.7,.7,1))
                # pg.plotArrow(base.render, spos = m_0, epos = m_0+xplus, rgba = Vec4(.7,0,.7,1))
                # # alpha_m0
                # print np.degrees(alpha_m0)
                # # plot aixs
                # zplus = np.asarray([0,0,1])
                # pg.plotArrow(base.render, spos = m_0, epos = m_0+xplus*10, rgba = Vec4(.3,0,0,.3))
                # pg.plotArrow(base.render, spos = m_0, epos = m_0+yplus*10, rgba = Vec4(0,.3,0,.3))
                # pg.plotArrow(base.render, spos = m_0, epos = m_0+zplus*10, rgba = Vec4(0,0,.3,.3))

                if key in gmd.keys():
                    gmd[key].append([i, j, alpha_m0])
                else:
                    gmd[key] = [[i, j, alpha_m0]]
        for key in gmd.keys():
            print key
        return gmd

    def perceivePoints(self, nframe = 5):
        """
        capture a new 3d point cloud
        :return:

        author: weiwei
        date:20170802
        """

        objectpnts = self.kif.getObjectPcd(nframe = nframe)
        # normals
        objectnormals = tools.estimatenormals(objectpnts)
        return objectpnts, objectnormals


    def match(self, perceivedpnts, perceivednormals, ddist = 5.0, dangle = 30.0):
        """
        do a match
        :return: rotmats of top matches

        author: weiwei
        date: 20170802
        """

        # save txt
        pverts = np.array([tuple(x) for x in perceivedpnts], dtype=[('x', 'f4'),('y', 'f4'),('z', 'f4')])
        el = PlyElement.describe(pverts, 'vertex')
        PlyData([el], text = True).write('perceivedpnts.ply')
        # save txt
        vandnarray = []
        for i in range(len(self.temppnts)):
            v = self.temppnts[i]
            n = self.tempnormals[i]
            vandn = (v[0],v[1],v[2],n[0],n[1],n[2])
            vandnarray.append(vandn)
        pverts = np.array(vandnarray, dtype=[('x', 'f4'),('y', 'f4'),('z', 'f4'),\
                                                                    ('nx','f4'),('ny','f4'),('nz','f4')])
        el = PlyElement.describe(pverts, 'vertex')
        PlyData([el], text = True).write('ttube.ply')

        accspace = {}
        # get the preceived global model descriptor
        nperceivedpnts = perceivedpnts.shape[0]
        i = np.argmax(perceivedpnts, axis = 0)[2]
        for j in range(0,nperceivedpnts):
            print j, nperceivedpnts
            m_0 = np.asarray(perceivedpnts[i])
            m_1 = np.asarray(perceivedpnts[j])
            v_m0m1 = m_0-m_1
            v_m1m0 = m_1-m_0
            n_m0 = perceivednormals[i]
            n_m1 = perceivednormals[j]
            # f1, namely ||d||2
            f1 = np.linalg.norm(m_0-m_1)
            # f2, namely angle between n_m0 and v_m1m0
            f2 = rm.degree_between(n_m0, v_m1m0)
            # f3, namely angle between n_m1 and v_m0m1
            f3 = rm.degree_between(n_m1, v_m0m1)
            # f4, namely angle between n_m0 and n_m1
            f4 = rm.degree_between(n_m0, n_m1)
            # discretize the values
            try:
                f1d = int(math.floor(f1/ddist)*ddist+ddist)
                f2d = int(math.floor(f2/dangle)*dangle+dangle)
                f3d = int(math.floor(f3/dangle)*dangle+dangle)
                f4d = int(math.floor(f4/dangle)*dangle+dangle)
            except:
                continue
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
            if key in self.gmd.keys():
                malist = self.gmd[key]
                print len(malist)
                for maslot in malist:
                    alpha = math.degrees(alpha_m0-maslot[2])
                    try:
                        alphadiscrete = int(math.floor(alpha/dangle)*dangle+dangle)
                    except:
                        continue
                    acckey = (maslot[0], alphadiscrete)
                    if acckey in accspace.keys():
                        accspace[acckey] += 1
                    else:
                        accspace[acckey] = 1
        if len(accspace.keys()) is 0:
            return (None, None)
        # find top matches and rot matrices
        maxn = sorted(accspace.iteritems(), key=operator.itemgetter(1), reverse=True)[:5]
        rotmat4list = []
        silist = []
        milist = []
        for maxnele in maxn:
            mi, alpha = maxnele[0]
            # step1 move to temppnts[mi]
            displacement0 = -self.temppnts[mi]
            rotmat4_0 = Mat4.translateMat(displacement0[0], displacement0[1], displacement0[2])
            # step2 rotate to goal
            normalangle = math.degrees(rm.radian_between(self.tempnormals[mi], perceivednormals[i]))
            normalrotax = np.cross(self.tempnormals[mi], perceivednormals[i])
            normalrotmat = rm.rodrigues(normalrotax, normalangle)
            anglerotmat = rm.rodrigues(perceivednormals[i], -alpha)
            rotmat = np.dot(anglerotmat, normalrotmat)
            rotmat4_1 = pg.npToMat4(rotmat)
            # step3 move to perceivedpnts[i]
            displacement1 = perceivedpnts[i]
            rotmat4_2 = Mat4.translateMat(displacement1[0], displacement1[1], displacement1[2])
            rotmat4 = rotmat4_0*rotmat4_1*rotmat4_2
            rotmat4list.append(rotmat4)
            silist.append(i)
            milist.append(mi)

        return rotmat4list, silist, milist

if __name__=='__main__':

    import copy

    base = pandactrl.World(camp=[0,0,700], lookatp=[0,0,0])

    this_dir, this_filename = os.path.split(__file__)
    objpath = os.path.join(this_dir, "models", "ttube.stl")
    ppfpath = os.path.join(this_dir, "models", "ttubegmd.pickle")

    # ppfmatch = PPFMatch(objpath, ppfpath = None, bsave = True)
    ppfmatch = PPFMatch(objpath, ppfpath = ppfpath)

    perceivedpnts, perceivednormals = ppfmatch.perceivePoints(1)
    # plotperceived pnts
    pntsnp = pg.genPntsnp(perceivedpnts)
    pntsnp.reparentTo(base.render)
    for i, normal in enumerate(perceivednormals):
        pg.plotArrow(base.render, spos = perceivedpnts[i], epos = normal+perceivedpnts[i], thickness = .2, length = 10)

    rotmat4list, silist, milist = ppfmatch.match(perceivedpnts, perceivednormals, ddist = 5.0, dangle = 30.0)

    try:
        for i in range(len(rotmat4list)):
        # for i in range(1):
            rotmat4 = rotmat4list[i]
            si = silist[i]
            mi = milist[i]
            #object
            # ppfmatch.objnp.setMat(rotmat4)
            # ppfmatch.objnp.reparentTo(base.render)
            # point arrow
            pg.plotArrow(base.render, spos = perceivedpnts[si], epos = perceivednormals[si]+perceivedpnts[si], thickness = 4, length = 50)
            # temp arrow
            # spos = Vec3(ppfmatch.temppnts[mi][0], ppfmatch.temppnts[mi][1], ppfmatch.temppnts[mi][2])
            # spos = rotmat4.xfrom(spos)
            # spos = np.array([spos[0], spos[1], spos[2]])
            # normal = Vec3(ppfmatch.tempnormals[mi][0], ppfmatch.tempnormals[mi][1], ppfmatch.tempnormals[mi][2])
            # normal = rotmat4.xformVec(normal)
            # normal = np.array([normal[0], normal[1], normal[2]])
            # pg.plotArrow(base.render, spos = spos, epos = spos+normal, thickness = 8, length = 25)
            # temp cloud
            # temppntsnp = pg.genPntsnp(ppfmatch.temppnts)
            # temppntsnp.setMat(rotmat4)
            # temppntsnp.reparentTo(base.render)
            # temp nodepath
            # before rotate
            tempnp = copy.deepcopy(ppfmatch.objnp)
            tempnp.reparentTo(base.render)
            pg.plotArrow(base.render, spos = ppfmatch.temppnts[mi], epos = ppfmatch.temppnts[mi]+ppfmatch.tempnormals[mi], thickness = 8, length = 25)
            # after rotate
            tempnp = copy.deepcopy(ppfmatch.objnp)
            tempnp.setMat(rotmat4)
            tempnp.reparentTo(base.render)
    except:
        pass

    # ppfmatch = PPFMatch(objpath)
    # ppfmatch.computePPFwithAlpha()
    #
    # colors = []
    # color = [np.random.rand(), np.random.rand(), np.random.rand(), np.random.rand()]
    # for pnt in ppfmatch.temppnts:
    #     colors.append(color)
    # pntsnp = pg.genPntsnp(ppfmatch.temppnts, colors=colors)
    # for i, normal in enumerate(ppfmatch.tempnormals):
    #     pg.plotArrow(base.render, spos = ppfmatch.temppnts[i], epos = normal+ppfmatch.temppnts[i], thickness = .2, length =2)

    # pg.plotAxisSelf(base.render)
    base.run()