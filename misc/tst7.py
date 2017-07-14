from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
import pandaplotutils.pandageom as pg
import pandaplotutils.pandactrl as pc
import numpy as np
from sklearn.cluster import DBSCAN

import math
from vision import CADTemp

def colorToDepth(dwidth, dheight, cframe, cwidth, cheight):
    """

    :param cframe:
    :return:
    """

    print cframe
    depthcolor = []
    for i in range(dheight):
        for j in range(dwidth):
            depthcolor.append([0.0,0.0,0.0,1.0])
    tmat = np.array([[0.3584, 0.0031, -101.5934], [0.0089, 0.3531, 13.6311], [0.0000, 0.0001, 0.9914]])
    for i in range(cheight):
        for j in range(cwidth):
            tvec = np.array([i,j,1])
            nvec = tmat.dot(tvec)
            # print nvec, nvec[0]/nvec[2], nvec[1]/nvec[2]
            newi = int(nvec[0]/nvec[2])
            newj = int(nvec[1]/nvec[2])
            # print newi,newj
            if newi >= 0 and newi < dheight and  newj >= 0 and newj < dwidth:
                depthcolor[newi*dwidth+newj] = [float(cframe[cwidth*4*i+j*4])/255.0, float(cframe[cwidth*4*i+j*4+1])/255.0, \
                               float(cframe[cwidth*4*i+j*4+2])/255.0, float(cframe[cwidth*4*i+j*4+3])/255.0]
                # print cframe[cwidth*4*i+j*4]
            # print depthcolor[-1]

    return depthcolor


def depthToXYZ(dframe, width, height):
    """

    :param dframe:
    :param width:
    :param height:
    :return:

    author: weiwei
    date: 20170329
    """

    cx = 254.878
    cy = 205.395
    fx = 365.456
    fy = 365.456

    verts = []
    for i in range(height):
        for j in range(width):
            z3d = dframe[i * kinect.depth_frame_desc.Width + j]
            verts.append([(j-cy)*z3d/fy, -(i-cx)*z3d/fx, -float(z3d)])
    return verts

def getRotMat(verts):
    """

    find the table and do calibration

    :param verts: see depthToXYZ
    :return:

    author: weiwei
    date: 20170711
    """

    cutverts = []
    for vert in verts:
        if vert[0] < 700.0 and vert[0] > -700.0:
            if vert[1] < 200.0 and vert[1] > -600.0:
                if vert[2] < -1000.0 and vert[2] > -1500.0:
                    cutverts.append([vert[0], vert[1], vert[2]])

    # clustering using DBSCAN
    X = np.array(cutverts)
    db = DBSCAN(eps=20, min_samples = 100, n_jobs = -1).fit(X)
    print db.labels_
    core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True
    labels = db.labels_

    returnvertslist = []
    unique_labels = set(labels)
    for k in unique_labels:
        class_member_mask = (labels == k)
        print class_member_mask, core_samples_mask
        xyzlist = X[class_member_mask & core_samples_mask]
        print xyzlist
        returnvertslist.append(xyzlist.tolist())

    return returnvertslist
    # return verts

def kinxyzToRobXYZ(verts):
    """
    convert the xyz coordinates from the frame of kinect to the frame of robot

    :param verts: a list of [x,y,z] in kinect's frame
    :return:

    author: weiwei
    date: 20170407
    """

    offsetx = 1000.0
    offsetz = 1200.0

    verts_robxyz = []
    for vert in verts:
        verts_robxyz.append([vert[1]+offsetx, vert[0], vert[2]+offsetz])
    return verts_robxyz

base = pc.World(camp=[500,0,3000], lookatp=[0,0,0])

kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Depth)

while True:
    if kinect.has_new_depth_frame():
        cframe = kinect.get_last_color_frame()
        cwidth = kinect.color_frame_desc.Width
        cheight = kinect.color_frame_desc.Height
        dframe = kinect.get_last_depth_frame()
        dwidth =  kinect.depth_frame_desc.Width
        dheight = kinect.depth_frame_desc.Height
        tmpvertslist = getRotMat(depthToXYZ(dframe, dwidth, dheight))
        for tmpverts in tmpvertslist:
            dverts= kinxyzToRobXYZ(tmpverts)
            color = [np.random.rand(), np.random.rand(), np.random.rand(), np.random.rand()]
            colors = []
            for dvert in dverts:
                colors.append(color)
            pntsnp = pg.genPntsnp(dverts, colors = colors)
            pntsnp.reparentTo(base.render)
        break

import robotsim.nextage.nxt as nxt
import robotsim.nextage.nxtplot as nxtplot
import pandaplotutils.pandageom as pg
from manipulation.grip.robotiq85 import rtq85nm
handpkg = rtq85nm
nxtrobot = nxt.NxtRobot()
nxtmnp = nxtplot.genmnp(nxtrobot, handpkg)
nxtmnp.reparentTo(base.render)
pg.plotAxisSelf(base.render)

base.run()