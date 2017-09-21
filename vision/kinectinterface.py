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
from trimesh import points
from sklearn import linear_model

class KinectInterface(object):

    def __init__(self):
        """
        Kinect interface

        author: weiwei
        date: 20170715
        """

        self.kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth)
        self.dbscan = DBSCAN(eps=50, min_samples=100, n_jobs=-1)
        self.randsac = linear_model.RANSACRegressor(linear_model.LinearRegression(), residual_threshold = 15)

    def __depthToXYZ(self, dframe, width, height):
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
                z3d = dframe[i * self.kinect.depth_frame_desc.Width + j]
                verts.append([(j - cy) * z3d / fy, -(i - cx) * z3d / fx, -float(z3d)])
        return verts

    def __getClustersofPnts(self, verts):
        """

        get clusters of points

        :param verts: see depthToXYZ
        :return: vertslist each element of the list is a cluter of points, the last one is the table

        author: weiwei
        date: 20170711
        """

        cutverts = []
        for vert in verts:
            if vert[0] < 600.0 and vert[0] > -250.0:
                if vert[1] < 150.0 and vert[1] > -300.0:
                    if vert[2] < -800.0 and vert[2] > -1200.0:
                            cutverts.append([vert[0], vert[1], vert[2]])

        XYZ = np.array(cutverts)
        self.randsac.fit(XYZ[:,0:2], XYZ[:, 2])
        inlier_mask = self.randsac.inlier_mask_
        outlier_mask = np.logical_not(inlier_mask)

        tablepcd = XYZ[inlier_mask]
        objpcds = XYZ[outlier_mask]

        # clustering using DBSCAN
        db = self.dbscan.fit(objpcds)
        core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
        core_samples_mask[db.core_sample_indices_] = True
        labels = db.labels_

        returnvertslist = []
        unique_labels = set(labels)
        for k in unique_labels:
            class_member_mask = (labels == k)
            objpcd = objpcds[class_member_mask & core_samples_mask]
            returnvertslist.append(objpcd)
        returnvertslist.append(tablepcd)

        return np.array(returnvertslist)

    def __getMultiframeVerts(self, nframe = 3):
        """
        merge three frames of verts

        :param nframe:
        :return:
        """

        verts = []
        counter = 0
        while True:
            if counter >= nframe:
                break
            if self.kinect.has_new_depth_frame():
                dframe = self.kinect.get_last_depth_frame()
                dwidth = self.kinect.depth_frame_desc.Width
                dheight = self.kinect.depth_frame_desc.Height
                verts.extend(self.__depthToXYZ(dframe, dwidth, dheight))
                counter += 1

        return verts


    def getTablePcd(self, sampleradius = 10):
        """
        Get the pointcloud of a table # last cluster
        :return: a list of points representing the table

        :param sampleradius: radius to refined the samples

        author: weiwei
        date: 20170711
        """

        while True:
            if self.kinect.has_new_depth_frame():
                dframe = self.kinect.get_last_depth_frame()
                dwidth = self.kinect.depth_frame_desc.Width
                dheight = self.kinect.depth_frame_desc.Height
                tmpvertsarray = self.__getClustersofPnts(self.__depthToXYZ(dframe, dwidth, dheight))

                self.tablepnt = tmpvertsarray[-1]

                # sampling
                self.tablepnt = points.remove_close(self.tablepnt, sampleradius)
                return self.tablepnt

    def getObjectPcd(self, sampleradius = 2, nframe = 5):
        """
        Get the pointcloud of an object # first cluster
        :return: a list of points representing the object

        :param sampleradius: radius to refined the samples

        author: weiwei
        date: 20170711
        """

        tmpvertsarray = self.__getClustersofPnts(self.__getMultiframeVerts(nframe = nframe))

        # use the largest cluster as object
        nmaxverts = 0
        for i, tmpverts in enumerate(tmpvertsarray):
            if tmpverts.shape[0] > nmaxverts and i != tmpvertsarray.shape[0]-1:
                nmaxverts = tmpverts.shape[0]
                self.objectpnt = tmpverts

        # sampling
        self.objectpnt = points.remove_close(self.objectpnt, sampleradius)
        return self.objectpnt

    def getAllObjectPcd(self, sampleradius = 2):
        """
        Get the pointcloud of all objects except for the table
        :return: a list of points representing the object

        :param sampleradius: radius to refined the samples

        author: weiwei
        date: 20170711
        """

        tmpvertsarray = self.__getClustersofPnts(self.__getMultiframeVerts(nframe = 5))
        # sampling
        for i in range(tmpvertsarray.shape[0]):
            tmpvertsarray[i] = points.remove_close(tmpvertsarray[i], sampleradius)
        return tmpvertsarray[0:tmpvertsarray.shape[0]-1]

if __name__=='__main__':

    base = pandactrl.World(camp=[0,0,300], lookatp=[0,0,0])

    kface = KinectInterface()
    # points
    objectpnts = kface.getObjectPcd()
    print objectpnts
    # # normals
    normals = tools.estimatenormals(objectpnts)

    # transform
    objectpntscenter = np.mean(objectpnts, axis=0)
    for i in range(objectpnts.shape[0]):
        objectpnts[i] = objectpnts[i]-objectpntscenter

    colors = []
    color = [np.random.rand(), np.random.rand(), np.random.rand(), np.random.rand()]
    for pnt in objectpnts:
        colors.append(color)
    pntsnp = pg.genPntsnp(objectpnts, colors=colors)
    pntsnp.reparentTo(base.render)
    for i, normal in enumerate(normals):
        pg.plotArrow(base.render, spos = objectpnts[i], epos = normal+objectpnts[i], thickness = .2, length = 2)

    # import robotsim.nextage.nxt as nxt
    # import robotsim.nextage.nxtplot as nxtplot
    # import pandaplotutils.pandageom as pg
    # from manipulation.grip.robotiq85 import rtq85nm
    # handpkg = rtq85nm
    # nxtrobot = nxt.NxtRobot()
    # nxtmnp = nxtplot.genmnp(nxtrobot, handpkg)
    # nxtmnp.reparentTo(base.render)
    # pg.plotAxisSelf(base.render)

    # objarray  = kface.getAllObjectPcd()
    # for objectpnts in objarray:
    #     colors = []
    #     color = [np.random.rand(), np.random.rand(), np.random.rand(), np.random.rand()]
    #     for pnt in objectpnts:
    #         colors.append(color)
    #     pntsnp = pg.genPntsnp(objectpnts, colors=colors)
    #     pntsnp.reparentTo(base.render)

    base.run()