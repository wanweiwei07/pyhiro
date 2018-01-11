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
import ctypes

class KinectAPI():

    def __init__(self):
        """
        Kinect interface

        author: weiwei
        date: 20170715
        """

        self.kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth)
        self.dbscan = DBSCAN(eps=50, min_samples=100, n_jobs=-1)
        self.randsac = linear_model.RANSACRegressor(linear_model.LinearRegression(), residual_threshold = 15)

    def getPointCloud(self):
        while True:
            if self.kinect.has_new_depth_frame():
                dframe = self.kinect.get_last_depth_frame()
                dwidth = self.kinect.depth_frame_desc.Width
                dheight = self.kinect.depth_frame_desc.Height

                pcd = []
                for i in range(dheight):
                    for j in range(dwidth):
                        depth = dframe[i * dwidth + j]
                        if depth > 0:
                            point3d = self.kinect.mapper().MapDepthPointToCameraSpace(
                                PyKinectV2._DepthSpacePoint(ctypes.c_float(j), ctypes.c_float(i)), ctypes.c_ushort(depth))
                            # pcd.append((point3d.x*1000.0, point3d.y*1000.0, point3d.z*1000.0))
                            pcd.append((point3d.x, point3d.y, point3d.z))
                break

        return pcd

    def getMakerXYZ(self, markercolumnid, markerrowid, markerdepth):
        if markerdepth > 0:
            point3d = self.kinect.mapper().MapDepthPointToCameraSpace(
                PyKinectV2._DepthSpacePoint(ctypes.c_float(markercolumnid), ctypes.c_float(markerrowid)),
                ctypes.c_ushort(markerdepth))
            return [point3d.x, point3d.y, point3d.z]
        else:
            return []

if __name__=='__main__':

    base = pandactrl.World(camp=[0,0,3000], lookatp=[0,0,0])

    kapi = KinectAPI()
    # points
    pntclds = kapi.getPointCloud()

    colors = []
    color = [np.random.rand(), np.random.rand(), np.random.rand(), np.random.rand()]
    for pnt in pntclds:
        colors.append(color)
    pntsnp = pg.genPntsnp(pntclds, colors=colors)
    pntsnp.reparentTo(base.render)

    from plyfile import PlyData, PlyElement
    verts = np.array(pntclds, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
    el = PlyElement.describe(verts, 'vertex')
    PlyData([el], text=True).write('pythoncloud.ply')

    base.run()