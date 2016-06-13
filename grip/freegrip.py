#!/usr/bin/python

import numpy as np
import trimesh
from trimesh import sample
import plot.ax3dequal as ax3dequal
import plot.packpandageom as ppg

def surfsampling():
    facets = mesh.facets()
    for faces in facets:
        for faceid in faces:
            for vertid in mesh.faces[faceid]:
                print mesh.vertices[vertid]

class freegrip:
    def __init__(self):
        self.objmodel = None
        self.hndmodel = None
    def loadObjModel(self, ompath):
        mesh = trimesh.load_mesh(ompath)


if __name__=='__main__':
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D as ax3d
    from mpl_toolkits.mplot3d import art3d
    fig = plt.figure()
    ax1 = fig.add_subplot(121, projection='3d')

    mesh = trimesh.load_mesh('./circlestar.obj')
    facets, facets_area = mesh.facets(return_area=True)
    for face in facets:
        mesh.visual.face_colors[np.asarray(face)] = [trimesh.visual.random_color()]*mesh.visual.face_colors[face].shape[0]


    # mesh.show()
    # samples = sample.sample_surface_even(mesh, mesh.vertices.shape[0]*10)
    # ax3d.plot(ax1, samples[:,0], samples[:,1], samples[:,2], 'r.')
    # ax3dequal.set_axes_equal(ax1)
    #
    # ax2 = fig.add_subplot(122, projection='3d')
    # for face in facets:
    #     rndcolor = trimesh.visual.random_color()
    #     for faceid in face:
    #         triarray = mesh.vertices[mesh.faces[faceid]]
    #         tri = art3d.Poly3DCollection([triarray])
    #         tri.set_facecolor(mesh.visual.face_colors[faceid])
    #         ax2.add_collection3d(tri)
    #
    # ax3dequal.set_axes_equal(ax2)
    # plt.show()

    from direct.showbase.ShowBase import ShowBase
    from panda3d.core import *

    geom = ppg.packpandageom(mesh.vertices, mesh.faces)
    node = GeomNode('usr gnode')

    node.addGeom(geom)
    base = ShowBase()
    base.render.attachNewNode(node)
    base.run()
