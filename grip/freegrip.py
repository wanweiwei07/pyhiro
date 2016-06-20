#!/usr/bin/python

import numpy as np
import trimesh
from trimesh import sample
import plot.ax3dequal as ax3dequal
import plot.pandageom as ppg

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

    # for face in facets:
    #     mesh.visual.face_colors[np.asarray(face)] = [trimesh.visual.random_color()]*mesh.visual.face_colors[face].shape[0]
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

    # ax3dequal.set_axes_equal(ax2)
    # plt.show()
    #
    from direct.showbase.ShowBase import ShowBase
    from panda3d.core import *
    import plot.pandactrl as pandactrl
    import plot.pandageom as pandageom

    geom = ppg.packpandageom(mesh.vertices, mesh.face_normals, mesh.faces)
    node = GeomNode('star')
    node.addGeom(geom)
    star = NodePath('star')
    star.attachNewNode(node)
    star.setColor(1,0,0)


    base = ShowBase()

    # for i, face in enumerate(mesh.faces):
    #     vert = (mesh.vertices[face[0],:]+mesh.vertices[face[1],:]+mesh.vertices[face[2],:])/3
    #     pandageom.plotArrow(base, star, spos=vert, epos=vert+mesh.face_normals[i,:], rgba=[1,0,0,1], length = 5, thickness = 0.1)

    # for i, vert in enumerate(mesh.vertices):
    #     pandageom.plotArrow(base, star, spos=vert, epos=vert+mesh.vertex_normals[i,:], rgba=[1,0,0,1], length = 5, thickness = 0.1)

    pandactrl.setRenderEffect(base)
    pandactrl.setLight(base)
    pandactrl.setCam(base, 0, 100, 100, 'perspective')

    star.reparentTo(base.render)

    generator = MeshDrawer()
    generatorNode = generator.getRoot()
    generatorNode.reparentTo(base.render)
    generatorNode.setDepthWrite(False)
    generatorNode.setTransparency(True)
    generatorNode.setTwoSided(True)
    generatorNode.setBin("fixed", 0)
    generatorNode.setLightOff(True)

    generator.begin(base.cam, base.render)
    generator.segment(Vec3(0,0,0), Vec3(10,0,0), Vec4(1,1,1,1), 0.5, Vec4(0,1,0,1))
    generator.end()

    base.run()
