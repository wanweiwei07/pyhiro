#!/usr/bin/python

import numpy as np
import trimesh
import sample
import plot.pandageom as ppg
from direct.showbase.ShowBase import ShowBase
from panda3d.core import *
import plot.pandactrl as pandactrl
import plot.pandageom as pandageom
from utils import robotmath
from shapely.geometry import Polygon
from shapely.geometry import Point
import matplotlib.pyplot as plt


class freegrip:
    def __init__(self, ompath):
        self.objtrimesh = None
        # the sampled points and their normals
        self.objsamplepnts = None
        self.objsamplenrmls = None
        # the sampled points (bad samples removed)
        self.objsamplepnts_grp = None
        self.objsamplenrmls_grp = None
        self.hndmodel = None
        self.grasps = None
        self.loadObjModel(ompath)

    def loadObjModel(self, ompath):
        self.objtrimesh=trimesh.load_mesh(ompath)
        self.sampleObjModel()

    def sampleObjModel(self, numpointsoververts=10):
        '''
        sample the object model

        ## input
        numpointsoververts
            the number of sampled points = numpointsoververts*mesh.vertices.shape[0]

        self.objsamplepnts and self.objsamplenrmls
        are filled in this function

        author: weiwei
        date: 20160623 flight to tokyo
        '''
        facets = self.objtrimesh.facets()
        samples, face_idx = sample.sample_surface_even(self.objtrimesh, count=self.objtrimesh.vertices.shape[0] * 10)
        self.objsamplepnts = np.ndarray(shape=(facets.shape[0],), dtype=np.object)
        self.objsamplenrmls = np.ndarray(shape=(facets.shape[0],), dtype=np.object)
        for i, faces in enumerate(facets):
            sample_idx = np.empty([0, 0], dtype=np.int)
            for face in faces:
                sample_idx = np.append(sample_idx, np.where(face_idx == face)[0])
            self.objsamplepnts[i] = samples[sample_idx]
            self.objsamplenrmls[i] = np.asarray([self.objtrimesh.face_normals[faces[0]]]*samples[sample_idx].shape[0])

    def removeBadSamples(self, mindist=3, maxdist=20):
        '''
        Do the following refinement:
        (1) remove the samples who's minimum distance to facet boundary is smaller than mindist
        (2) remove the samples who's maximum distance to facet boundary is larger than mindist

        ## input
        mindist, maxdist
            as explained in the begining

        author: weiwei
        date: 20160623 flight to tokyo
        '''
        facets = self.objtrimesh.facets()
        self.objsamplepnts_grp = np.ndarray(shape=(facets.shape[0],), dtype=np.object)
        self.objsamplenrmls_grp = np.ndarray(shape=(facets.shape[0],), dtype=np.object)
        for i, faces in enumerate(facets):
            facetp = None
            face0verts = self.objtrimesh.vertices[self.objtrimesh.faces[faces[0]]]
            facetmat = robotmath.rotmatfacet(self.objtrimesh.face_normals[faces[0]], face0verts[0], face0verts[1])
            # face samples
            samplepntsp =[]
            for j, apnt in enumerate(self.objsamplepnts[i]):
                apntp = np.dot(facetmat, apnt)[:2]
                samplepntsp.append(apntp)
            # face boundaries
            for j, faceidx in enumerate(faces):
                vert0 = self.objtrimesh.vertices[self.objtrimesh.faces[faceidx][0]]
                vert1 = self.objtrimesh.vertices[self.objtrimesh.faces[faceidx][1]]
                vert2 = self.objtrimesh.vertices[self.objtrimesh.faces[faceidx][2]]
                vert0p = np.dot(facetmat, vert0)[:2]
                vert1p = np.dot(facetmat, vert1)[:2]
                vert2p = np.dot(facetmat, vert2)[:2]
                facep = Polygon([vert0p, vert1p, vert2p, vert0p])
                # fpx, fpy = facep.exterior.xy
                # plt.plot(fpx, fpy)
                if facetp is None:
                    facetp = facep
                else:
                    facetp = facetp.union(facep)
            selectedele = []
            for j, apntp in enumerate(samplepntsp):
                apntpnt = Point(apntp[0], apntp[1])
                dbnds = []
                dbnds.append(apntpnt.distance(facetp.exterior))
                for fpinter in facetp.interiors:
                    dbnds.append(apntpnt.distance(fpinter))
                dbnd = min(dbnds)
                if dbnd < mindist or dbnd > maxdist:
                    plt.plot(apntpnt.x, apntpnt.y, 'bo')
                else:
                    selectedele.append(j)
            self.objsamplepnts_grp[i] = np.asarray([self.objsamplepnts[i][j] for j in selectedele])
            self.objsamplenrmls_grp[i] = np.asarray([self.objsamplenrmls[i][j] for j in selectedele])
            # for j, apnt in enumerate([samplepntsp[j] for j in selectedele]):
            #     plt.plot(apnt[0], apnt[1], 'ro')
            # ftpx, ftpy = facetp.exterior.xy
            # plt.plot(ftpx, ftpy)
            # for fpinters in facetp.interiors:
            #     ftpxi, ftpyi = fpinters.xy
            #     plt.plot(ftpxi, ftpyi)
            # plt.axis('equal')
            # plt.show()

                # old code for concatenating in 3d space
                # boundaryedges = []
                # for faceid in faces:
                #     faceverts = self.objtrimesh.faces[faceid]
                #     try:
                #         boundaryedges.remove([faceverts[1], faceverts[0]])
                #     except:
                #         boundaryedges.append([faceverts[0], faceverts[1]])
                #     try:
                #         boundaryedges.remove([faceverts[2], faceverts[1]])
                #     except:
                #         boundaryedges.append([faceverts[1], faceverts[2]])
                #     try:
                #         boundaryedges.remove([faceverts[0], faceverts[2]])
                #     except:
                #         boundaryedges.append([faceverts[2], faceverts[0]])
                # print boundaryedges
                # print len(boundaryedges)
                # TODO: compute boundary polygons, both outsider and inner should be considered
                # assort boundaryedges
                # boundarypolygonlist = []
                # boundarypolygon = [boundaryedges[0]]
                # boundaryedgesfirstcolumn = [row[0] for row in boundaryedges]
                # for i in range(len(boundaryedges)-1):
                #     vertpivot = boundarypolygon[i][1]
                #     boundarypolygon.append(boundaryedges[boundaryedgesfirstcolumn.index(vertpivot)])
                # print boundarypolygon
                # print len(boundarypolygon)
                # return boundaryedges, boundarypolygon

    def tstShow(self, base, togglesamples=False, togglenormals=False, togglesamples_grp=False, togglenormals_grp=False):
        try:
            for i, faces in enumerate(self.objtrimesh.facets()):
                rgba = [np.random.random(), np.random.random(), np.random.random(), 1]
                geom = ppg.packpandageom(self.objtrimesh.vertices, self.objtrimesh.face_normals[faces],
                                         self.objtrimesh.faces[faces])
                node = GeomNode('piece')
                node.addGeom(geom)
                star = NodePath('piece')
                star.attachNewNode(node)
                star.setColor(Vec4(rgba[0], rgba[1], rgba[2], rgba[3]))
                star.setTwoSided(True)
                star.reparentTo(base.render)
                # sampledpnts = samples[sample_idxes[i]]
                # for apnt in sampledpnts:
                #     pandageom.plotSphere(base, star, pos=apnt, radius=1, rgba=rgba)
                if togglesamples:
                    for j, apnt in enumerate(self.objsamplepnts[i]):
                        pandageom.plotSphere(base, star, pos=apnt, radius=1, rgba=rgba)
                if togglenormals:
                    for j, apnt in enumerate(self.objsamplepnts[i]):
                        pandageom.plotArrow(base, star, spos=apnt, epos=apnt + freegriptst.objsamplenrmls[i][j],
                                            rgba=[1, 0, 0, 1], length=10)
                if togglesamples_grp:
                    for j, apnt in enumerate(self.objsamplepnts_grp[i]):
                        pandageom.plotSphere(base, star, pos=apnt, radius=1, rgba=rgba)
                if togglenormals_grp:
                    for j, apnt in enumerate(self.objsamplepnts_grp[i]):
                        pandageom.plotArrow(base, star, spos=apnt, epos=apnt + freegriptst.objsamplenrmls_grp[i][j],
                                            rgba=[1, 0, 0, 1], length=10)
        except:
            print "You might need to loadmodel first!"

if __name__=='__main__':
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D as ax3d
    from mpl_toolkits.mplot3d import art3d
    # fig = plt.figure()
    # ax1 = fig.add_subplot(121, projection='3d')
    #
    # mesh = trimesh.load_mesh('./circlestar.obj')
    # samples, face_idx = sample.sample_surface_even(mesh, mesh.vertices.shape[0] * 10)
    # facets, facets_area = mesh.facets(return_area=True)
    # sample_idxes = np.ndarray(shape=(facets.shape[0],),dtype=np.object)
    # for i,faces in enumerate(facets):
    #     sample_idx = np.empty([0,0], dtype=np.int)
    #     for face in faces:
    #         sample_idx = np.append(sample_idx, np.where(face_idx == face)[0])
    #     sample_idxes[i]=sample_idx
    #


    freegriptst = freegrip('./wheelmeter.stl')
    # freegriptst.objtrimesh.show()

    base = ShowBase()
    freegriptst.removeBadSamples()
    freegriptst.tstShow(base, togglesamples_grp=True, togglenormals_grp=True)
    # freegriptst.tstShow(base)

    # geom = None
    # for i, faces in enumerate(freegriptst.objtrimesh.facets()):
    #     rgba = [np.random.random(),np.random.random(),np.random.random(),1]
    #     geom = ppg.packpandageom(freegriptst.objtrimesh.vertices, freegriptst.objtrimesh.face_normals[faces], freegriptst.objtrimesh.faces[faces])
    #     node = GeomNode('piece')
    #     node.addGeom(geom)
    #     star = NodePath('piece')
    #     star.attachNewNode(node)
    #     star.setColor(Vec4(rgba[0],rgba[1],rgba[2],rgba[3]))
    #     star.setTwoSided(True)
    #     star.reparentTo(base.render)
    #     # sampledpnts = samples[sample_idxes[i]]
    #     # for apnt in sampledpnts:
    #     #     pandageom.plotSphere(base, star, pos=apnt, radius=1, rgba=rgba)
    #     for j, apnt in enumerate(freegriptst.objsamplepnts[i]):
    #         pandageom.plotSphere(base, star, pos=apnt, radius=0.7, rgba=rgba)
    #         pandageom.plotArrow(base, star, spos=apnt, epos=apnt+freegriptst.objsamplenrmls[i][j], rgba=[1,0,0,1], length=5, thickness=0.1)
    # # selectedfacet = 2
    # geom = ppg.packpandageom(mesh.vertices, mesh.face_normals[facets[selectedfacet]], mesh.faces[facets[selectedfacet]])
    # node = GeomNode('piece')
    # node.addGeom(geom)
    # star = NodePath('piece')
    # star.attachNewNode(node)
    # star.setColor(Vec4(1,0,0,1))
    # star.setTwoSided(True)
    # star.reparentTo(base.render)

    # for i, face in enumerate(mesh.faces[facets[selectedfacet]]):
    #     vert = (mesh.vertices[face[0],:]+mesh.vertices[face[1],:]+mesh.vertices[face[2],:])/3
    #     pandageom.plotArrow(base, star, spos=vert, epos=vert+mesh.face_normals[facets[selectedfacet][i],:], rgba=[1,0,0,1], length = 5, thickness = 0.1)

    # for i, vert in enumerate(mesh.vertices):
    #     pandageom.plotArrow(base, star, spos=vert, epos=vert+mesh.vertex_normals[i,:], rgba=[1,0,0,1], length = 5, thickness = 0.1)

    pandactrl.setRenderEffect(base)
    pandactrl.setLight(base)
    pandactrl.setCam(base, 0, 500, 500, 'perspective')


    # generator = MeshDrawer()
    # generatorNode = generator.getRoot()
    # generatorNode.reparentTo(base.render)
    # generatorNode.setDepthWrite(False)
    # generatorNode.setTransparency(True)
    # generatorNode.setTwoSided(True)
    # generatorNode.setBin("fixed", 0)
    # generatorNode.setLightOff(True)
    #
    # generator.begin(base.cam, base.render)
    # generator.segment(Vec3(0,0,0), Vec3(10,0,0), Vec4(1,1,1,1), 0.5, Vec4(0,1,0,1))
    # generator.end()
    # mesh.show()
    base.run()

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
    # from direct.showbase.ShowBase import ShowBase
    # from panda3d.core import *
    # import plot.pandactrl as pandactrl
    # import plot.pandageom as pandageom
    #
    # geom = ppg.packpandageom(mesh.vertices, mesh.face_normals, mesh.faces)
    # node = GeomNode('star')
    # node.addGeom(geom)
    # star = NodePath('star')
    # star.attachNewNode(node)
    # star.setColor(1,0,0)
    #
    #
    # base = ShowBase()
    #
    # # for i, face in enumerate(mesh.faces):
    # #     vert = (mesh.vertices[face[0],:]+mesh.vertices[face[1],:]+mesh.vertices[face[2],:])/3
    # #     pandageom.plotArrow(base, star, spos=vert, epos=vert+mesh.face_normals[i,:], rgba=[1,0,0,1], length = 5, thickness = 0.1)
    #
    # # for i, vert in enumerate(mesh.vertices):
    # #     pandageom.plotArrow(base, star, spos=vert, epos=vert+mesh.vertex_normals[i,:], rgba=[1,0,0,1], length = 5, thickness = 0.1)
    #
    # pandactrl.setRenderEffect(base)
    # pandactrl.setLight(base)
    # pandactrl.setCam(base, 0, 100, 100, 'perspective')
    #
    # star.reparentTo(base.render)
    #
    # generator = MeshDrawer()
    # generatorNode = generator.getRoot()
    # generatorNode.reparentTo(base.render)
    # generatorNode.setDepthWrite(False)
    # generatorNode.setTransparency(True)
    # generatorNode.setTwoSided(True)
    # generatorNode.setBin("fixed", 0)
    # generatorNode.setLightOff(True)
    #
    # generator.begin(base.cam, base.render)
    # generator.segment(Vec3(0,0,0), Vec3(10,0,0), Vec4(1,1,1,1), 0.5, Vec4(0,1,0,1))
    # generator.end()
    #
    # base.run()
