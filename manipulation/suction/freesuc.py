#!/usr/bin/python

import os
import pickle
import sys

import numpy as np
from panda3d.core import *
from shapely.geometry import Point
from shapely.geometry import Polygon
from sklearn.cluster import KMeans
from sklearn.neighbors import RadiusNeighborsClassifier

import manipulation.suction.sample
import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pandageom
from panda3d.bullet import BulletWorld
from utils import collisiondetection as cd
import trimesh
from utils import robotmath
from utils import dbcvt as dc
import manipulation.suction.sandmmbs.sdmbs as sdmbstipsd

class Freesuc(object):

    def __init__(self, ompath, handpkg, ser=False, torqueresist = 50):
        self.objtrimesh = None
        # the sampled points and their normals
        self.objsamplepnts = None
        self.objsamplenrmls = None
        # the sampled points (bad samples removed)
        self.objsamplepnts_ref = None
        self.objsamplenrmls_ref = None
        # the sampled points (bad samples removed + clustered)
        self.objsamplepnts_refcls = None
        self.objsamplenrmls_refcls = None
        # facets is used to avoid repeated computation
        self.facets = None
        # facetnormals is used to plot overlapped facets with different heights
        self.facetnormals = None
        # facet2dbdries saves the 2d boundaries of each facet
        self.facet2dbdries = None
        # for plot
        self.facetcolorarray = None
        self.counter = 0
        # for collision detection
        self.bulletworld = BulletWorld()
        self.hand = handpkg.newHandNM(hndcolor = [.2,0.7,.2,.3])

        # collision free grasps
        self.sucrotmats = []
        self.succontacts = []
        self.succontactnormals = []
        # collided grasps
        self.sucrotmatscld = []
        self.succontactscld = []
        self.succontactnormalscld = []


        self.objcenter = [0,0,0]
        self.torqueresist = torqueresist

        if ser is False:
            self.loadObjModel(ompath)
            self.saveSerialized("tmpfsc.pickle")
        else:
            self.loadSerialized("tmpfsc.pickle", ompath)


    def loadObjModel(self, ompath):
        self.objtrimesh=trimesh.load_mesh(ompath)
        # oversegmentation
        self.facets, self.facetnormals = self.objtrimesh.facets_over(faceangle=.95)
        self.facetcolorarray = pandageom.randomColorArray(self.facets.shape[0])
        self.sampleObjModel()
        # prepare the model for collision detection
        self.objgeom = pandageom.packpandageom(self.objtrimesh.vertices, self.objtrimesh.face_normals, self.objtrimesh.faces)
        self.objmeshbullnode = cd.genCollisionMeshGeom(self.objgeom)
        self.bulletworld.attachRigidBody(self.objmeshbullnode)
        # object center
        self.objcenter = [0,0,0]

        for pnt in self.objtrimesh.vertices:
            self.objcenter[0]+=pnt[0]
            self.objcenter[1]+=pnt[1]
            self.objcenter[2]+=pnt[2]
        self.objcenter[0] = self.objcenter[0]/self.objtrimesh.vertices.shape[0]
        self.objcenter[1] = self.objcenter[1]/self.objtrimesh.vertices.shape[0]
        self.objcenter[2] = self.objcenter[2]/self.objtrimesh.vertices.shape[0]

    def loadSerialized(self, filename, ompath):
        self.objtrimesh=trimesh.load_mesh(ompath)
        try:
            self.facets, self.facetnormals, self.facetcolorarray, self.objsamplepnts, \
            self.objsamplenrmls, self.objsamplepnts_ref, self.objsamplenrmls_ref, \
            self.objsamplepnts_refcls, self.objsamplenrmls_refcls = \
            pickle.load(open(filename, mode="rb"))
        except:
            print str(sys.exc_info()[0])+" cannot load tmpcp.pickle"
            raise

    def saveSerialized(self, filename):
        pickle.dump([self.facets, self.facetnormals, self.facetcolorarray, self.objsamplepnts, \
         self.objsamplenrmls, self.objsamplepnts_ref, self.objsamplenrmls_ref, \
         self.objsamplepnts_refcls, self.objsamplenrmls_refcls], open(filename, mode="wb"))

    def sampleObjModel(self, numpointsoververts=5):
        """
        sample the object model
        self.objsamplepnts and self.objsamplenrmls
        are filled in this function

        :param: numpointsoververts: the number of sampled points = numpointsoververts*mesh.vertices.shape[0]
        :return: nverts: the number of verts sampled

        author: weiwei
        date: 20160623 flight to tokyo
        """

        nverts = self.objtrimesh.vertices.shape[0]
        samples, face_idx = manipulation.suction.sample.sample_surface_even(self.objtrimesh,
                                                                            count=(1000 if nverts*numpointsoververts > 1000 \
                                                                  else nverts*numpointsoververts))
        # print nverts
        self.objsamplepnts = np.ndarray(shape=(self.facets.shape[0],), dtype=np.object)
        self.objsamplenrmls = np.ndarray(shape=(self.facets.shape[0],), dtype=np.object)
        for i, faces in enumerate(self.facets):
            for face in faces:
                sample_idx = np.where(face_idx==face)[0]
                if len(sample_idx) > 0:
                    if self.objsamplepnts[i] is not None:
                        self.objsamplepnts[i] = np.vstack((self.objsamplepnts[i], samples[sample_idx]))
                        self.objsamplenrmls[i] = np.vstack((self.objsamplenrmls[i],
                                                            [self.objtrimesh.face_normals[face]]*samples[sample_idx].shape[0]))
                    else:
                        self.objsamplepnts[i] = np.array(samples[sample_idx])
                        self.objsamplenrmls[i] = np.array([self.objtrimesh.face_normals[face]]*samples[sample_idx].shape[0])
            if self.objsamplepnts[i] is None:
                self.objsamplepnts[i] = np.empty(shape=[0,0])
                self.objsamplenrmls[i] = np.empty(shape=[0,0])
        return nverts

    def removeBadSamples(self, mindist=7, maxdist=9999):
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

        # ref = refine
        self.objsamplepnts_ref = np.ndarray(shape=(self.facets.shape[0],), dtype=np.object)
        self.objsamplenrmls_ref = np.ndarray(shape=(self.facets.shape[0],), dtype=np.object)
        self.facet2dbdries = []
        for i, faces in enumerate(self.facets):
            # print "removebadsample"
            # print i,len(self.facets)
            facetp = None
            face0verts = self.objtrimesh.vertices[self.objtrimesh.faces[faces[0]]]
            facetmat = robotmath.rotmatfacet(self.facetnormals[i], face0verts[0], face0verts[1])
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
                facep = Polygon([vert0p, vert1p, vert2p])
                if facetp is None:
                    facetp = facep
                else:
                    try:
                        facetp = facetp.union(facep)
                    except:
                        continue
            self.facet2dbdries.append(facetp)
            selectedele = []
            for j, apntp in enumerate(samplepntsp):
                try:
                    apntpnt = Point(apntp[0], apntp[1])
                    dbnds = []
                    dbnds.append(apntpnt.distance(facetp.exterior))
                    for fpinter in facetp.interiors:
                        dbnds.append(apntpnt.distance(fpinter))
                    dbnd = min(dbnds)
                    if dbnd < mindist or dbnd > maxdist:
                        pass
                    else:
                        selectedele.append(j)
                except:
                    pass
            self.objsamplepnts_ref[i] = np.asarray([self.objsamplepnts[i][j] for j in selectedele])
            self.objsamplenrmls_ref[i] = np.asarray([self.objsamplenrmls[i][j] for j in selectedele])
        self.facet2dbdries = np.array(self.facet2dbdries)

            # if i is 3:
            #     for j, apntp in enumerate(samplepntsp):
            #         apntpnt = Point(apntp[0], apntp[1])
            #         plt.plot(apntpnt.x, apntpnt.y, 'bo')
            #     for j, apnt in enumerate([samplepntsp[j] for j in selectedele]):
            #         plt.plot(apnt[0], apnt[1], 'ro')
            #     ftpx, ftpy = facetp.exterior.xy
            #     plt.plot(ftpx, ftpy)
            #     for fpinters in facetp.interiors:
            #         ftpxi, ftpyi = fpinters.xy
            #         plt.plot(ftpxi, ftpyi)
            #     plt.axis('equal')
            #     plt.show()
            #     pass

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

    def clusterFacetSamplesKNN(self, reduceRatio=3, maxNPnts=5):
        """
        cluster the samples of each facet using k nearest neighbors
        the cluster center and their correspondent normals will be saved
        in self.objsamplepnts_refcls and self.objsamplenrmals_refcls

        :param: reduceRatio: the ratio of points to reduce
        :param: maxNPnts: the maximum number of points on a facet
        :return: None

        author: weiwei
        date: 20161129, tsukuba
        """

        self.objsamplepnts_refcls = np.ndarray(shape=(self.facets.shape[0],), dtype=np.object)
        self.objsamplenrmls_refcls = np.ndarray(shape=(self.facets.shape[0],), dtype=np.object)
        for i, facet in enumerate(self.facets):
            self.objsamplepnts_refcls[i] = np.empty(shape=(0,0))
            self.objsamplenrmls_refcls[i] = np.empty(shape=(0,0))
            X = self.objsamplepnts_ref[i]
            nX = X.shape[0]
            if nX > reduceRatio:
                kmeans = KMeans(n_clusters=maxNPnts if nX/reduceRatio>maxNPnts else nX/reduceRatio, random_state=0).fit(X)
                self.objsamplepnts_refcls[i] = kmeans.cluster_centers_
                self.objsamplenrmls_refcls[i] = np.tile(self.facetnormals[i], [self.objsamplepnts_refcls.shape[0],1])

    def clusterFacetSamplesRNN(self, reduceRadius=3):
        """
        cluster the samples of each facet using radius nearest neighbours
        the cluster center and their correspondent normals will be saved
        in self.objsamplepnts_refcls and self.objsamplenrmals_refcls

        :param: reduceRadius: the neighbors that fall inside the reduceradius will be removed
        :return: None

        author: weiwei
        date: 20161130, osaka
        """

        self.objsamplepnts_refcls = np.ndarray(shape=(self.facets.shape[0],), dtype=np.object)
        self.objsamplenrmls_refcls = np.ndarray(shape=(self.facets.shape[0],), dtype=np.object)
        for i, facet in enumerate(self.facets):
            # print "cluster"
            # print i,len(self.facets)
            self.objsamplepnts_refcls[i] = []
            self.objsamplenrmls_refcls[i] = []
            X = self.objsamplepnts_ref[i]
            nX = X.shape[0]
            if nX > 0:
                neigh = RadiusNeighborsClassifier(radius=1.0)
                neigh.fit(X, range(nX))
                neigharrays = neigh.radius_neighbors(X, radius=reduceRadius, return_distance=False)
                delset = set([])
                for j in range(nX):
                    if j not in delset:
                        self.objsamplepnts_refcls[i].append(np.array(X[j]))
                        self.objsamplenrmls_refcls[i].append(np.array(self.objsamplenrmls_ref[i][j]))
                        # if self.objsamplepnts_refcls[i].size:
                        #     self.objsamplepnts_refcls[i] = np.vstack((self.objsamplepnts_refcls[i], X[j]))
                        #     self.objsamplenrmls_refcls[i] = np.vstack((self.objsamplenrmls_refcls[i],
                        #                                                 self.objsamplenrmls_ref[i][j]))
                        # else:
                        #     self.objsamplepnts_refcls[i] = np.array([])
                        #     self.objsamplenrmls_refcls[i] = np.array([])
                        #     self.objsamplepnts_refcls[i] = np.hstack((self.objsamplepnts_refcls[i], X[j]))
                        #     self.objsamplenrmls_refcls[i] = np.hstack((self.objsamplenrmls_refcls[i],
                        #                                                 self.objsamplenrmls_ref[i][j]))
                        delset.update(neigharrays[j].tolist())
            if self.objsamplepnts_refcls[i]:
                self.objsamplepnts_refcls[i] = np.vstack(self.objsamplepnts_refcls[i])
                self.objsamplenrmls_refcls[i] = np.vstack(self.objsamplenrmls_refcls[i])
            else:
                self.objsamplepnts_refcls[i] = np.empty(shape=(0,0))
                self.objsamplenrmls_refcls[i] = np.empty(shape=(0,0))

    def removeHndcc(self, base, discretesize=8):
        """
        Handcc means hand collision detection

        :param discretesize: the number of hand orientations
        :return:

        author: weiwei
        date: 20161212, tsukuba
        """

        # isplotted = 0

        # if self.rtq85plotlist:
        #     for rtq85plotnode in self.rtq85plotlist:
        #         rtq85plotnode.removeNode()
        # self.rtq85plotlist = []

        self.sucrotmats = []
        self.succontacts = []
        self.succontactnormals = []
        # collided
        self.sucrotmatscld = []
        self.succontactscld = []
        self.succontactnormalscld = []

        plotoffsetfp = 3

        self.counter = 0

        while self.counter < self.facets.shape[0]:
            # print str(self.counter) + "/" + str(self.facetpairs.shape[0]-1)
            # print self.gripcontactpairs_precc

            for i in range(self.objsamplepnts_refcls[self.counter].shape[0]):
                for angleid in range(discretesize):
                    cctpnt = self.objsamplepnts_refcls[self.counter][i] + plotoffsetfp * self.objsamplenrmls_refcls[self.counter][i]
                    # check torque resistance
                    print Vec3(cctpnt[0],cctpnt[1],cctpnt[2]).length()
                    if Vec3(cctpnt[0],cctpnt[1],cctpnt[2]).length() < self.torqueresist:
                        cctnrml = self.objsamplenrmls_refcls[self.counter][i]
                        rotangle = 360.0 / discretesize * angleid
                        tmphand = self.hand
                        tmphand.attachTo(cctpnt[0], cctpnt[1], cctpnt[2], cctnrml[0], cctnrml[1], cctnrml[2], rotangle)
                        hndbullnode = cd.genCollisionMeshMultiNp(tmphand.handnp, base.render)
                        result = self.bulletworld.contactTest(hndbullnode)
                        if not result.getNumContacts():
                            self.succontacts.append(self.objsamplepnts_refcls[self.counter][i])
                            self.succontactnormals.append(self.objsamplenrmls_refcls[self.counter][i])
                            self.sucrotmats.append(tmphand.getMat())
                        else:
                            self.succontactscld.append(self.objsamplepnts_refcls[self.counter][i])
                            self.succontactnormalscld.append(self.objsamplenrmls_refcls[self.counter][i])
                            self.sucrotmatscld.append(tmphand.getMat())
            self.counter+=1
        self.counter = 0

    def segShow(self, base, togglesamples=False, togglenormals=False,
                togglesamples_ref=False, togglenormals_ref=False,
                togglesamples_refcls=False, togglenormals_refcls=False, alpha = .1):
        """

        :param base:
        :param togglesamples:
        :param togglenormals:
        :param togglesamples_ref: toggles the sampled points that fulfills the dist requirements
        :param togglenormals_ref:
        :return:
        """

        nfacets = self.facets.shape[0]
        facetcolorarray = self.facetcolorarray

        # offsetf = facet
        # plot the segments
        plotoffsetf = .0
        for i, facet in enumerate(self.facets):
            geom = pandageom.packpandageom(self.objtrimesh.vertices+np.tile(plotoffsetf*i*self.facetnormals[i],
                                                                            [self.objtrimesh.vertices.shape[0],1]), \
                                           # -np.tile(self.objcenter,[self.objtrimesh.vertices.shape[0],1]),
                                           self.objtrimesh.face_normals[facet], self.objtrimesh.faces[facet])
            node = GeomNode('piece')
            node.addGeom(geom)
            star = NodePath('piece')
            star.attachNewNode(node)
            star.setColor(Vec4(facetcolorarray[i][0], facetcolorarray[i][1],
                               facetcolorarray[i][2], alpha))
            star.setTransparency(TransparencyAttrib.MAlpha)

            star.setTwoSided(True)
            star.reparentTo(base.render)
            # sampledpnts = samples[sample_idxes[i]]
            # for apnt in sampledpnts:
            #     pandageom.plotSphere(base, star, pos=apnt, radius=1, rgba=rgba)
            rgbapnts0 = [1,1,1,1]
            rgbapnts1 = [.5,.5,0,1]
            rgbapnts2 = [1,0,0,1]
            if togglesamples:
                for j, apnt in enumerate(self.objsamplepnts[i]):
                    pandageom.plotSphere(star, pos=apnt+plotoffsetf*i*self.facetnormals[i], radius=3, rgba=rgbapnts0)
            if togglenormals:
                for j, apnt in enumerate(self.objsamplepnts[i]):
                    pandageom.plotArrow(star, spos=apnt+plotoffsetf*i*self.facetnormals[i],
                                        epos=apnt + plotoffsetf*i*self.facetnormals[i] + self.objsamplenrmls[i][j],
                                        rgba=rgbapnts0, length=10)
            if togglesamples_ref:
                for j, apnt in enumerate(self.objsamplepnts_ref[i]):
                    pandageom.plotSphere(star, pos=apnt+plotoffsetf*i*self.facetnormals[i], radius=3, rgba=rgbapnts1)
            if togglenormals_ref:
                for j, apnt in enumerate(self.objsamplepnts_ref[i]):
                    pandageom.plotArrow(star, spos=apnt+plotoffsetf*i*self.facetnormals[i],
                                        epos=apnt + plotoffsetf*i*self.facetnormals[i] + self.objsamplenrmls_ref[i][j],
                                        rgba=rgbapnts1, length=10)
            if togglesamples_refcls:
                for j, apnt in enumerate(self.objsamplepnts_refcls[i]):
                    pandageom.plotSphere(star, pos=apnt+plotoffsetf*i*self.facetnormals[i], radius=3, rgba=rgbapnts2)
            if togglenormals_refcls:
                for j, apnt in enumerate(self.objsamplepnts_refcls[i]):
                    pandageom.plotArrow(star, spos=apnt+plotoffsetf*i*self.facetnormals[i],
                                        epos=apnt + plotoffsetf*i*self.facetnormals[i] + self.objsamplenrmls_refcls[i][j],
                                        rgba=rgbapnts2, length=10)

    def segShow2(self, base, togglesamples=False, togglenormals=False,
                togglesamples_ref=False, togglenormals_ref=False,
                togglesamples_refcls=False, togglenormals_refcls=False, specificface = True):
        """

        :param base:
        :param togglesamples:
        :param togglenormals:
        :param togglesamples_ref: toggles the sampled points that fulfills the dist requirements
        :param togglenormals_ref:
        :return:
        """

        nfacets = self.facets.shape[0]
        facetcolorarray = self.facetcolorarray

        rgbapnts0 = [1, 1, 1, 1]
        rgbapnts1 = [0.2, 0.7, 1, 1]
        rgbapnts2 = [1, 0.7, 0.2, 1]

        # offsetf = facet
        plotoffsetf = .0
        faceplotted = False
        # plot the segments
        for i, facet in enumerate(self.facets):
            if not specificface:
                geom = pandageom.packpandageom(self.objtrimesh.vertices+np.tile(plotoffsetf*i*self.facetnormals[i],
                                                                                [self.objtrimesh.vertices.shape[0],1]),
                                               self.objtrimesh.face_normals[facet], self.objtrimesh.faces[facet])
                node = GeomNode('piece')
                node.addGeom(geom)
                star = NodePath('piece')
                star.attachNewNode(node)
                star.setColor(Vec4(.77, .17, 0, 1))
                star.setTransparency(TransparencyAttrib.MAlpha)

                star.setTwoSided(True)
                star.reparentTo(base.render)
                # sampledpnts = samples[sample_idxes[i]]
                # for apnt in sampledpnts:
                #     pandageom.plotSphere(base, star, pos=apnt, radius=1, rgba=rgba)
                if togglesamples:
                    for j, apnt in enumerate(self.objsamplepnts[i]):
                        pandageom.plotSphere(star, pos=apnt+plotoffsetf*i*self.facetnormals[i], radius=2.8, rgba=rgbapnts0)
                if togglenormals:
                    for j, apnt in enumerate(self.objsamplepnts[i]):
                        pandageom.plotArrow(star, spos=apnt+plotoffsetf*i*self.facetnormals[i],
                                            epos=apnt + plotoffsetf*i*self.facetnormals[i] + self.objsamplenrmls[i][j],
                                            rgba=rgbapnts0, length=10)
                if togglesamples_ref:
                    for j, apnt in enumerate(self.objsamplepnts_ref[i]):
                        pandageom.plotSphere(star, pos=apnt+plotoffsetf*i*self.facetnormals[i], radius=2.9, rgba=rgbapnts1)
                if togglenormals_ref:
                    for j, apnt in enumerate(self.objsamplepnts_ref[i]):
                        pandageom.plotArrow(star, spos=apnt+plotoffsetf*i*self.facetnormals[i],
                                            epos=apnt + plotoffsetf*i*self.facetnormals[i] + self.objsamplenrmls_ref[i][j],
                                            rgba=rgbapnts1, length=10)
                if togglesamples_refcls:
                    for j, apnt in enumerate(self.objsamplepnts_refcls[i]):
                        pandageom.plotSphere(star, pos=apnt+plotoffsetf*i*self.facetnormals[i], radius=3, rgba=rgbapnts2)
                if togglenormals_refcls:
                    for j, apnt in enumerate(self.objsamplepnts_refcls[i]):
                        pandageom.plotArrow(star, spos=apnt+plotoffsetf*i*self.facetnormals[i],
                                            epos=apnt + plotoffsetf*i*self.facetnormals[i] + self.objsamplenrmls_refcls[i][j],
                                            rgba=rgbapnts2, length=10)
            if specificface:
                plotoffsetf = .1
                if faceplotted:
                    continue
                else:
                    if len(self.objsamplepnts[i])>25:
                        faceplotted = True
                        geom = pandageom.packpandageom(self.objtrimesh.vertices+np.tile(plotoffsetf*i*self.facetnormals[i],
                                                                                        [self.objtrimesh.vertices.shape[0],1]),
                                                       self.objtrimesh.face_normals[facet], self.objtrimesh.faces[facet])
                        node = GeomNode('piece')
                        node.addGeom(geom)
                        star = NodePath('piece')
                        star.attachNewNode(node)
                        star.setColor(Vec4(facetcolorarray[i][0], facetcolorarray[i][1], facetcolorarray[i][2], 1))
                        star.setTransparency(TransparencyAttrib.MAlpha)

                        star.setTwoSided(True)
                        star.reparentTo(base.render)
                        # sampledpnts = samples[sample_idxes[i]]
                        # for apnt in sampledpnts:
                        #     pandageom.plotSphere(base, star, pos=apnt, radius=1, rgba=rgba)
                        if togglesamples:
                            for j, apnt in enumerate(self.objsamplepnts[i]):
                                pandageom.plotSphere(star, pos=apnt+plotoffsetf*i*self.facetnormals[i], radius=2.8, rgba=rgbapnts0)
                        if togglenormals:
                            for j, apnt in enumerate(self.objsamplepnts[i]):
                                pandageom.plotArrow(star, spos=apnt+plotoffsetf*i*self.facetnormals[i],
                                                    epos=apnt + plotoffsetf*i*self.facetnormals[i] + self.objsamplenrmls[i][j],
                                                    rgba=rgbapnts0, length=10)
                        if togglesamples_ref:
                            for j, apnt in enumerate(self.objsamplepnts_ref[i]):
                                pandageom.plotSphere(star, pos=apnt+plotoffsetf*i*self.facetnormals[i], radius=3, rgba=rgbapnts1)
                        if togglenormals_ref:
                            for j, apnt in enumerate(self.objsamplepnts_ref[i]):
                                pandageom.plotArrow(star, spos=apnt+plotoffsetf*i*self.facetnormals[i],
                                                    epos=apnt + plotoffsetf*i*self.facetnormals[i] + self.objsamplenrmls_ref[i][j],
                                                    rgba=rgbapnts1, length=10)
                        if togglesamples_refcls:
                            for j, apnt in enumerate(self.objsamplepnts_refcls[i]):
                                pandageom.plotSphere(star, pos=apnt+plotoffsetf*i*self.facetnormals[i], radius=3.5, rgba=rgbapnts2)
                        if togglenormals_refcls:
                            for j, apnt in enumerate(self.objsamplepnts_refcls[i]):
                                pandageom.plotArrow(star, spos=apnt+plotoffsetf*i*self.facetnormals[i],
                                                    epos=apnt + plotoffsetf*i*self.facetnormals[i] + self.objsamplenrmls_refcls[i][j],
                                                    rgba=rgbapnts2, length=10)

if __name__=='__main__':

    base = pandactrl.World(camp=[700,300,700], lookatp=[0,0,0])

    handpkg = sdmbstipsd
    this_dir, this_filename = os.path.split(__file__)
    objpath = os.path.join(os.path.split(this_dir)[0]+os.sep, "grip", "objects", "sandpart2.stl")
    freesuctst = Freesuc(objpath, handpkg = handpkg, torqueresist = 500)
    print len(freesuctst.objtrimesh.faces)
    # freegriptst.objtrimesh.show()
    import time
    tic = time.clock()
    freesuctst.removeBadSamples(mindist = 5)
    # freegriptst.clusterFacetSamplesKNN(reduceRatio=15, maxNPnts=5)
    freesuctst.clusterFacetSamplesRNN(reduceRadius=10)
    # freesuctst.segShow2(base, togglesamples=True, togglenormals=False,
    #                     togglesamples_ref=True, togglenormals_ref=False,
    #                     togglesamples_refcls=True, togglenormals_refcls=False, specificface = True)
    # import pandaplotutils.pandageom as pg
    # pg.plotAxisSelf(base.render, Vec3(0,0,0))
    freesuctst.removeHndcc(base)
    toc = time.clock()
    print toc-tic

    # freesuctst.segShow(base, togglesamples=False, togglenormals=False,
    #                     togglesamples_ref=False, togglenormals_ref=False,
    #                     togglesamples_refcls=True, togglenormals_refcls=True, alpha = 1)
    objnp = pandageom.packpandanp(freesuctst.objtrimesh.vertices,
                           freesuctst.objtrimesh.face_normals,
                           freesuctst.objtrimesh.faces, name='')
    objnp.setColor(.37,.37,.35,1)
    objnp.reparentTo(base.render)
    for i, hndrot in enumerate(freesuctst.sucrotmats):
        if i == 1:
            tmphand = handpkg.newHandNM(hndcolor=[.7,.7,.7,.7])
            centeredrot = Mat4(hndrot)
            # centeredrot.setRow(3,hndrot.getRow3(3)-Vec3(freesuctst.objcenter[0], freesuctst.objcenter[1], freesuctst.objcenter[2]))
            tmphand.setMat(centeredrot)
            tmphand.reparentTo(base.render)
            tmphand.setColor(.5,.5,.5,.3)
    # for i, hndrot in enumerate(freesuctst.sucrotmatscld):
    #     tmphand = handpkg.newHandNM(hndcolor=[.7,.7,.7,.7])
    #     centeredrot = Mat4(hndrot)
    #     # centeredrot.setRow(3,hndrot.getRow3(3)-Vec3(freesuctst.objcenter[0], freesuctst.objcenter[1], freesuctst.objcenter[2]))
    #     tmphand.setMat(centeredrot)
    #     tmphand.reparentTo(base.render)
    #     tmphand.setColor(.7,.3,.3,1)

    # # object rotation
    # import math
    # from trimesh import transformations as tf
    # from pandaplotutils import pandageom as pg
    # rpymat = tf.euler_matrix(math.degrees(10),0,0)
    # rpypdmat = pg.cvtMat4(rpymat)
    # objmnp = pandageom.genObjmnp(objpath)
    # objmnp.setMat(objmnp.getMat()*rpypdmat)
    # objmnp.reparentTo(base.render)
    # for i, hndrot in enumerate(freesuctst.sucrotmats):
    #     tmphand = handpkg.newHandNM(hndcolor=[.7,.7,.7,.3])
    #     centeredrot = Mat4(hndrot)
    #     # centeredrot.setRow(3,hndrot.getRow3(3)-Vec3(freesuctst.objcenter[0], freesuctst.objcenter[1], freesuctst.objcenter[2]))
    #     tmphand.setMat(centeredrot*rpypdmat)
    #     tmphand.reparentTo(base.render)
    #
    # import csv
    # file = open('sandmedia_cvt.csv', 'wb')
    # writer = csv.writer(file)
    # writer.writerow(['Autopick'])
    # writer.writerow(['Version 0.11'])
    # writer.writerow(['See slides for hand base, hand coordinates'])
    # writer.writerow(['HandX', 'Y', 'Z', 'RX', 'RY', 'RZ', '#Tool', 'WorkX', 'Y', 'Z','RX', 'RY', 'RZ', '#Model', 'HandRotmat4'])
    #
    # import math
    # for i, hndrot in enumerate(freesuctst.sucrotmats):
    #     tmphand = handpkg.newHandNM(hndcolor=[.7,.7,.7,.7])
    #     centeredrot = Mat4(hndrot)
    #     # centeredrot.setRow(3,hndrot.getRow3(3)-Vec3(freesuctst.objcenter[0], freesuctst.objcenter[1], freesuctst.objcenter[2]))
    #     tmphand.setMat(centeredrot)
    #     rotmatnp = pg.mat3ToNp(tmphand.getMat().getUpper3())
    #     rpyangles3 = tf.euler_from_matrix(rotmatnp, 'sxyz')
    #     writer.writerow([str(centeredrot.getRow3(3)[0]),str(centeredrot.getRow3(3)[1]), str(centeredrot.getRow3(3)[2]), \
    #                      str(math.degrees(rpyangles3[0])), str(math.degrees(rpyangles3[1])), str(math.degrees(rpyangles3[2])),\
    #                      'T0','0','0','0','0','0','0','M0', dc.mat4ToStr(centeredrot)])
    # file.close()
    #
    # def updateshow0(freegriptst, task):
    #     npc = base.render.findAllMatches("**/piece")
    #     for np in npc:
    #         np.removeNode()
    #     freegriptst.segShow2(base, togglesamples=True, togglenormals=False,
    #                         togglesamples_ref=False, togglenormals_ref=False,
    #                         togglesamples_refcls=False, togglenormals_refcls=False, specificface = True)
    #     freegriptst.segShow(base, togglesamples=False, togglenormals=False,
    #                         togglesamples_ref=False, togglenormals_ref=False,
    #                         togglesamples_refcls=False, togglenormals_refcls=False)
    #     return task.done
    #
    # def updateshow1(freegriptst, task):
    #     npc = base.render.findAllMatches("**/piece")
    #     for np in npc:
    #         np.removeNode()
    #     freegriptst.segShow2(base, togglesamples=True, togglenormals=False,
    #                         togglesamples_ref=True, togglenormals_ref=False,
    #                         togglesamples_refcls=False, togglenormals_refcls=False, specificface = True)
    #     freegriptst.segShow(base, togglesamples=False, togglenormals=False,
    #                         togglesamples_ref=False, togglenormals_ref=False,
    #                         togglesamples_refcls=False, togglenormals_refcls=False)
    #     return task.done
    #
    # def updateshow2(freegriptst, task):
    #     np = base.render.find("**/piece")
    #     if np:
    #         np.removeNode()
    #     freegriptst.segShow2(base, togglesamples=True, togglenormals=False,
    #                         togglesamples_ref=True, togglenormals_ref=False,
    #                         togglesamples_refcls=True, togglenormals_refcls=False, specificface = True)
    #     freegriptst.segShow(base, togglesamples=False, togglenormals=False,
    #                         togglesamples_ref=False, togglenormals_ref=False,
    #                         togglesamples_refcls=False, togglenormals_refcls=False)
    #     return task.done
    #
    # taskMgr.doMethodLater(10, updateshow0, "tickTask", extraArgs=[freegriptst], appendTask=True)
    # taskMgr.doMethodLater(20, updateshow1, "tickTask", extraArgs=[freegriptst], appendTask=True)
    # taskMgr.doMethodLater(30, updateshow2, "tickTask", extraArgs=[freegriptst], appendTask=True)
    # base.run()

    # def updateshow(task):
    #     freegriptst.pairShow(base, togglecontacts=True, togglecontactnormals=True)
    #     print task.delayTime
    #     if abs(task.delayTime-13) < 1:
    #         task.delayTime -= 12.85
    #     return task.again
    # taskMgr.doMethodLater(1, updateshow, "tickTask")

    # geom = None
    # for i, faces in enumerate(freegriptst.objtrimesh.facets()):
    #     rgba = [np.random.random(),np.random.random(),np.random.random(),1]
    #     # geom = pandageom.packpandageom(freegriptst.objtrimesh.vertices, freegriptst.objtrimesh.face_normals[faces], freegriptst.objtrimesh.faces[faces])
    #     # compute facet normal
    #     facetnormal = np.sum(freegriptst.objtrimesh.face_normals[faces], axis=0)
    #     facetnormal = facetnormal/np.linalg.norm(facetnormal)
    #     geom = pandageom.packpandageom(freegriptst.objtrimesh.vertices +
    #                             np.tile(0 * facetnormal,
    #                                     [freegriptst.objtrimesh.vertices.shape[0], 1]),
    #                             freegriptst.objtrimesh.face_normals[faces],
    #                             freegriptst.objtrimesh.faces[faces])
    #     node = GeomNode('piece')
    #     node.addGeom(geom)
    #     star = NodePath('piece')
    #     star.attachNewNode(node)
    #     star.setColor(Vec4(rgba[0],rgba[1],rgba[2],rgba[3]))
    #     # star.setColor(Vec4(.7,.4,0,1))
    #     star.setTwoSided(True)
    #     star.reparentTo(base.render)
        # sampledpnts = samples[sample_idxes[i]]
        # for apnt in sampledpnts:
        #     pandageom.plotSphere(base, star, pos=apnt, radius=1, rgba=rgba)
        # for j, apnt in enumerate(freegriptst.objsamplepnts[i]):
        #     pandageom.plotSphere(base, star, pos=apnt, radius=0.7, rgba=rgba)
        #     pandageom.plotArrow(base, star, spos=apnt, epos=apnt+freegriptst.objsamplenrmls[i][j], rgba=[1,0,0,1], length=5, thickness=0.1)
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
    base.run()
