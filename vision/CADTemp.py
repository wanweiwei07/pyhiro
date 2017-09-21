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
import pandaplotutils.pandageom as pg
from panda3d.bullet import BulletWorld
from utils import collisiondetection as cd
import trimesh
from utils import robotmath
from utils import dbcvt as dc
import manipulation.suction.sandmmbs.sdmbs as sdmbstipsd

class CADTemp(object):

    def __init__(self, ompath, density = 4.0):
        """
        generate point cloud template for cad model

        :param ompath:
        :param numpointsoververts: the density of sampling, the total count is numpointsoververts*nverts
                it could be 'auto' if decide automatically

        :return an array of points

        author: weiwei
        date: 20170713
        """
        self.objtrimesh = trimesh.load_mesh(ompath)

        # decide the number of samples considering surface area
        area = self.objtrimesh.area_faces
        area_sum = np.sum(area)
        # 1 point every 4by4
        nsample = area_sum/(density*density)
        samples, faceid = trimesh.sample.sample_surface_even_withfaceid(
            mesh = self.objtrimesh, count = nsample)
        self.__samplestemp = np.append(samples, self.objtrimesh.vertices, axis = 0)
        self.__samplestempnoverts = samples
        self.__samplestempnovertsnormals = self.objtrimesh.face_normals[faceid]

        # for computing the inner samples
        self.bulletworldray = BulletWorld()
        self.__objgeom = pg.packpandageom(self.objtrimesh.vertices, self.objtrimesh.face_normals, self.objtrimesh.faces)
        self.__objmeshbullnode = cd.genCollisionMeshGeom(self.__objgeom)
        self.bulletworldray.attachRigidBody(self.__objmeshbullnode)

        # remove inner
        # use the inner samples and normals for ppf match
        self.__newsamplesnoverts = []
        self.__newsamplesnovertsnormals = []
        self.__removeinnersamples()

    @property
    def pcdtemp(self):
        """
        pcdtemp is designed for icp

        :return:
        """
        return self.__samplestemp

    @property
    def pcdtempnoverts(self):
        """
        it is preferrable to use pcdtmpnoverts for ppfmatch
        pcdtempnovertsnormals saves the normal of each thing
        :return: an array of points

        author: weiwei
        date: 20170715
        """
        return self.__samplestempnoverts

    @property
    def pcdtempnovertsnormals(self):
        """
        it is preferrable to use pcdtmpnoverts for ppfmatch
        pcdtempnovertsnormals saves the normal of each thing
        :return: an array of normals corresponding to pcdtempnoverts

        author: weiwei
        date: 20170715
        """
        return np.array(self.__samplestempnovertsnormals)

    @property
    def pcdtempnovertsinner(self):
        """
        only the samples pointing outside are kept
        :return: an array of points

        author: weiwei
        date: 20170802
        """
        return np.array(self.__newsamplesnoverts)

    @property
    def pcdtempnovertsnormalsinner(self):
        """
        only the samples pointing outside are kept
        :return: an array of points

        author: weiwei
        date: 20170802
        """
        return self.__newsamplesnovertsnormals

    def __removeinnersamples(self):
        """
        remove the samples whose normals collide with the mesh
        (approximately removes the inner samples)

        :return:

        author: weiwei
        date: 20170802
        """

        for i, sample in enumerate(self.__samplestempnoverts):
            normal = self.__samplestempnovertsnormals[i]
            pFrom = Point3(sample[0], sample[1], sample[2])
            pTo = pFrom+Vec3(normal[0], normal[1], normal[2])*9999
            result = self.bulletworldray.rayTestClosest(pFrom, pTo)
            if not result.hasHit():
                self.__newsamplesnoverts.append(sample)
                self.__newsamplesnovertsnormals.append(normal)
            else:
                continue


if __name__=='__main__':

    base = pandactrl.World(camp=[0,0,300], lookatp=[0,0,0])

    this_dir, this_filename = os.path.split(__file__)
    objpath = os.path.join(this_dir, "models", "ttube.stl")
    cadtemp = CADTemp(ompath = objpath)

    objnp = pg.packpandanp(cadtemp.objtrimesh.vertices,
                           cadtemp.objtrimesh.face_normals,
                           cadtemp.objtrimesh.faces,
                           name='')
    # objnp.reparentTo(base.render)

    colors = []
    color = [1,0,0,1]
    for pnt in cadtemp.pcdtempnovertsinner:
        colors.append(color)
    pntsnp = pg.genPntsnp(cadtemp.pcdtempnovertsinner, colors=colors)
    pntsnp.reparentTo(base.render)
    for i, normal in enumerate(cadtemp.pcdtempnovertsnormalsinner):
        pg.plotArrow(base.render, spos = cadtemp.pcdtempnovertsinner[i],
                     epos = normal+cadtemp.pcdtempnovertsinner[i], thickness = .2, length =10)

    base.run()