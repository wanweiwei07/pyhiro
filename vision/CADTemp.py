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

class CADTemp(object):

    def __init__(self, ompath):
        """

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
        nsample = area_sum/16
        if nsample > 200:
            nsample = 200
        samples, faceid = trimesh.sample.sample_surface_even_withfaceid(
            mesh = self.objtrimesh, count = nsample)
        self.__samplestemp = np.append(samples, self.objtrimesh.vertices, axis = 0)
        self.__samplestempnoverts = samples
        self.__samplestempnovertsnormals = self.objtrimesh.face_normals[faceid]

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
        return self.__samplestempnovertsnormals