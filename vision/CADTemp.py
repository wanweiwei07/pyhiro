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

    def __init__(self, ompath, numpointsoververts=10):
        """

        :param ompath:
        :param numpointsoververts: the density of sampling, the total count is numpointsoververts*nverts

        :return an array of points

        author: weiwei
        date: 20170713
        """
        self.objtrimesh = trimesh.load_mesh(ompath)

        nverts = self.objtrimesh.vertices.shape[0]
        samples = trimesh.sample.sample_surface_even(mesh = self.objtrimesh, count = nverts*numpointsoververts)
        self.__samplestemp = np.append(samples, self.objtrimesh.vertices, axis = 0)

    @property
    def pcdtemp(self):
        return self.__samplestemp