#!/usr/bin/python

import os
import math
import numpy as np
import trimesh
import plot.pandageom as ppg
from direct.showbase.ShowBase import ShowBase
from panda3d.core import *
import plot.pandactrl as pandactrl
import plot.pandageom as pandageom
from geomutils import robotmath
from shapely.geometry import Polygon
from shapely.geometry import Point
import matplotlib.pyplot as plt

def _genHand(pandabase, jawwidth):
    '''
    load the robotiq85 model and return a nodepath

    ## input
    pandabase:
        the showbase() object
    jawwidth:
        the distance between fingertips

    ## output
    rtq85np:
        the nodepath of this rtq85 hand

    author: weiwei
    date: 20160627
    '''
    this_dir, this_filename = os.path.split(__file__)
    rtq85basepath = os.path.join(this_dir, "rtq85egg", "robotiq_85_base_link.egg")
    rtq85fingerpath = os.path.join(this_dir, "rtq85egg", "robotiq_85_finger_link.egg")
    rtq85fingertippath = os.path.join(this_dir, "rtq85egg", "robotiq_85_finger_tip_link.egg")
    rtq85innerknucklepath = os.path.join(this_dir, "rtq85egg", "robotiq_85_inner_knuckle_link.egg")
    rtq85knucklepath = os.path.join(this_dir, "rtq85egg", "robotiq_85_knuckle_link.egg")

    rtq85hnd = NodePath("rtq85hnd")
    rtq85base = NodePath("rtq85base")
    rtq85lknuckle = NodePath("rtq85lknuckle")
    rtq85rknuckle = NodePath("rtq85rknuckle")
    rtq85lfgr = NodePath("rtq85lfgr")
    rtq85rfgr = NodePath("rtq85rfgr")
    rtq85ilknuckle = NodePath("rtq85ilknuckle")
    rtq85irknuckle = NodePath("rtq85irknuckle")
    rtq85lfgrtip = NodePath("rtq85lfgrtip")
    rtq85rfgrtip = NodePath("rtq85rfgrtip")

    rtq85_basel = pandabase.loader.loadModel(rtq85basepath)
    rtq85_fingerl = pandabase.loader.loadModel(rtq85fingerpath)
    rtq85_fingertipl = pandabase.loader.loadModel(rtq85fingertippath)
    rtq85_innerknucklel = pandabase.loader.loadModel(rtq85innerknucklepath)
    rtq85_knucklel = pandabase.loader.loadModel(rtq85knucklepath)

    # base
    rtq85_basel.instanceTo(rtq85base)
    rtq85base.setPos(0,0,0)
    rtq85base.setColor(1,0,0,0.3)
    rtq85base.setTransparency(TransparencyAttrib.MAlpha)

    # left and right outer knuckle
    rtq85_knucklel.instanceTo(rtq85lknuckle)
    rtq85lknuckle.setPos(-3.060114443, 5.490451627, 0)
    rtq85lknuckle.setHpr(0, 0, 180)
    rtq85lknuckle.setColor(1,1,1,1)
    rtq85lknuckle.reparentTo(rtq85base)
    rtq85_knucklel.instanceTo(rtq85rknuckle)
    rtq85rknuckle.setPos(3.060114443, 5.490451627, 0)
    rtq85rknuckle.setHpr(0, 0, 0)
    rtq85rknuckle.setColor(1,1,1,1)
    rtq85rknuckle.reparentTo(rtq85base)

    # left and right finger
    rtq85_fingerl.instanceTo(rtq85lfgr)
    rtq85lfgr.setPos(3.148504435, -0.408552455, 0)
    rtq85lfgr.setColor(1,1,0,1)
    rtq85lfgr.reparentTo(rtq85lknuckle)
    rtq85_fingerl.instanceTo(rtq85rfgr)
    rtq85rfgr.setPos(3.148504435, -0.408552455, 0)
    rtq85rfgr.setColor(1,1,0,1)
    rtq85rfgr.reparentTo(rtq85rknuckle)

    # left and right inner knuckle
    rtq85_innerknucklel.instanceTo(rtq85ilknuckle)
    rtq85ilknuckle.setPos(-1.27, 6.142, 0)
    rtq85ilknuckle.setHpr(0, 0, 180)
    rtq85ilknuckle.setColor(1,1,0,1)
    rtq85ilknuckle.reparentTo(rtq85base)
    rtq85_innerknucklel.instanceTo(rtq85irknuckle)
    rtq85irknuckle.setPos(1.27, 6.142, 0)
    rtq85irknuckle.setHpr(0, 0, 0)
    rtq85irknuckle.setColor(1,1,0,1)
    rtq85irknuckle.reparentTo(rtq85base)

    # left and right fgr tip
    rtq85_fingertipl.instanceTo(rtq85lfgrtip)
    rtq85lfgrtip.setPos(3.759940821, 4.303959807, 0)
    rtq85lfgrtip.setColor(1,0,1,1)
    rtq85lfgrtip.reparentTo(rtq85ilknuckle)
    rtq85_fingertipl.instanceTo(rtq85rfgrtip)
    rtq85rfgrtip.setPos(3.759940821, 4.303959807, 0)
    rtq85rfgrtip.setHpr(0, 0, 0)
    rtq85rfgrtip.setColor(1,0,1,1)
    rtq85rfgrtip.reparentTo(rtq85irknuckle)

    rtq85base.reparentTo(rtq85hnd)

    return rtq85hnd

if __name__=='__main__':

    base = ShowBase()
    rtq85hnd = _genHand(base, 0)
    rtq85hnd.reparentTo(base.render)
    pandactrl.setRenderEffect(base)
    pandactrl.setLight(base)
    pandactrl.setCam(base, 0, 500, 500, 'perspective')
    base.run()