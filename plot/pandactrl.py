from panda3d.core import *
from direct.filter.CommonFilters import CommonFilters
from direct.showbase.ShowBase import ShowBase
import os
import numpy as np


def setRenderEffect(base):
    """
    Set a cartoonink shader and the background color etc to base

    ## input:
        a showbase object

    author: weiwei
    date: 20160616
    """
    try:
        # setbgcolor must be done before setting shader
        base.setBackgroundColor(1, 1, 1)

        # base.render.setAttrib(LightRampAttrib.makeSingleThreshold(0.5, 0.4))
        # base.render.setShaderAuto()
        base.separation = 1 # Pixels
        base.filters = CommonFilters(base.win, base.cam)
        base.filters.setCartoonInk(separation=base.separation)

        base.loadPrcFileData("", "framebuffer-multisample 1")
        base.loadPrcFileData('', 'multisamples 4')
        base.render.setAntialias(AntialiasAttrib.MAuto)
    except:
        pass


def setNodeCartoon(nodepath):
    """
    Set a cartoon-style shader, set nodepath
    TODO: not working now (you have to implement an independant shader

    ## input:
    base
        a showbase object
    nodepath
        a pand3d nodepath

    author: weiwei
    date: 20160620
    """
    try:
        # setbgcolor must be done before setting shader
        nodepath.setBackgroundColor(1, 1, 1)

        # nodepath.setAttrib(LightRampAttrib.makeSingleThreshold(0.5, 0.4))
        # nodepath.setShaderAuto(Glow=True)
        # nodepath.filters = CommonFilters(base.win, base.cam)
        # nodepath.filters.setCartoonInk(separation=1)
    except:
        pass


def setLight(base):
    """
    Set simple light style
    background: white
    ambient light: 0.7, 0.7, 0.7

    ## input:
        a showbase object

    author: weiwei
    date: 20160616
    """

    try:
        ablight = AmbientLight("ambientlight")
        ablight.setColor(Vec4(0.1, 0.1, 0.1, 1))
        ablightnode = base.render.attachNewNode(ablight)
        base.render.setLight(ablightnode)
        ptlight0 = PointLight("pointlight0")
        ptlight0.setColor(VBase4(1, 1, 1, 1))
        ptlightnode0 = base.render.attachNewNode(ptlight0)
        ptlightnode0.setPos(500, 0, 0)
        base.render.setLight(ptlightnode0)
        ptlight1 = PointLight("pointlight1")
        ptlight1.setColor(VBase4(1, 1, 1, 1))
        ptlightnode1 = base.render.attachNewNode(ptlight1)
        ptlightnode1.setPos(0, 500, 500)
        base.render.setLight(ptlightnode1)
    except:
        pass


def setCam(base, posx, posy, posz, view=None):
    """
    Set the position of cam
    set the cam lookAt 0,0,0

    ## input
    base:
        a showbase object
    posx, posy, posz:
        the position of the cam

    note:
        the operation should be done on base.cam, not base.camera

    author: weiwei
    date: 20160618
    """

    try:
        lens = None
        if view is None or view is 'orthogonal':
            # set orthogonal perspective
            lens = OrthographicLens()
            lens.setFilmSize(2, 2)
            # base.trackball.node().setOrigin(Vec3(0, 0, 0))
        elif view is 'perspective':
            lens = PerspectiveLens()

        base.cam.setPos(posx, posy, posz)
        base.cam.lookAt(0, 0, 0)
        aspect_ratio = base.win.getAspectRatio()
        lens.set_aspect_ratio(aspect_ratio)
        base.cam.node().setLens(lens)
    except:
        pass

# def onWindowEvent(window):
#     width = base.win.getProperties().getXSize()
#     height = base.win.getProperties().getYSize()
#     base.cam.node().getLens().setFilmSize(width, height)