from panda3d.core import *
from direct.task import Task
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
        # base.separation = 1 # Pixels
        # base.filters = CommonFilters(base.win, base.cam)
        # base.filters.setCartoonInk(separation=base.separation)

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
        ablight.setColor(Vec4(0.3, 0.3, 0.3, 1))
        ablightnode = base.render.attachNewNode(ablight)
        base.render.setLight(ablightnode)
        ptlight0 = PointLight("pointlight0")
        ptlight0.setColor(VBase4(0.2, 0.2, 0.2, 1))
        ptlightnode0 = base.render.attachNewNode(ptlight0)
        ptlightnode0.setPos(500, 0, 500)
        base.render.setLight(ptlightnode0)
        ptlight1 = PointLight("pointlight1")
        ptlight1.setColor(VBase4(0.2, 0.2, 0.2, 1))
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

    def setAspect(base, task):
        aspectRatio = base.getAspectRatio()
        base.cam.node().getLens().setAspectRatio(aspectRatio)
        return task.cont

    def mouseMove(base, params, task):
        if params['m1down']:
            mw = base.mouseWatcherNode
            hasMouse = mw.hasMouse()
            if hasMouse:
                m2downmpos = Vec2(mw.getMouseX()*2*base.camera.getPos().length(),mw.getMouseY()*2*base.camera.getPos().length())
                m2downmposworld = Vec3(base.render.getRelativePoint(base.camera, Vec3(m2downmpos[0], 0, m2downmpos[1])))
                m2downmposworld.normalize()
                rotatevec = m2downmposworld.cross(params['m1downposinworld'])
                rotateangle = m2downmposworld.signedAngleDeg(params['m1downposinworld'], rotatevec)
                if rotateangle > 2 or rotateangle < -2:
                    # print base.camera.getHpr()
                    base.camera.setPos(Mat3.rotateMat(rotateangle, rotatevec).xform(base.camera.getPos()))
                    base.camera.lookAt(0,0,0)
        return task.cont

    def mouse1Down(ispressdown, base, params):
        if ispressdown:
            params['m1down'] = True
            mw = base.mouseWatcherNode
            hasMouse = mw.hasMouse()
            if hasMouse:
                params['m1downpos'] = Vec2(mw.getMouseX()*2*base.camera.getPos().length(), mw.getMouseY()*2*base.camera.getPos().length())
                params['m1downposinworld'] = Vec3(base.render.getRelativePoint(base.camera, Vec3(params['m1downpos'][0], 0, params['m1downpos'][1])))
                params['m1downposinworld'].normalize()
                # print params['m2downposinworld']
        else:
            params['m1down'] = False

    def mouseWheel(isrollup, base):
        """
        The mouse wheel moved. If direction is True, then it went up.
        """
        forward = base.camera.getNetTransform().getMat().getRow3(1)
        forward.normalize()
        if isrollup:
            # zoom in\
            if base.camera.getPos().length() > 100:
                newpos = base.camera.getPos() + forward*35
                base.camera.setPos(newpos[0], newpos[1], newpos[2])
            pass
        else:
            # zoom out
            if base.camera.getPos().length() < 500:
                newpos = base.camera.getPos() - forward*35
                base.camera.setPos(newpos[0], newpos[1], newpos[2])

    try:
        lens = None
        if view is None or view is 'orthogonal':
            lens = OrthographicLens()
        elif view is 'perspective':
            lens = PerspectiveLens()

        # base.cam.setPos(posx, posy, posz)
        # base.cam.lookAt(0,0,0)
        # base.cam.node().setLens(lens)
        # base.taskMgr.add(setaspect, "setaspect")
        # own implementation
        base.disableMouse()
        base.camera.setPos(posx, posy, posz)
        base.camera.lookAt(0,0,0)
        lens.setNear(0.01)
        base.cam.node().setLens(lens)

        params = {'m1down':False, 'm1downpos':Vec2(0,0), 'm1downposinworld':Vec3(0,0,0)}
        base.accept("mouse1", mouse1Down, [True, base, params])
        base.accept("mouse1-up", mouse1Down, [False, base, params])
        base.accept("wheel_up", mouseWheel, [True, base])
        base.accept("wheel_down", mouseWheel, [False, base])
        base.taskMgr.add(mouseMove, "mousemove", extraArgs=[base, params], appendTask=True)
        # TODO: check if there is an accept event for window-event
        base.taskMgr.add(setAspect, "setaspect", extraArgs=[base], appendTask=True)
    except:
        print "Error Set cam"
        pass

# def onWindowEvent(window):
#     width = base.win.getProperties().getXSize()
#     height = base.win.getProperties().getYSize()
#     base.cam.node().getLens().setFilmSize(width, height)