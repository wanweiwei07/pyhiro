from panda3d.core import *
from direct.showbase.ShowBase import ShowBase
import os
import numpy as np


def packpandageom(vertices, facenormals, triangles, name=''):
    """
    package the vertices and triangles into a panda3d geom

    ## input
    vertices:
        a n-by-3 nparray, each row is a vertex
    facenormals:
        a n-by-3 nparray, each row is the normal of a face
    triangles:
        a n-by-3 nparray, each row is three idx to the vertices
    name:
        not as important

    ## output
    geom
        a Geom model which is ready to be added to a node

    author: weiwei
    date: 20160613
    """

    format = GeomVertexFormat.getV3n3()
    vertexData = GeomVertexData(name, format, Geom.UHStatic)
    vertexData.setNumRows(vertices.shape[0])
    vertwritter = GeomVertexWriter(vertexData, 'vertex')
    normalwritter = GeomVertexWriter(vertexData, 'normal')
    primitive = GeomTriangles(Geom.UHStatic)
    for i,fvidx in enumerate(triangles):
        vert0 = vertices[fvidx[0],:]
        vert1 = vertices[fvidx[1],:]
        vert2 = vertices[fvidx[2],:]
        vertwritter.addData3f(vert0[0], vert0[1], vert0[2])
        normalwritter.addData3f(facenormals[i,0], facenormals[i,1], facenormals[i,2])
        vertwritter.addData3f(vert1[0], vert1[1], vert1[2])
        normalwritter.addData3f(facenormals[i,0], facenormals[i,1], facenormals[i,2])
        vertwritter.addData3f(vert2[0], vert2[1], vert2[2])
        normalwritter.addData3f(facenormals[i,0], facenormals[i,1], facenormals[i,2])
        primitive.addVertex(i*3)
        primitive.addVertex(i*3+1)
        primitive.addVertex(i*3+2)
    geom = Geom(vertexData)
    geom.addPrimitive(primitive)

    return geom


def _genArrow(pandabase, length, thickness = 0.005):
    """
    Generate a arrow node for plot
    This function should not be called explicitly

    ## input
    pandabase:
        the panda direct.showbase.ShowBase object
    length:
        length of the arrow
    thickness:
        thickness of the arrow, set to 0.005 as default

    ## output

    """

    this_dir, this_filename = os.path.split(__file__)
    cylinderpath = os.path.join(this_dir, "geomprim", "cylinder.egg")
    conepath = os.path.join(this_dir, "geomprim", "cone.egg")

    arrow = NodePath("arrow")
    arrowbody = pandabase.loader.loadModel(cylinderpath)
    arrowhead = pandabase.loader.loadModel(conepath)
    arrowbody.setPos(0,0,0)
    arrowbody.setScale(thickness, length, thickness)
    arrowbody.reparentTo(arrow)
    arrowhead.setPos(arrow.getX(), length, arrow.getZ())
    # set scale (consider relativitly)
    arrowhead.setScale(thickness*2, thickness*9, thickness*2)
    arrowhead.reparentTo(arrow)

    return arrow


def plotArrow(pandabase, nodepath = None, spos = None, epos = None, length = None, thickness = 0.01, rgba=None):
    """
    plot an arrow to nodepath

    ## input:
    pandabase:
        the panda direct.showbase.ShowBase object
        will be sent to _genArrow
    nodepath:
        defines which parent should the arrow be attached to
    spos:
        1-by-3 nparray or list, starting position of the arrow
    epos:
        1-by-3 nparray or list, goal position of the arrow
    length:
        will be sent to _genArrow, if length is None, its value will be computed using np.linalg.norm(epos-spos)
    thickness:
        will be sent to _genArrow
    rgba:
        1-by-3 nparray or list

    author: weiwei
    date: 20160616
    """

    if nodepath is None:
        nodepath = pandabase.render
    if spos is None:
        spos = np.array([0,0,0])
    if epos is None:
        epos = np.array([0,0,1])
    if length is None:
        length = np.linalg.norm(epos-spos)
    if rgba is None:
        rgba = np.array([1,1,1,1])

    arrow = _genArrow(pandabase, length, thickness)
    arrow.setPos(spos[0], spos[1], spos[2])
    arrow.lookAt(epos[0], epos[1], epos[2])
    # lookAt points y+ to epos, use the following command to point x+ to epos
    # http://stackoverflow.com/questions/15126492/panda3d-how-to-rotate-object-so-that-its-x-axis-points-to-a-location-in-space
    # arrow.setHpr(arrow, Vec3(0,0,90))
    arrow.setColor(rgba[0], rgba[1], rgba[2], rgba[3])

    arrow.reparentTo(nodepath)


def plotFrame(pandabase, nodepath = None, spos = None, epos = None, length = None, thickness = 0.01, rgba=None):
    """
    plot an arrow to nodepath

    ## input:
    pandabase:
        the panda direct.showbase.ShowBase object
        will be sent to _genArrow
    nodepath:
        defines which parent should the arrow be attached to
    spos:
        1-by-3 nparray or list, starting position of the arrow
    epos:
        1-by-3 nparray or list, goal position of the arrow
    length:
        will be sent to _genArrow, if length is None, its value will be computed using np.linalg.norm(epos-spos)
    thickness:
        will be sent to _genArrow
    rgba:
        1-by-3 nparray or list

    author: weiwei
    date: 20160616
    """

    if nodepath is None:
        nodepath = pandabase.render
    if spos is None:
        spos = np.array([0,0,0])
    if epos is None:
        epos = np.array([0,0,1])
    if length is None:
        length = np.linalg.norm(epos-spos)
    if rgba is None:
        rgba = np.array([1,1,1,1])

    arrow = _genArrow(pandabase, length, thickness)
    arrow.setPos(spos[0], spos[1], spos[2])
    arrow.lookAt(epos[0], epos[1], epos[2])
    # lookAt points y+ to epos, use the following command to point x+ to epos
    # http://stackoverflow.com/questions/15126492/panda3d-how-to-rotate-object-so-that-its-x-axis-points-to-a-location-in-space
    # arrow.setHpr(arrow, Vec3(0,0,90))
    arrow.setColor(rgba[0], rgba[1], rgba[2], rgba[3])

    arrow.reparentTo(nodepath)

if __name__=="__main__":

    # show in panda3d
    from direct.showbase.ShowBase import ShowBase
    from panda3d.core import *

    base = ShowBase()
    # base.setBackgroundColor(0,0,0)
    # arrow = genArrow(base, 0.05)
    arrow = base.loader.loadModel("./geomprim/cylinder.egg")
    arrow.setPos(0,0,0)
    arrow.reparentTo(base.render)
    # arrowpholder = base.render.attachNewNode("arrow")
    # arrowpholder.setPos(0,0,0)
    # arrow.instanceTo(arrowpholder)
    base.camLens.setNearFar(0.01, 40.0)
    base.camLens.setFov(45.0)
    base.disableMouse()
    base.camera.setPos(0, 5, 15)
    base.camera.lookAt(0, 0, 0)
    # base.camera.setPos(0, -10, 0)
    # base.camera.lookAt(arrow)
    # print base.camera.getPos()
    # base.oobe()'

    import plot.pandactrl as pandactrl
    # pandactrl.setRenderEffect(base)
    pandactrl.setLight(base)

    base.run()