import numpy as np
import os
from panda3d.core import *

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

    # vertformat = GeomVertexFormat.getV3()
    vertformat = GeomVertexFormat.getV3n3()
    # vertformat = GeomVertexFormat.getV3n3c4()
    vertexdata = GeomVertexData(name, vertformat, Geom.UHStatic)
    vertexdata.setNumRows(triangles.shape[0]*3)
    vertwritter = GeomVertexWriter(vertexdata, 'vertex')
    normalwritter = GeomVertexWriter(vertexdata, 'normal')
    # colorwritter = GeomVertexWriter(vertexdata, 'color')
    primitive = GeomTriangles(Geom.UHStatic)
    for i,fvidx in enumerate(triangles):
        vert0 = vertices[fvidx[0],:]
        vert1 = vertices[fvidx[1],:]
        vert2 = vertices[fvidx[2],:]
        vertwritter.addData3f(vert0[0], vert0[1], vert0[2])
        normalwritter.addData3f(facenormals[i,0], facenormals[i,1], facenormals[i,2])
        # print vert0[0], vert0[1], vert0[2]
        # print facenormals[i,0], facenormals[i,1], facenormals[i,2]
        # colorwritter.addData4f(1,1,1,1)
        vertwritter.addData3f(vert1[0], vert1[1], vert1[2])
        normalwritter.addData3f(facenormals[i,0], facenormals[i,1], facenormals[i,2])
        # print vert1[0], vert1[1], vert1[2]
        # print facenormals[i,0], facenormals[i,1], facenormals[i,2]
        # colorwritter.addData4f(1,1,1,1)
        vertwritter.addData3f(vert2[0], vert2[1], vert2[2])
        normalwritter.addData3f(facenormals[i,0], facenormals[i,1], facenormals[i,2])
        # print vert2[0], vert2[1], vert2[2]
        # print facenormals[i,0], facenormals[i,1], facenormals[i,2]
        # colorwritter.addData4f(1,1,1,1)
        primitive.addVertices(i*3, i*3+1, i*3+2)
    geom = Geom(vertexdata)
    geom.addPrimitive(primitive)

    return geom


def _genArrow(length, thickness = 0.5):
    """
    Generate a arrow node for plot
    This function should not be called explicitly

    ## input
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
    arrowbody = loader.loadModel(cylinderpath)
    arrowhead = loader.loadModel(conepath)
    arrowbody.setPos(0,0,0)
    arrowbody.setScale(thickness, length, thickness)
    arrowbody.reparentTo(arrow)
    arrowhead.setPos(arrow.getX(), length, arrow.getZ())
    # set scale (consider relativitly)
    arrowhead.setScale(thickness*2, thickness*9, thickness*2)
    arrowhead.reparentTo(arrow)

    return arrow


def plotArrow(nodepath, spos = None, epos = None, length = None, thickness = 0.5, rgba=None):
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

    if spos is None:
        spos = np.array([0,0,0])
    if epos is None:
        epos = np.array([0,0,1])
    if length is None:
        length = np.linalg.norm(epos-spos)
    if rgba is None:
        rgba = np.array([1,1,1,1])

    arrow = _genArrow(length, thickness)
    arrow.setPos(spos[0], spos[1], spos[2])
    arrow.lookAt(epos[0], epos[1], epos[2])
    # lookAt points y+ to epos, use the following command to point x+ to epos
    # http://stackoverflow.com/questions/15126492/panda3d-how-to-rotate-object-so-that-its-x-axis-points-to-a-location-in-space
    # arrow.setHpr(arrow, Vec3(0,0,90))
    arrow.setColor(rgba[0], rgba[1], rgba[2], rgba[3])

    arrow.reparentTo(nodepath)

def _genDumbbell(length, thickness = 0.5):
    """
    Generate a dumbbell node for plot
    This function should not be called explicitly

    ## input
    length:
        length of the dumbbell
    thickness:
        thickness of the dumbbell, set to 0.005 as default

    ## output

    """

    this_dir, this_filename = os.path.split(__file__)
    cylinderpath = os.path.join(this_dir, "geomprim", "cylinder.egg")
    conepath = os.path.join(this_dir, "geomprim", "sphere.egg")

    dumbbell = NodePath("dumbbell")
    dumbbellbody = loader.loadModel(cylinderpath)
    dumbbellhead = loader.loadModel(conepath)
    dumbbellbody.setPos(0,0,0)
    dumbbellbody.setScale(thickness, length, thickness)
    dumbbellbody.reparentTo(dumbbell)
    dumbbellhead0 = NodePath("dumbbellhead0")
    dumbbellhead1 = NodePath("dumbbellhead1")
    dumbbellhead0.setPos(dumbbellbody.getX(), length, dumbbellbody.getZ())
    dumbbellhead1.setPos(dumbbellbody.getX(), dumbbellbody.getY(), dumbbellbody.getZ())
    dumbbellhead.instanceTo(dumbbellhead0)
    dumbbellhead.instanceTo(dumbbellhead1)
    # set scale (consider relativitly)
    dumbbellhead0.setScale(thickness*2, thickness*2, thickness*2)
    dumbbellhead1.setScale(thickness*2, thickness*2, thickness*2)
    dumbbellhead0.reparentTo(dumbbell)
    dumbbellhead1.reparentTo(dumbbell)

    return dumbbell

def plotDumbbell(nodepath, spos = None, epos = None, length = None, thickness = 0.5, rgba=None):
    """
    plot a dumbbell to nodepath

    ## input:
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
        1-by-4 nparray or list

    author: weiwei
    date: 20160616
    """

    if spos is None:
        spos = np.array([0,0,0])
    if epos is None:
        epos = np.array([0,0,1])
    if length is None:
        length = np.linalg.norm(epos-spos)
    if rgba is None:
        rgba = np.array([1,1,1,1])

    dumbbell = _genDumbbell(length, thickness)
    dumbbell.setPos(spos[0], spos[1], spos[2])
    dumbbell.lookAt(epos[0], epos[1], epos[2])
    # lookAt points y+ to epos, use the following command to point x+ to epos
    # http://stackoverflow.com/questions/15126492/panda3d-how-to-rotate-object-so-that-its-x-axis-points-to-a-location-in-space
    # arrow.setHpr(arrow, Vec3(0,0,90))
    dumbbell.setColor(rgba[0], rgba[1], rgba[2], rgba[3])

    dumbbell.reparentTo(nodepath)

def plotFrame(nodepath, spos = None, epos = None, length = None, thickness = 0.01, rgba=None):
    """
    plot an arrow to nodepath

    ## input:
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

    if spos is None:
        spos = np.array([0,0,0])
    if epos is None:
        epos = np.array([0,0,1])
    if length is None:
        length = np.linalg.norm(epos-spos)
    if rgba is None:
        rgba = np.array([1,1,1,1])

    arrow = _genArrow(length, thickness)
    arrow.setPos(spos[0], spos[1], spos[2])
    arrow.lookAt(epos[0], epos[1], epos[2])
    # lookAt points y+ to epos, use the following command to point x+ to epos
    # http://stackoverflow.com/questions/15126492/panda3d-how-to-rotate-object-so-that-its-x-axis-points-to-a-location-in-space
    # arrow.setHpr(arrow, Vec3(0,0,90))
    arrow.setColor(rgba[0], rgba[1], rgba[2], rgba[3])

    arrow.reparentTo(nodepath)

def _genSphere(pandabase, radius = None):
    """
    Generate a sphere for plot
    This function should not be called explicitly

    ## input
    pandabase:
        the panda direct.showbase.ShowBase object
    radius:
        the radius of the sphere

    ## output
    sphere: pathnode

    author: weiwei
    date: 20160620 ann arbor
    """

    if radius is None:
        radius = 0.05

    this_dir, this_filename = os.path.split(__file__)
    spherepath = os.path.join(this_dir, "geomprim", "sphere.egg")

    spherepnd = NodePath("arrow")
    spherend = pandabase.loader.loadModel(spherepath)
    spherend.setPos(0,0,0)
    spherend.setScale(radius, radius, radius)
    spherend.reparentTo(spherepnd)

    return spherepnd

def plotSphere(pandabase, nodepath = None, pos = None, radius=None, rgba=None):
    """
    plot a sphere to nodepath

    ## input:
    pandabase:
        the panda direct.showbase.ShowBase object
        will be sent to _genSphere
    nodepath:
        defines which parent should the arrow be attached to
    pos:
        1-by-3 nparray or list, position of the sphere
    radius:
        will be sent to _genSphere
    rgba:
        1-by-3 nparray or list

    author: weiwei
    date: 20160620 ann arbor
    """

    if nodepath is None:
        nodepath = pandabase.render
    if pos is None:
        pos = np.array([0,0,0])
    if rgba is None:
        rgba = np.array([1,1,1,1])

    spherend = _genSphere(pandabase, radius)
    spherend.setPos(pos[0], pos[1], pos[2])
    spherend.setColor(rgba[0], rgba[1], rgba[2], rgba[3])

    spherend.reparentTo(nodepath)

def cvtMat3(npmat3):
    """
    convert numpy.2darray to LMatrix3f defined in Panda3d

    :param npmat3: a 3x3 numpy ndarray
    :return: a LMatrix3f object, see panda3d

    author: weiwei
    date: 20161107, tsukuba
    """
    return Mat3(npmat3[0, 0], npmat3[0, 1], npmat3[0, 2], npmat3[1, 0], npmat3[1, 1], npmat3[1, 2], npmat3[2, 0], npmat3[2, 1], npmat3[2, 2])

def cvtMat4(npmat3, npvec3=np.array([0,0,0])):
    """
    convert numpy.2darray to LMatrix3f defined in Panda3d

    :param npmat3: a 3x3 numpy ndarray
    :param npvec3: a 1x3 numpy ndarray
    :return: a LMatrix3f object, see panda3d

    author: weiwei
    date: 20161107, tsukuba
    """
    return Mat4(npmat3[0, 0], npmat3[1, 0], npmat3[2, 0], 0, npmat3[0, 1], npmat3[1, 1], npmat3[2, 1], 0, npmat3[0, 2], npmat3[1, 2], npmat3[2, 2], 0, npvec3[0], npvec3[1], npvec3[2], 1)

def plotAxis(nodepath, npmat4=Mat4.identMat()):
    """
    plot an axis to the scene

    :param npmat4: a panda3d LMatrix4f matrix
    :return: null

    author: weiwei
    date: 20161109, tsukuba
    """

    dbgaxisnp = NodePath("debugaxis")
    dbgaxis = loader.loadModel('zup-axis.egg')
    dbgaxis.instanceTo(dbgaxisnp)
    dbgaxis.setScale(5)
    dbgaxisnp.setMat(npmat4)
    dbgaxisnp.reparentTo(nodepath)

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

    from robotsim.nextage import nxtplot as pandactrl

    # pandactrl.setRenderEffect(base)
    pandactrl.setLight(base)

    base.run()