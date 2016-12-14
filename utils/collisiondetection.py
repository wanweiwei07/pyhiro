
from panda3d.core import *
from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletRigidBodyNode
from panda3d.bullet import BulletTriangleMesh
from panda3d.bullet import BulletTriangleMeshShape

from pandaplotutils import pandageom

def rayHit(pfrom, pto, geom):
    """
    NOTE: this function is quite slow
    find the nearest collision point between vec(pto-pfrom) and the mesh of nodepath

    :param pfrom: starting point of the ray, Point3
    :param pto: ending point of the ray, Point3
    :param geom: meshmodel, a panda3d datatype
    :return: None or Point3

    author: weiwei
    date: 20161201
    """

    bulletworld = BulletWorld()
    facetmesh = BulletTriangleMesh()
    facetmesh.addGeom(geom)
    facetmeshnode = BulletRigidBodyNode('facet')
    facetmeshnode.addShape(BulletTriangleMeshShape(facetmesh, dynamic=True))
    bulletworld.attachRigidBody(facetmeshnode)
    result = bulletworld.rayTestClosest(pfrom, pto)

    if result.hasHit():
        return result.getHitPos()
    else:
        return None

def genCollisionMeshNp(nodepath, basenodepath=None, name='autogen'):
    """
    generate the collision mesh of a nodepath using nodepath
    this function suppose the nodepath is a single model with one geomnode

    :param nodepath: the panda3d nodepath of the object
    :param basenodepath: the nodepath to compute relative transform, identity if none
    :param name: the name of the rigidbody
    :return: bulletrigidbody

    author: weiwei
    date: 20161212, tsukuba
    """

    geomnodepath = nodepath.find("**/+GeomNode")
    geombullnode = BulletRigidBodyNode(name)
    geom = geomnodepath.node().getGeom(0)
    geomtf = nodepath.getTransform(base.render)
    if basenodepath:
        geomtf = nodepath.getTransform(basenodepath)
    geombullmesh = BulletTriangleMesh()
    geombullmesh.addGeom(geom)
    geombullnode.addShape(BulletTriangleMeshShape(geombullmesh, dynamic=True), geomtf)
    return geombullnode

def genCollisionMeshMultiNp(nodepath, basenodepath=None, name='autogen'):
    """
    generate the collision mesh of a nodepath using nodepath
    this function suppose the nodepath has multiple models with many geomnodes

    :param nodepath: the panda3d nodepath of the object
    :param basenodepath: the nodepath to compute relative transform, identity if none
    :param name: the name of the rigidbody
    :return: bulletrigidbody

    author: weiwei
    date: 20161212, tsukuba
    """

    gndcollection = nodepath.findAllMatches("**/+GeomNode")
    geombullnode = BulletRigidBodyNode(name)
    for gnd in gndcollection:
        geom = gnd.node().getGeom(0)
        geomtf = gnd.getTransform(base.render)
        if basenodepath:
            geomtf = gnd.getTransform(basenodepath)
        geombullmesh = BulletTriangleMesh()
        geombullmesh.addGeom(geom)
        geombullnode.addShape(BulletTriangleMeshShape(geombullmesh, dynamic=True), geomtf)
    return geombullnode

def genCollisionMeshGeom(geom, name='autogen'):
    """
    generate the collision mesh of a nodepath using geom

    :param geom: the panda3d geom of the object
    :param basenodepath: the nodepath to compute relative transform
    :return: bulletrigidbody

    author: weiwei
    date: 20161212, tsukuba
    """

    geomtf = TransformState.makeIdentity()
    geombullmesh = BulletTriangleMesh()
    geombullmesh.addGeom(geom)
    geombullnode = BulletRigidBodyNode(name)
    geombullnode.addShape(BulletTriangleMeshShape(geombullmesh, dynamic=True), geomtf)
    return geombullnode
