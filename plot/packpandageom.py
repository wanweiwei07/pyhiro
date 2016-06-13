from panda3d.core import *
# package the vertices and triangles into a panda3d geom
#
# input:
#  vertices: a n-by-3 nparray, each row is a vertex
#  triangles: a n-by-3 nparray, each row is three idx to the vertices
#  name: not as important
#
# output:
#  geom: a Geom model which is ready to be added to a node
#
# author: weiwei
# date: 20160613

def packpandageom(vertices, triangles, name=''):

    format = GeomVertexFormat.getV3()
    vertexData = GeomVertexData(name, format, Geom.UHStatic)
    vertexData.setNumRows(vertices.shape[0])
    vertwritter = GeomVertexWriter(vertexData, 'vertex')
    for vert in vertices:
        vertwritter.addData3f(vert[0], vert[1], vert[2])
    primitive = GeomTriangles(Geom.UHStatic)
    for fvidx in triangles:
        primitive.addVertices(fvidx[0], fvidx[1], fvidx[2])
    geom = Geom(vertexData)
    geom.addPrimitive(primitive)

    return geom