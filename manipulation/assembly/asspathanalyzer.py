#!/usr/bin/python

import os
import trimesh

class AssPathAnalyzer(object):
    """
    assembly.asspathanalyzer analyzes the available grasps for an assembly path
    an assembly path is defined as a list where each element in the list is a tuple
    (objname, relMat4). relMat4 is the relative rotmat in the local coordinate system of the preceding object.
    an example is: [(obj1, identity), (obj2, obj2to1RelMat4), (obj3, obj3to2RelMat4), ...]
    """

    def __init__(self, asspath, gdb):
        """

        :param asspath: an assembly path, see the definition in the leading text
        :param gdb:

        author: weiwei
        date: 20170227
        """

        self.basetrimeshList = []
        self.objtrimesh = trimesh.load_mesh(objpath)

        pass

    def genFreeGrasps(self, relMat4):
        """
        generate the grasps for the objects

        asspathassgrips are the grips to assemble objectx to objectx-1
        asspathhldgrips are the grips to hold objectx-1 when another arm assembles objectx to it

        :param relMat4:
        :return: [asspathassgrips, asspathhldgrips]

        author: weiwei
        date: 20170227
        """

        pass

    def xformGrasps(self, baseMat4):
        """
        generate the grasps for the two objects
        base rotmat --> baseMat4
        object rotmat --> baseMat4 * relMat4

        :param baseMat4:
        :return: [asspathassgrips, asspathhldgrips], they are xformed to baseMat4

        author: weiwei
        date: 20170227
        """

        pass


if __name__ == '__main__':
    pass