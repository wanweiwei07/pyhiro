#!/usr/bin/python

import os
import trimesh
import manipulation.grip.freegrip.FreeAirGrip as FreeAirGrip

class AssPathAnalyzer(object):
    """
    assembly.asspathanalyzer analyzes the available grasps for an assembly path
    an assembly path is defined as a list where each element in the list is a tuple
    (objname, relMat4). relMat4 is the relative rotmat in the local coordinate system of the preceding object.
    an example is: [(obj1, identity), (obj2, obj2to1RelMat4), (obj3, obj3to2RelMat4), ...]
    """

    def __init__(self, gdb, asspath, handpkg):
        """

        :param asspath: an assembly path, see the definition in the leading text
        :param gdb:

        author: weiwei
        date: 20170227
        """

        self.handpkg = handpkg
        self.objTrimeshList = []
        self.objFAGList = [] # Fag = free air grip
        self.objRotmat4List = []
        for objpath, rotmat4 in asspath:
            self.objTrimeshList.append(trimesh.load_mesh(objpath))
            curRotmat4 = rotmat4
            for prevRotmat4 in self.objRotmat4List[::-1]:
                curRotmat4 = prevRotmat4*curRotmat4
            self.objRotMat4List.append(curRotmat4)
            dbobjname = os.path.splitext(os.path.basename(objpath))[0]
            self.objFAGList.append(FreeAirGrip(gdb, dbobjname, self.handpkg.getHandName()))
        pass

    def __setupAssemDB(self):
        """
        save assembly sequence to DB

        :return:
        author: weiwei
        date: 20170306
        """



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