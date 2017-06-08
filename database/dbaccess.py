#!/usr

import MySQLdb as mdb
import numpy as np
from utils import dbcvt as dc
from panda3d.core import *

class GraspDB(object):

    def __init__(self):
        self.dbconnection = mdb.connect("localhost", "weiweilab", "weiweilab", "freegrip")
        self.cursor = self.dbconnection.cursor()

    def execute(self, sql):
        """
        execute sql

        :param sql:
        :return: list if select, lastid if insert
        """

        try:
            self.cursor.execute(sql)
        except mdb.Error as e:
            print "MySQL Error [%d]: %s" % (e.args[0], e.args[1])
            self.dbconnection.rollback()
            raise mdb.Error

        if sql[:3] == 'SEL':
            return list(self.cursor.fetchall())
        elif sql[:3] == 'INS':
            self.dbconnection.commit()
            return self.cursor.lastrowid
        else:
            self.dbconnection.commit()

    def unsafeexecute(self, sql):
        """
        execute sql

        :param sql:
        :return: list if select, lastid if insert
        """

        try:
            self.cursor.execute("SET FOREIGN_KEY_CHECKS = 0")
            self.cursor.execute("SET SQL_SAFE_UPDATES = 0")
            self.cursor.execute(sql)
            self.cursor.execute("SET SQL_SAFE_UPDATES = 1")
            self.cursor.execute("SET FOREIGN_KEY_CHECKS = 1")
        except mdb.Error as e:
            print "MySQL Error [%d]: %s" % (e.args[0], e.args[1])
            self.dbconnection.rollback()
            self.cursor.execute("SET SQL_SAFE_UPDATES = 1")
            self.cursor.execute("SET FOREIGN_KEY_CHECKS = 1")
            raise mdb.Error

        self.dbconnection.commit()

    def loadFreeAirGrip(self, objname, handname = "rtq85"):
        """
        load self.freegripid, etc. from mysqldatabase

        :param handname which hand to use, rtq85 by default
        :return: a list of [freegripid, freegripcontacts, freegripnormals, freegriprotmats, freegripjawwidth]

        author: weiwei
        date: 20170112
        """

        freegripid = []
        freegripcontacts = []
        freegripnormals = []
        freegriprotmats = []
        freegripjawwidth = []
        # access to db

        sql = "SELECT freeairgrip.idfreeairgrip, freeairgrip.contactpnt0, freeairgrip.contactpnt1, \
                freeairgrip.contactnormal0, freeairgrip.contactnormal1, freeairgrip.rotmat, \
                freeairgrip.jawwidth FROM freeairgrip, hand, object \
                WHERE freeairgrip.idobject = object.idobject AND object.name like '%s' \
                AND freeairgrip.idhand = hand.idhand AND hand.name like '%s'" % (objname, handname)
        data = self.execute(sql)
        if len(data) != 0:
            for i in range(len(data)):
                freegripid.append(int(data[i][0]))
                freegripcontacts.append([dc.strToV3(data[i][1]), dc.strToV3(data[i][2])])
                freegripnormals.append([dc.strToV3(data[i][3]), dc.strToV3(data[i][4])])
                freegriprotmats.append(dc.strToMat4(data[i][5]))
                freegripjawwidth.append(float(data[i][6]))

            return [freegripid, freegripcontacts, freegripnormals, freegriprotmats, freegripjawwidth]
        else:
            return None

    def loadFreeTabletopPlacement(self, objname):
        """
        load self.freegripid, etc. from mysqldatabase

        :param handname which hand to use, rtq85 by default
        :return: a list of [freegripid, freegripcontacts, freegripnormals, freegriprotmats, freegripjawwidth]

        author: weiwei
        date: 20170112
        """

        tpsmat4s = []
        # access to db

        sql = "SELECT freetabletopplacement.rotmat FROM freetabletopplacement,object WHERE \
                freetabletopplacement.idobject = object.idobject \
                AND object.name LIKE '%s'" % objname
        data = self.execute(sql)
        if len(data) != 0:
            for i in range(len(data)):
                tpsmat4s.append(dc.strToMat4(data[i][0]))

            return tpsmat4s
        else:
            print "Plan tabletoplacements using freetabletopplacement.removebadfacets first!"
            return None

    def loadIKRet(self):
        sql = "SELECT rethandx, retworldz, retworlda FROM ikret"
        result = self.execute(sql)
        if len(result) != 0:
            rethandx = result[0][0]
            retworldz = result[0][1]
            retworlda = result[0][2]
        else:
            # default value: 50,50,50
            sql = "INSERT INTO ikret VALUES (50,50,50)"
            self.execute(sql)
            rethandx = 50
            retworldz = 50
            retworlda = 50
        return [rethandx, retworldz, retworlda, Vec3(0,0,1)]

    def loadIdHand(self, handname):
        sql = "SELECT idhand FROM hand WHERE name = '%s'" % handname
        result = self.execute(sql)
        print result
        if len(result) != 0:
            idhand = int(result[0][0])
        else:
            assert "No hand found in hand table!"
        return idhand

    def loadIdArm(self, armname):
        sql = "SELECT idarm FROM arm WHERE name = '%s'" % armname
        result = self.execute(sql)
        if len(result) != 0:
            idarm = int(result[0][0])
        else:
            assert "No arm found in arm table!"
        return idarm

    def loadIdObject(self, objname):
        sql = "SELECT idobject FROM object WHERE name = '%s'" % objname
        result = self.execute(sql)
        if len(result) != 0:
            idobj = int(result[0][0])
        else:
            assert "No object named " + objname + " found in object table!"
        return idobj

    def loadIdRobot(self, robot):
        # select idrobot
        sql = "SELECT idrobot FROM robot WHERE robot.name='%s'" % robot.name
        result = self.execute(sql)
        if len(result) != 0:
            idrobot = result[0][0]
        else:
            sql = "INSERT INTO robot(name) VALUES ('%s')" % robot.name
            idrobot = self.execute(sql)
        return idrobot

    def loadIdAssembly(self, dbobj0name, rotmat0, dbobj1name, rotmat1, assdirect1to0):
        # select assembly id
        # return [] if not found
        idobj0 = self.loadIdObject(dbobj0name)
        idobj1 = self.loadIdObject(dbobj1name)
        strrotmat0 = dc.mat4ToStr(rotmat0)
        strrotmat1 = dc.mat4ToStr(rotmat1)
        strassdirect1to0 = dc.v3ToStr(assdirect1to0)
        sql = "SELECT idassembly FROM assembly WHERE assembly.idobject0 = %d AND assembly.idobject1 = %d AND \
                assembly.rotmat0 LIKE '%s' AND assembly.rotmat1 LIKE '%s' AND assembly.assdirect1to0 LIKE '%s'" % \
              (idobj0, idobj1, strrotmat0, strrotmat1, strassdirect1to0)
        result = self.execute(sql)
        if len(result) != 0:
            idassembly = result[0][0]
        else:
            sql = "INSERT INTO assembly(idobject0, rotmat0, idobject1, rotmat1, assdirect1to0) VALUES (%d, '%s', %d, '%s', '%s')" % \
                  (idobj0, strrotmat0, idobj1, strrotmat1, strassdirect1to0)
            idassembly = self.execute(sql)
        return idassembly

    def checkExistenceAssembly(self, objname):
        # select idrobot
        sql = "SELECT idrobot FROM robot WHERE robot.name='%s'" % robot.name
        result = self.execute(sql)
        if len(result) != 0:
            idrobot = result[0][0]
        else:
            sql = "INSERT INTO robot(name) VALUES ('%s')" % robot.name
            idrobot = self.execute(sql)
        return idrobot

    def deleteRegraspPlan(self, robot, objname, handname):
        """
        Delete the Regrasp Plan
        Assembly is not applicable!

        :param robot:
        :param objname:
        :param handname:
        :return:
        """
        idrobot = self.loadIdRobot(robot)
        idobj = self.loadIdObject(objname)
        idhand = self.loadIdHand(handname)
        print idobj, idhand
        # delete freeairgrip
        sql = "DELETE FROM freeairgrip \
                WHERE freeairgrip.idobject = %d AND \
                freeairgrip.idhand = %d" % (idobj, idhand)
        self.unsafeexecute(sql)
        # delete freetabletopgrip and tabletopgrips, NOTE: the placements are not deleted
        # delete freetabletopgrip
        sql = "DELETE FROM freetabletopgrip WHERE freetabletopgrip.idfreeairgrip NOT IN \
                (SELECT freeairgrip.idfreeairgrip FROM freeairgrip)"
        self.unsafeexecute(sql)
        # delete tabletopgrip
        sql = "DELETE FROM tabletopgrips WHERE tabletopgrips.idfreeairgrip NOT IN \
                (SELECT freeairgrip.idfreeairgrip FROM freeairgrip)"
        self.unsafeexecute(sql)
        # delete ik
        sql = "DELETE FROM ik WHERE ik.idtabletopgrips NOT IN \
                (SELECT tabletopgrips.idtabletopgrips FROM tabletopgrips)"
        self.unsafeexecute(sql)

    def __del__(self):
        self.dbconnection.close()

if __name__=='__main__':

    # obj path
    import os
    this_dir, this_filename = os.path.split(__file__)
    # objpath = os.path.join(os.path.split(this_dir)[0], "grip", "objects", "ttube.stl")
    objpath = os.path.join(os.path.split(this_dir)[0], "manipulation", "objects", "tool.stl")
    objname = os.path.splitext(os.path.basename(objpath))[0]

    # delete Regrasp plan
    from manipulation.grip.hrp5three import hrp5threenm
    from robotsim.hrp5n import hrp5n
    from robotsim.hrp5n import hrp5nplot
    handpkg = hrp5threenm
    handname = handpkg.getHandName()
    hrp5nrobot = hrp5n.Hrp5NRobot()
    gdb = GraspDB()
    gdb.deleteRegraspPlan(hrp5nrobot, objname, handname)