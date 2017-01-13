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

        if sql[:3] == 'INS':
            self.dbconnection.commit()
            return self.cursor.lastrowid
        else:
            if sql[:3] == 'SEL':
                return list(self.cursor.fetchall())

    def loadFreeAirGrip(self, objname):
        """
        load self.freegripid, etc. from mysqldatabase


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
                freeairgrip.jawwidth FROM freeairgrip, object \
                WHERE freeairgrip.idobject = object.idobject AND object.objname like '%s'" % objname
        data = self.execute(sql)
        if len(data) != 0:
            for i in range(len(data)):
                freegripid.append(data[i][0])
                freegripcontacts.append([dc.strToV3(data[i][1]), dc.strToV3(data[i][2])])
                freegripnormals.append([dc.strToV3(data[i][3]), dc.strToV3(data[i][4])])
                freegriprotmats.append(dc.strToMat4(data[i][5]))
                freegripjawwidth.append(float(data[i][6]))

            return [freegripid, freegripcontacts, freegripnormals, freegriprotmats, freegripjawwidth]
        else:
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

    def selectRobot(self, robot):
        # select idrobot
        sql = "SELECT idrobot FROM robot WHERE robot.name='%s'" % robot.name
        result = self.execute(sql)
        if len(result) != 0:
            idrobot = result[0][0]
        else:
            sql = "INSERT INTO robot(name) VALUES ('%s')" % robot.name
            idrobot = gdb.execute(sql)
        return idrobot

    def __del__(self):
        self.dbconnection.close()