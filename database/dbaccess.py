#!/usr

import MySQLdb as mdb


class GraspDB(object):

    def __init__(self):
        self.dbconnection = mdb.connect("localhost", "weiweilab", "weiweilab", "freegrip")
        self.cursor = self.dbconnection.cursor()

    def execute(self, sql):
        try:
            self.cursor.execute(sql)
        except mdb.Error as e:
            print "MySQL Error [%d]: %s" % (e.args[0], e.args[1])
            self.db.rollback()
            raise mdb.Error

        return list(cursor.fetchall())

    def __del__(self):
        self.dbconnection.close()