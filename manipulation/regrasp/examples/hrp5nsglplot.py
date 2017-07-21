# plot the shortest path using hrp5robot

from robotsim.nextage import nxt
from robotsim.nextage import nxtplot
from robotsim.hrp5n import hrp5n
from robotsim.hrp5n import hrp5nplot
from manipulation.grip.robotiq85 import rtq85nm
from manipulation.grip.hrp5three import hrp5threenm

from manipulation.regrasp import regriptpp
from database import dbaccess as db
import pandaplotutils.pandactrl as pandactrl
from panda3d.core import *
from pandaplotutils import pandageom as pg
import matplotlib.pyplot as plt
import os
import numpy as np

def getMotionSequence(regrip, id = 0):
    """
    generate motion sequence using the shortest path
    right arm

    :param: regrip an object of the regriptpp.RegripTpp class
    :param: gdb GraspDB object

    :return: [[waist, shoulder, sixjoints],...]

    author: weiwei
    date: 20170113
    """

    if len(regrip.directshortestpaths) == 0:
        print "no path found"
        return

    pathnidlist = regrip.directshortestpaths[id]
    numikrlist = []
    objmat4list = []
    jawwidth = []
    for i in range(len(pathnidlist) - 1):
        if i == 0 and len(pathnidlist) == 2:
            # two node path
            ## starting node
            nid = pathnidlist[i]
            grpposhandx = regrip.regg.node[nid]['fgrcenterhandx']
            grppos = regrip.regg.node[nid]['fgrcenter']
            grpposworlda = regrip.regg.node[nid]['fgrcenterworlda']
            grpposworldaworldz = regrip.regg.node[nid]['fgrcenterworldaworldz']
            grprot = regrip.regg.node[nid]['hndrotmat3np']
            grpjawwidth = regrip.regg.node[nid]['jawwidth']
            armjntsgrphandx = regrip.robot.numikr(grpposhandx, grprot)
            armjntsgrp = regrip.robot.numikr(grppos, grprot)
            armjntsgrpworlda = regrip.robot.numikr(grpposworlda, grprot)
            armjntsgrpworldaworldz = regrip.robot.numikr(grpposworldaworldz, grprot)
            numikrlist.append(armjntsgrphandx)
            jawwidth.append(regrip.robothand.jawwidthopen)
            numikrlist.append(armjntsgrp)
            jawwidth.append(regrip.robothand.jawwidthopen)
            numikrlist.append(armjntsgrp)
            jawwidth.append(grpjawwidth)
            numikrlist.append(armjntsgrpworlda)
            jawwidth.append(grpjawwidth)
            numikrlist.append(armjntsgrpworldaworldz)
            jawwidth.append(grpjawwidth)
            objmat4handx = regrip.regg.node[nid]['tabletopplacementrotmathandx']
            objmat4 = regrip.regg.node[nid]['tabletopplacementrotmat']
            objmat4worlda = regrip.regg.node[nid]['tabletopplacementrotmatworlda']
            objmat4worldaworldz = regrip.regg.node[nid]['tabletopplacementrotmatworldaworldz']
            objmat4list.append(objmat4handx)
            objmat4list.append(objmat4)
            objmat4list.append(objmat4)
            objmat4list.append(objmat4worlda)
            objmat4list.append(objmat4worldaworldz)
            ## first node
            nid = pathnidlist[i + 1]
            grpposworldaworldz = regrip.regg.node[nid]['fgrcenterworldaworldz']
            grpposworlda = regrip.regg.node[nid]['fgrcenterworlda']
            grppos = regrip.regg.node[nid]['fgrcenter']
            grpposhandx = regrip.regg.node[nid]['fgrcenterhandx']
            grpposhandxworldz = regrip.regg.node[nid]['fgrcenterhandxworldz']
            grprot = regrip.regg.node[nid]['hndrotmat3np']
            grpjawwidth = regrip.regg.node[nid]['jawwidth']
            armjntsgrpworldaworldz = regrip.robot.numikr(grpposworldaworldz, grprot)
            armjntsgrpworlda = regrip.robot.numikr(grpposworlda, grprot)
            armjntsgrp = regrip.robot.numikr(grppos, grprot)
            armjntsgrphandx = regrip.robot.numikr(grpposhandx, grprot)
            armjntsgrphandxworldz = regrip.robot.numikr(grpposhandxworldz, grprot)
            numikrlist.append(armjntsgrpworldaworldz)
            jawwidth.append(grpjawwidth)
            numikrlist.append(armjntsgrpworlda)
            jawwidth.append(grpjawwidth)
            numikrlist.append(armjntsgrp)
            jawwidth.append(grpjawwidth)
            numikrlist.append(armjntsgrp)
            jawwidth.append(regrip.robothand.jawwidthopen)
            numikrlist.append(armjntsgrphandx)
            jawwidth.append(regrip.robothand.jawwidthopen)
            numikrlist.append(armjntsgrphandxworldz)
            jawwidth.append(regrip.robothand.jawwidthopen)
            objmat4worldaworldz = regrip.regg.node[nid]['tabletopplacementrotmatworldaworldz']
            objmat4worlda = regrip.regg.node[nid]['tabletopplacementrotmatworlda']
            objmat4 = regrip.regg.node[nid]['tabletopplacementrotmat']
            objmat4handx = regrip.regg.node[nid]['tabletopplacementrotmathandx']
            objmat4handxworldz = regrip.regg.node[nid]['tabletopplacementrotmathandxworldz']
            objmat4list.append(objmat4worldaworldz)
            objmat4list.append(objmat4worlda)
            objmat4list.append(objmat4)
            objmat4list.append(objmat4)
            objmat4list.append(objmat4handx)
            objmat4list.append(objmat4handxworldz)
        else:
            if i == 0:
                # not two nodepath, starting node, transfer
                ## starting node
                nid = pathnidlist[i]
                grpposhandx = regrip.regg.node[nid]['fgrcenterhandx']
                grppos = regrip.regg.node[nid]['fgrcenter']
                grpposworlda = regrip.regg.node[nid]['fgrcenterworlda']
                grpposworldaworldz = regrip.regg.node[nid]['fgrcenterworldaworldz']
                grprot = regrip.regg.node[nid]['hndrotmat3np']
                grpjawwidth = regrip.regg.node[nid]['jawwidth']
                armjntsgrphandx = regrip.robot.numikr(grpposhandx, grprot)
                armjntsgrp = regrip.robot.numikr(grppos, grprot)
                armjntsgrpworlda = regrip.robot.numikr(grpposworlda, grprot)
                armjntsgrpworldaworldz = regrip.robot.numikr(grpposworldaworldz, grprot)
                numikrlist.append(armjntsgrphandx)
                jawwidth.append(regrip.robothand.jawwidthopen)
                numikrlist.append(armjntsgrp)
                jawwidth.append(regrip.robothand.jawwidthopen)
                numikrlist.append(armjntsgrp)
                jawwidth.append(grpjawwidth)
                numikrlist.append(armjntsgrpworlda)
                jawwidth.append(grpjawwidth)
                numikrlist.append(armjntsgrpworldaworldz)
                jawwidth.append(grpjawwidth)
                objmat4handx = regrip.regg.node[nid]['tabletopplacementrotmathandx']
                objmat4 = regrip.regg.node[nid]['tabletopplacementrotmat']
                objmat4worlda = regrip.regg.node[nid]['tabletopplacementrotmatworlda']
                objmat4worldaworldz = regrip.regg.node[nid]['tabletopplacementrotmatworldaworldz']
                objmat4list.append(objmat4handx)
                objmat4list.append(objmat4)
                objmat4list.append(objmat4)
                objmat4list.append(objmat4worlda)
                objmat4list.append(objmat4worldaworldz)
                ## first node
                nid = pathnidlist[i + 1]
                grpposworldaworldz = regrip.regg.node[nid]['fgrcenterworldaworldz']
                grpposworlda = regrip.regg.node[nid]['fgrcenterworlda']
                grppos = regrip.regg.node[nid]['fgrcenter']
                grpposhandx = regrip.regg.node[nid]['fgrcenterhandx']
                grpposhandxworldz = regrip.regg.node[nid]['fgrcenterhandxworldz']
                grprot = regrip.regg.node[nid]['hndrotmat3np']
                grpjawwidth = regrip.regg.node[nid]['jawwidth']
                armjntsgrpworldaworldz = regrip.robot.numik(grpposworldaworldz, grprot)
                armjntsgrpworlda = regrip.robot.numik(grpposworlda, grprot)
                armjntsgrp = regrip.robot.numik(grppos, grprot)
                armjntsgrphandx = regrip.robot.numik(grpposhandx, grprot)
                armjntsgrphandxworldz = regrip.robot.numik(grpposhandxworldz, grprot)
                numikrlist.append([0, armjntsgrpworldaworldz])
                jawwidth.append(grpjawwidth)
                numikrlist.append([0, armjntsgrpworlda])
                jawwidth.append(grpjawwidth)
                numikrlist.append([0, armjntsgrp])
                jawwidth.append(grpjawwidth)
                numikrlist.append([0, armjntsgrp])
                jawwidth.append(regrip.robothand.jawwidthopen)
                numikrlist.append([0, armjntsgrphandx])
                jawwidth.append(regrip.robothand.jawwidthopen)
                numikrlist.append([0, armjntsgrphandxworldz])
                jawwidth.append(regrip.robothand.jawwidthopen)
                objmat4worldaworldz = regrip.regg.node[nid]['tabletopplacementrotmatworldaworldz']
                objmat4worlda = regrip.regg.node[nid]['tabletopplacementrotmatworlda']
                objmat4 = regrip.regg.node[nid]['tabletopplacementrotmat']
                objmat4handx = regrip.regg.node[nid]['tabletopplacementrotmathandx']
                objmat4handxworldz = regrip.regg.node[nid]['tabletopplacementrotmathandxworldz']
                objmat4list.append(objmat4worldaworldz)
                objmat4list.append(objmat4worlda)
                objmat4list.append(objmat4)
                objmat4list.append(objmat4)
                objmat4list.append(objmat4handx)
                objmat4list.append(objmat4handxworldz)
            else:
                if i + 1 != len(pathnidlist) - 1:
                    # not two node path, middle nodes, if transit, pass
                    if regrip.regg.edge[pathnidlist[i]][pathnidlist[i+1]]['edgetype'] == "transit":
                        pass
                    else:
                        # not two node path, middle nodes, if transfer
                        ## middle first
                        nid = pathnidlist[i]
                        grpposhandxworldz = regrip.regg.node[nid]['fgrcenterhandxworldz']
                        grpposhandx = regrip.regg.node[nid]['fgrcenterhandx']
                        grppos = regrip.regg.node[nid]['fgrcenter']
                        grpposworlda = regrip.regg.node[nid]['fgrcenterworlda']
                        grpposworldaworldz = regrip.regg.node[nid]['fgrcenterworldaworldz']
                        grprot = regrip.regg.node[nid]['hndrotmat3np']
                        grpjawwidth = regrip.regg.node[nid]['jawwidth']
                        armjntsgrphandxworldz = regrip.robot.numik(grpposhandxworldz, grprot)
                        armjntsgrphandx = regrip.robot.numik(grpposhandx, grprot)
                        armjntsgrp = regrip.robot.numik(grppos, grprot)
                        armjntsgrpworlda = regrip.robot.numik(grpposworlda, grprot)
                        armjntsgrpworldaworldz = regrip.robot.numik(grpposworldaworldz, grprot)
                        numikrlist.append([0, armjntsgrphandxworldz])
                        jawwidth.append(regrip.robothand.jawwidthopen)
                        numikrlist.append([0, armjntsgrphandx])
                        jawwidth.append(regrip.robothand.jawwidthopen)
                        numikrlist.append([0, armjntsgrp])
                        jawwidth.append(regrip.robothand.jawwidthopen)
                        numikrlist.append([0, armjntsgrp])
                        jawwidth.append(grpjawwidth)
                        numikrlist.append([0, armjntsgrpworlda])
                        jawwidth.append(grpjawwidth)
                        numikrlist.append([0, armjntsgrpworldaworldz])
                        jawwidth.append(grpjawwidth)
                        objmat4handxworldz = regrip.regg.node[nid]['tabletopplacementrotmathandxworldz']
                        objmat4handx = regrip.regg.node[nid]['tabletopplacementrotmathandx']
                        objmat4 = regrip.regg.node[nid]['tabletopplacementrotmat']
                        objmat4worlda = regrip.regg.node[nid]['tabletopplacementrotmatworlda']
                        objmat4worldaworldz = regrip.regg.node[nid]['tabletopplacementrotmatworldaworldz']
                        objmat4list.append(objmat4handxworldz)
                        objmat4list.append(objmat4handx)
                        objmat4list.append(objmat4)
                        objmat4list.append(objmat4)
                        objmat4list.append(objmat4worlda)
                        objmat4list.append(objmat4worldaworldz)
                        ## middle second
                        nid = pathnidlist[i + 1]
                        grpposworldaworldz = regrip.regg.node[nid]['fgrcenterworldaworldz']
                        grpposworlda = regrip.regg.node[nid]['fgrcenterworlda']
                        grppos = regrip.regg.node[nid]['fgrcenter']
                        grpposhandx = regrip.regg.node[nid]['fgrcenterhandx']
                        grpposhandxworldz = regrip.regg.node[nid]['fgrcenterhandxworldz']
                        grprot = regrip.regg.node[nid]['hndrotmat3np']
                        grpjawwidth = regrip.regg.node[nid]['jawwidth']
                        armjntsgrpworldaworldz = regrip.robot.numik(grpposworldaworldz, grprot)
                        armjntsgrpworlda = regrip.robot.numik(grpposworlda, grprot)
                        armjntsgrp = regrip.robot.numik(grppos, grprot)
                        armjntsgrphandx = regrip.robot.numik(grpposhandx, grprot)
                        armjntsgrphandxworldz = regrip.robot.numik(grpposhandxworldz, grprot)
                        numikrlist.append([0, armjntsgrpworldaworldz])
                        jawwidth.append(grpjawwidth)
                        numikrlist.append([0, armjntsgrpworlda])
                        jawwidth.append(grpjawwidth)
                        numikrlist.append([0, armjntsgrp])
                        jawwidth.append(grpjawwidth)
                        numikrlist.append([0, armjntsgrp])
                        jawwidth.append(regrip.robothand.jawwidthopen)
                        numikrlist.append([0, armjntsgrphandx])
                        jawwidth.append(regrip.robothand.jawwidthopen)
                        numikrlist.append([0, armjntsgrphandxworldz])
                        jawwidth.append(regrip.robothand.jawwidthopen)
                        objmat4worldaworldz = regrip.regg.node[nid]['tabletopplacementrotmatworldaworldz']
                        objmat4worlda = regrip.regg.node[nid]['tabletopplacementrotmatworlda']
                        objmat4 = regrip.regg.node[nid]['tabletopplacementrotmat']
                        objmat4handx = regrip.regg.node[nid]['tabletopplacementrotmathandx']
                        objmat4handxworldz = regrip.regg.node[nid]['tabletopplacementrotmathandxworldz']
                        objmat4list.append(objmat4worldaworldz)
                        objmat4list.append(objmat4worlda)
                        objmat4list.append(objmat4)
                        objmat4list.append(objmat4)
                        objmat4list.append(objmat4handx)
                        objmat4list.append(objmat4handxworldz)
                else:
                    # not two node path, end nodes, transfer
                    ## second to last node
                    nid = pathnidlist[i]
                    grpposhandxworldz = regrip.regg.node[nid]['fgrcenterhandxworldz']
                    grpposhandx = regrip.regg.node[nid]['fgrcenterhandx']
                    grppos = regrip.regg.node[nid]['fgrcenter']
                    grpposworlda = regrip.regg.node[nid]['fgrcenterworlda']
                    grpposworldaworldz = regrip.regg.node[nid]['fgrcenterworldaworldz']
                    grprot = regrip.regg.node[nid]['hndrotmat3np']
                    grpjawwidth = regrip.regg.node[nid]['jawwidth']
                    armjntsgrphandxworldz = regrip.robot.numik(grpposhandxworldz, grprot)
                    armjntsgrphandx = regrip.robot.numik(grpposhandx, grprot)
                    armjntsgrp = regrip.robot.numik(grppos, grprot)
                    armjntsgrpworlda = regrip.robot.numik(grpposworlda, grprot)
                    armjntsgrpworldaworldz = regrip.robot.numik(grpposworldaworldz, grprot)
                    numikrlist.append([0, armjntsgrphandxworldz])
                    jawwidth.append(regrip.robothand.jawwidthopen)
                    numikrlist.append([0, armjntsgrphandx])
                    jawwidth.append(regrip.robothand.jawwidthopen)
                    numikrlist.append([0, armjntsgrp])
                    jawwidth.append(regrip.robothand.jawwidthopen)
                    numikrlist.append([0, armjntsgrp])
                    jawwidth.append(grpjawwidth)
                    numikrlist.append([0, armjntsgrpworlda])
                    jawwidth.append(grpjawwidth)
                    numikrlist.append([0, armjntsgrpworldaworldz])
                    jawwidth.append(grpjawwidth)
                    objmat4handxworldz = regrip.regg.node[nid]['tabletopplacementrotmathandxworldz']
                    objmat4handx = regrip.regg.node[nid]['tabletopplacementrotmathandx']
                    objmat4 = regrip.regg.node[nid]['tabletopplacementrotmat']
                    objmat4worlda = regrip.regg.node[nid]['tabletopplacementrotmatworlda']
                    objmat4worldaworldz = regrip.regg.node[nid]['tabletopplacementrotmatworldaworldz']
                    objmat4list.append(objmat4handxworldz)
                    objmat4list.append(objmat4handx)
                    objmat4list.append(objmat4)
                    objmat4list.append(objmat4)
                    objmat4list.append(objmat4worlda)
                    objmat4list.append(objmat4worldaworldz)
                    ## last node
                    nid = pathnidlist[i + 1]
                    grpposworldaworldz = regrip.regg.node[nid]['fgrcenterworldaworldz']
                    grpposworlda = regrip.regg.node[nid]['fgrcenterworlda']
                    grppos = regrip.regg.node[nid]['fgrcenter']
                    grpposhandx = regrip.regg.node[nid]['fgrcenterhandx']
                    grpposhandxworldz = regrip.regg.node[nid]['fgrcenterhandxworldz']
                    grprot = regrip.regg.node[nid]['hndrotmat3np']
                    grpjawwidth = regrip.regg.node[nid]['jawwidth']
                    armjntsgrpworldaworldz = regrip.robot.numikr(grpposworldaworldz, grprot)
                    armjntsgrpworlda = regrip.robot.numikr(grpposworlda, grprot)
                    armjntsgrp = regrip.robot.numikr(grppos, grprot)
                    armjntsgrphandx = regrip.robot.numikr(grpposhandx, grprot)
                    armjntsgrphandxworldz = regrip.robot.numikr(grpposhandxworldz, grprot)
                    numikrlist.append(armjntsgrpworldaworldz)
                    jawwidth.append(grpjawwidth)
                    numikrlist.append(armjntsgrpworlda)
                    jawwidth.append(grpjawwidth)
                    numikrlist.append(armjntsgrp)
                    jawwidth.append(grpjawwidth)
                    numikrlist.append(armjntsgrp)
                    jawwidth.append(regrip.robothand.jawwidthopen)
                    numikrlist.append(armjntsgrphandx)
                    jawwidth.append(regrip.robothand.jawwidthopen)
                    numikrlist.append(armjntsgrphandxworldz)
                    jawwidth.append(regrip.robothand.jawwidthopen)
                    objmat4worldaworldz = regrip.regg.node[nid]['tabletopplacementrotmatworldaworldz']
                    objmat4worlda = regrip.regg.node[nid]['tabletopplacementrotmatworlda']
                    objmat4 = regrip.regg.node[nid]['tabletopplacementrotmat']
                    objmat4handx = regrip.regg.node[nid]['tabletopplacementrotmathandx']
                    objmat4handxworldz = regrip.regg.node[nid]['tabletopplacementrotmathandxworldz']
                    objmat4list.append(objmat4worldaworldz)
                    objmat4list.append(objmat4worlda)
                    objmat4list.append(objmat4)
                    objmat4list.append(objmat4)
                    objmat4list.append(objmat4handx)
                    objmat4list.append(objmat4handxworldz)
    return [objmat4list, numikrlist, jawwidth]

if __name__=='__main__':
    gdb = db.GraspDB()
    handpkg = hrp5threenm
    hrp5nrobot = hrp5n.Hrp5NRobot()

    base = pandactrl.World(camp=[3000,0,2000], lookatp=[300,0,300])

    # ttube.stl
    this_dir, this_filename = os.path.split(__file__)
    # objpath = os.path.join(os.path.split(this_dir)[0], "grip", "objects", "ttube.stl")
    # objpath = os.path.join(os.path.split(this_dir)[0], "grip", "objects", "tool.stl")
    objpath = os.path.join(os.path.split(this_dir)[0], "grip", "objects", "tool2.stl")
    # objpath = os.path.join(os.path.split(this_dir)[0], "grip", "objects", "planewheel.stl")
    # objpath = os.path.join(os.path.split(this_dir)[0], "grip", "objects", "planelowerbody.stl")
    # objpath = os.path.join(os.path.split(this_dir)[0], "grip", "objects", "planefrontstay.stl")
    # objpath = os.path.join(os.path.split(this_dir)[0], "grip", "objects", "planerearstay.stl")
    regrip = regriptpp.RegripTpp(objpath, hrp5nrobot, handpkg, gdb, offset = 100)

    # ttube
    startrotmat4 = Mat4(-4.06358332355e-17,-0.183007523417,0.98311150074,0.0,
                        -1.0,2.22044604925e-16,0.0,0.0,
                        -2.18294604779e-16,-0.98311150074,-0.183007523417,0.0,
                        399.926879883,-196.8688812256,79.7615509033,1.0)
    # startrotmat4 = Mat4(0.129405856133,0.129405856133,0.98311150074,0.0,
    #                     0.707106769085,-0.707106769085,0.0,0.0,
    #                     0.69516479969,0.69516479969,-0.183007523417,0.0,
    #                     327.126998901,-102.976409912,19.7615509033,1.0)
    # startrotmat4 = Mat4(-0.707106769085, 0.707106769085, 0.0, 0.0, -0.707106769085, -0.707106769085, 0.0, 0.0, 0.0, 0.0,
    #                    1.0, 0.0, 396.234527588, -396.33795166, -55.0000038147, 1.0)
    goalrotmat4 = Mat4(-0.707106769085,-0.707106769085,0.0,0.0,0.707106769085,-0.707106769085,0.0,0.0,0.0,0.0,1.0,0.0,406.33795166,103.76546859741,-55.0000038147,1.0)
    # goalrotmat4 = Mat4(-1.0,1.22464685259e-16,0.0,0.0,-1.22464685259e-16,-1.0,0.0,0.0,0.0,0.0,1.0,0.0,294.747955322,-200.073135376,-55.0000038147,1.0)
    # goalrotmat4 = Mat4(-0.707106769085,0.707106769085,0.0,0.0,-0.707106769085,-0.707106769085,0.0,0.0,0.0,0.0,1.0,0.0,396.234527588,-396.33795166,-55.0000038147,1.0)
    objstart = pg.genObjmnp(objpath, color=Vec4(.3, .0, .0, .1))
    objstart.setMat(startrotmat4)
    objend = pg.genObjmnp(objpath, color=Vec4(.0, .3, .0, .1))
    objend.setMat(goalrotmat4)

    objstart.reparentTo(base.render)
    objend.reparentTo(base.render)
    #tool
    # startrotmat4 = Mat4(-0.0176398064941,-0.0176398064941,-0.99968880415,0.0,-0.707106769085,0.707106769085,0.0,0.0,0.706886708736,0.706886708736,-0.0249464549124,0.0,225.010162354,100,44.9175643921,1.0)
    # startrotmat4 = Mat4(0.129405856133,0.129405856133,0.98311150074,0.0,0.707106769085,-0.707106769085,0.0,0.0,0.69516479969,0.69516479969,-0.183007523417,0.0,227.126983643,-327.023590088,74.7615509033,1.0)
    # goalrotmat4 = Mat4(-1.0,1.22464685259e-16,0.0,0.0,-1.22464685259e-16,-1.0,0.0,0.0,0.0,0.0,1.0,0.0,294.747955322,-300.0731293559074,-3.99246982852e-06,1.0)
    # planewheel
    # startrotmat4 = Mat4(0.707106769085,0.707106769085,0.0,0.0,-4.32978030171e-17,4.32978030171e-17,-1.0,0.0,-0.707106769085,0.707106769085,6.12323426293e-17,0.0,400.0,-400.0,29.9999980927,1.0)
    # goalrotmat4 = Mat4(0.707106769085,-0.707106769085,0.0,0.0,4.32978030171e-17,4.32978030171e-17,-1.0,0.0,0.707106769085,0.707106769085,6.12323426293e-17,0.0,400.0,-1.7017070846e-15,29.9999980927,1.0)
    # planelowerbody
    # startrotmat4 = Mat4(1.35963113277e-32,6.12323426293e-17,-1.0,0.0,-1.0,2.22044604925e-16,0.0,0.0,2.22044604925e-16,1.0,6.12323426293e-17,0.0,399.997558594,-16.3771038055,74.2884140015,1.0)
    # goalrotmat4 = Mat4(1.35963113277e-32,6.12323426293e-17,-1.0,0.0,-1.0,2.22044604925e-16,0.0,0.0,2.22044604925e-16,1.0,6.12323426293e-17,0.0,399.997558594,-216.377105713,74.2884140015,1.0)
    # planefrontstay
    # startrotmat4 = Mat4(0.707106769085,-0.707106769085,0.0,0.0,0.707106769085,0.707106769085,0.0,0.0,0.0,0.0,1.0,0.0,399.983917236,-399.987426758,8.91688500815e-07,1.0)
    # goalrotmat4 = Mat4(-4.32978030171e-17,-4.32978030171e-17,-1.0,0.0,0.707106769085,-0.707106769085,0.0,0.0,-0.707106769085,-0.707106769085,6.12323426293e-17,0.0,424.570098877,224.573623657,14.9999990463,1.0)
    # goalrotmat4 = Mat4(-6.12323426293e-17,7.49879952079e-33,1.0,0.0,-1.22464685259e-16,-1.0,0.0,0.0,1.0,-1.22464685259e-16,6.12323426293e-17,0.0,465.250152588,200.002487183,15.000002861,1.0)
    # planerearstay
    # startrotmat4 = Mat4(-0.771227538586,-0.409778445959,-0.487123966217,0.0,-0.00774340564385,0.771227538586,-0.636512517929,0.0,0.636512517929,-0.487123966217,-0.597964942455,0.0,293.626312256,-47.1169815063,-13.2552099228,1.0)
    # goalrotmat4 = Mat4(0.705433428288,0.705435693264,0.0687321424484,0.0,-0.707105636597,0.707107901573,-4.63324249722e-05,0.0,-0.0486337244511,-0.04856820032,0.997635126114,0.0,491.219848633,-108.780509949,11.8269386292,1.0)

    # import time
    # tic = time.clock()
    regrip.findshortestpath(startrotmat4, goalrotmat4, base)
    # toc = time.clock()
    print regrip.directshortestpaths
    # print toc-tic
    # assert False
    #
    id = 1
    pltfig = plt.figure()
    regrip.plotgraph(pltfig)
    regrip.plotshortestpath(pltfig, id = id)
    plt.axis("equal")
    plt.show()
    #

    if len(regrip.directshortestpaths) > 0:
        [objms, numikrms, jawwidth] = getMotionSequence(regrip, id = id)
        hrp5nmnp = [None]
        objmnp = [None]
        counter = [0]
        import time
        start = time.clock()
        def updateshow(objms, numikrms, jawwidth, hrp5nmnp, objmnp, counter, hrp5nrobot, handpkg, objpath, task):
            end = time.clock()
            if end-start > 25.0:
                if counter[0] < len(numikrms):
                    if hrp5nmnp[0] is not None:
                        hrp5nmnp[0].detachNode()
                    if objmnp[0] is not None:
                        objmnp[0].detachNode()
                    print counter[0]
                    print numikrms[counter[0]]
                    hrp5nrobot.movearmfkr(numikrms[counter[0]])
                    hrp5nmnp[0] = hrp5nplot.genmnp(hrp5nrobot, handpkg, jawwidthrgt=jawwidth[counter[0]])
                    hrp5nrobot.goinitpose()
                    hrp5nmnp[0].reparentTo(base.render)
                    objmnp[0] = pg.genObjmnp(objpath, color = Vec4(.7,.7,0,1))
                    objmnp[0].setMat(objms[counter[0]])
                    # pg.plotAxisSelf(base.render,objms[counter[0]].getRow3(3), objms[counter[0]])
                    objmnp[0].reparentTo(base.render)
                    counter[0] += 1
            return task.again
        taskMgr.doMethodLater(.1, updateshow, "updateshow",
                              extraArgs = [objms, numikrms, jawwidth, hrp5nmnp, objmnp, counter, hrp5nrobot, handpkg,objpath],
                              appendTask = True)

    # one time show for start and end

    # the plane is at 000
    # this_dir, this_filename = os.path.split(__file__)
    # ttpath = Filename.fromOsSpecific(
    #     os.path.join(os.path.split(this_dir)[0] + os.sep, "grip", "supports", "tabletop.egg"))
    # ttnodepath = NodePath("tabletop")
    # ttl = loader.loadModel(ttpath)
    # ttl.instanceTo(ttnodepath)
    # ttnodepath.reparentTo(base.render)

    base.run()