# plot the shortest path using hrp5robot

from robotsim.hrp5 import hrp5
from robotsim.hrp5 import hrp5plot

from manipulation.regrasp import regriptpp
from database import dbaccess as db
import pandaplotutils.pandactrl as pandactrl
from panda3d.core import *
from pandaplotutils import pandageom as pg
import matplotlib.pyplot as plt
import os
import trimesh

def getMotionSequence(regrip):
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

    pathnidlist = regrip.directshortestpaths[0]
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
            jawwidth.append(regrip.rtq85hnd.jawwidthopen)
            numikrlist.append(armjntsgrp)
            jawwidth.append(regrip.rtq85hnd.jawwidthopen)
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
            jawwidth.append(regrip.rtq85hnd.jawwidthopen)
            numikrlist.append(armjntsgrphandx)
            jawwidth.append(regrip.rtq85hnd.jawwidthopen)
            numikrlist.append(armjntsgrphandxworldz)
            jawwidth.append(regrip.rtq85hnd.jawwidthopen)
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
                jawwidth.append(regrip.rtq85hnd.jawwidthopen)
                numikrlist.append(armjntsgrp)
                jawwidth.append(regrip.rtq85hnd.jawwidthopen)
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
                numikrlist.append([0,0,armjntsgrpworldaworldz])
                jawwidth.append(grpjawwidth)
                numikrlist.append([0,0,armjntsgrpworlda])
                jawwidth.append(grpjawwidth)
                numikrlist.append([0,0,armjntsgrp])
                jawwidth.append(grpjawwidth)
                numikrlist.append([0,0,armjntsgrp])
                jawwidth.append(regrip.rtq85hnd.jawwidthopen)
                numikrlist.append([0,0,armjntsgrphandx])
                jawwidth.append(regrip.rtq85hnd.jawwidthopen)
                numikrlist.append([0,0,armjntsgrphandxworldz])
                jawwidth.append(regrip.rtq85hnd.jawwidthopen)
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
                        numikrlist.append([0,0,armjntsgrphandxworldz])
                        jawwidth.append(regrip.rtq85hnd.jawwidthopen)
                        numikrlist.append([0,0,armjntsgrphandx])
                        jawwidth.append(regrip.rtq85hnd.jawwidthopen)
                        numikrlist.append([0,0,armjntsgrp])
                        jawwidth.append(regrip.rtq85hnd.jawwidthopen)
                        numikrlist.append([0,0,armjntsgrp])
                        jawwidth.append(grpjawwidth)
                        numikrlist.append([0,0,armjntsgrpworlda])
                        jawwidth.append(grpjawwidth)
                        numikrlist.append([0,0,armjntsgrpworldaworldz])
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
                        numikrlist.append([0,0,armjntsgrpworldaworldz])
                        jawwidth.append(grpjawwidth)
                        numikrlist.append([0,0,armjntsgrpworlda])
                        jawwidth.append(grpjawwidth)
                        numikrlist.append([0,0,armjntsgrp])
                        jawwidth.append(grpjawwidth)
                        numikrlist.append([0,0,armjntsgrp])
                        jawwidth.append(regrip.rtq85hnd.jawwidthopen)
                        numikrlist.append([0,0,armjntsgrphandx])
                        jawwidth.append(regrip.rtq85hnd.jawwidthopen)
                        numikrlist.append([0,0,armjntsgrphandxworldz])
                        jawwidth.append(regrip.rtq85hnd.jawwidthopen)
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
                    numikrlist.append([0,0,armjntsgrphandxworldz])
                    jawwidth.append(regrip.rtq85hnd.jawwidthopen)
                    numikrlist.append([0,0,armjntsgrphandx])
                    jawwidth.append(regrip.rtq85hnd.jawwidthopen)
                    numikrlist.append([0,0,armjntsgrp])
                    jawwidth.append(regrip.rtq85hnd.jawwidthopen)
                    numikrlist.append([0,0,armjntsgrp])
                    jawwidth.append(grpjawwidth)
                    numikrlist.append([0,0,armjntsgrpworlda])
                    jawwidth.append(grpjawwidth)
                    numikrlist.append([0,0,armjntsgrpworldaworldz])
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
                    jawwidth.append(regrip.rtq85hnd.jawwidthopen)
                    numikrlist.append(armjntsgrphandx)
                    jawwidth.append(regrip.rtq85hnd.jawwidthopen)
                    numikrlist.append(armjntsgrphandxworldz)
                    jawwidth.append(regrip.rtq85hnd.jawwidthopen)
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
    hrp5robot = hrp5.Hrp5Robot()

    base = pandactrl.World(camp=[0,0,5000], lookatp=[0,0,0])

    # ttube.stl
    this_dir, this_filename = os.path.split(__file__)
    objpath = os.path.join(os.path.split(this_dir)[0], "grip", "objects", "ttube.stl")
    # objpath = os.path.join(os.path.split(this_dir)[0], "grip", "objects", "tool.stl")
    # objpath = os.path.join(os.path.split(this_dir)[0], "grip", "objects", "planewheel.stl")
    # objpath = os.path.join(os.path.split(this_dir)[0], "grip", "objects", "planelowerbody.stl")
    # objpath = os.path.join(os.path.split(this_dir)[0], "grip", "objects", "planefrontstay.stl")
    # objpath = os.path.join(os.path.split(this_dir)[0], "grip", "objects", "planerearstay.stl")
    regrip = regriptpp.RegripTpp(objpath, hrp5robot, gdb)

    # ttube
    startrotmat4 = Mat4(0.707106769085,-0.707106769085,0.0,0.0,
                        4.32978030171e-17,4.32978030171e-17,1.0,0.0,
                        -0.707106769085,-0.707106769085,6.12323426293e-17,0.0,
                        250.996246338,-509.99848938,45.0004844666,1.0)
    goalrotmat4 = Mat4(-0.707106769085, -0.70710682869, 0.0, 0.0,
                       4.32978063259e-17, -4.32978030171e-17, -1.0, 0.0,
                       0.70710682869, -0.707106769085, 6.12323426293e-17, 0.0,
                       400.003753662, 400.001525879, 44.9995155334, 1.0)
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

    import time
    tic = time.clock()
    regrip.findshortestpath(startrotmat4, goalrotmat4, base)
    toc = time.clock()
    print len(regrip.directshortestpaths[0])
    print toc-tic
    # assert False

    pltfig = plt.figure()
    regrip.plotgraph(pltfig)
    regrip.plotshortestpath(pltfig)
    plt.axis("equal")
    plt.show()

    [objms, numikrms, jawwidth] = getMotionSequence(regrip)
    hrp5mnp = [None]
    objmnp = [None]
    counter = [0]
    def updateshow(objms, numikrms, jawwidth, hrp5mnp, objmnp, counter, hrp5robot, objpath, task):
        if counter[0] < len(numikrms):
            if hrp5mnp[0] is not None:
                hrp5mnp[0].detachNode()
            if objmnp[0] is not None:
                objmnp[0].detachNode()
            print counter[0]
            print numikrms[counter[0]]
            hrp5robot.movearmfkr(numikrms[counter[0]])
            hrp5mnp[0] = hrp5plot.genHrp5mnp(hrp5robot, jawwidthrgt=jawwidth[counter[0]])
            hrp5robot.goinitpose()
            hrp5mnp[0].reparentTo(base.render)
            objmnp[0] = pg.genObjmnp(objpath, color = Vec4(.7,.7,0,1))
            objmnp[0].setMat(objms[counter[0]])
            # pg.plotAxisSelf(base.render,objms[counter[0]].getRow3(3), objms[counter[0]])
            objmnp[0].reparentTo(base.render)
            counter[0] += 1
        return task.again
    taskMgr.doMethodLater(1, updateshow, "updateshow",
                          extraArgs = [objms, numikrms, jawwidth, hrp5mnp, objmnp, counter, hrp5robot, objpath],
                          appendTask = True)

    base.run()