#!/usr/bin/python

"""
The script is written following
http://myenigma.hatenablog.com/entry/2016/03/23/092002
the original file was 2d

author: weiwei
date: 20170609
"""

import random
import copy
import numpy as np


class RRTConnect(object):

    def __init__(self, start, goal, iscollidedfunc, jointlimits, expanddis=.3,
                 starttreesamplerate=10, endtreesamplerate=100, maxiter=5000,
                 robot = None, robotplot = None, handpkg = None):
        """

        :param start: nd point, list
        :param goal: nd point, list
        :param iscollidedfunc: a function returns whether collided or not
        :param jointlimits: [[join0low, joint0high], [joint1low, joint1high], ...]
        :param expandDis: how much to expand along the vector randnode - nearestnode
        :param starttreesamplerate: bias to set randnode to be goal
        :param endtreesamplerate: bias to set randnode to be start
        :param maxIter:

        :param the last three parameters are for robotsim robots

        author: weiwei
        date: 20170609
        """

        self.__start = np.asarray(start)
        self.__end = np.asarray(goal)

        self.__iscollidedcallback = iscollidedfunc
        self.__jointlimits = jointlimits
        self.__expanddis = expanddis
        self.__starttreesamplerate = starttreesamplerate
        self.__endtreesamplerate = endtreesamplerate
        self.__maxiter = maxiter

        self.__robot = robot
        self.__robotplot = robotplot
        self.__handpkg = handpkg

        self.__nodeliststart = []
        self.__nodelistend = []

    @property
    def start(self):
        # read-only property
        return self.__start

    @property
    def end(self):
        # read-only property
        return self.__end

    @property
    def nodeliststart(self):
        # read-only property
        return self.__nodeliststart

    @property
    def nodelistend(self):
        # read-only property
        return self.__nodelistend

    def planning(self, obstaclelist=[], animation=False):
        """
        Pathplanning

        animation: flag for animation on or off

        :return path [[joint0, joint1, ...], [joint0, joint1, ...], ...]
        """

        itercount = 0

        # one sampled point: [point, iscollided]
        sampledpoints = []

        self.__nodeliststart = [Node(self.__start)]
        self.__nodelistend = [Node(self.__end)]

        starttreegoal = self.__end
        endtreegoal = self.__start
        while True:

            if itercount > self.__maxiter:
                print "failed to find a path"
                break

            # if self.__starttreesamplerate < 80:
            #     self.__starttreesamplerate += itercount/float(self.__maxiter)*10
            # else:
            #     self.__starttreesamplerate = 80
            # print self.__starttreesamplerate

            # Random Sampling
            randnode = []
            if random.randint(0, 100) > self.__starttreesamplerate:
                for i,jntrng in enumerate(self.__jointlimits):
                    randnode.append(random.uniform(jntrng[0], jntrng[1]))
                randnode = np.asarray(randnode)
            else:
                randnode = copy.deepcopy(starttreegoal)

            # Find nearest node
            nind = self.getNearestListIndex(self.__nodeliststart, randnode)
            vec = randnode-self.__nodeliststart[nind].point
            vec = vec/np.linalg.norm(vec)

            # expand tree
            nearestnode = self.__nodeliststart[nind]
            newnode = copy.deepcopy(nearestnode)
            newnode.point += self.__expanddis * vec
            newnode.parent = nind

            if self.__iscollidedcallback(newnode.point, obstaclelist,
                                         self.__robot, self.__robotplot, self.__handpkg):
                sampledpoints.append([newnode.point, True])
                bswap = False
                # if collided, try the other tree
                while True:
                    randnode = []
                    if random.randint(0, 100) > self.__endtreesamplerate:
                        for i,jntrng in enumerate(self.__jointlimits):
                            randnode.append(random.uniform(jntrng[0], jntrng[1]))
                        randnode = np.asarray(randnode)
                    else:
                        randnode = copy.deepcopy(endtreegoal)

                    # Find nearest node
                    nind = self.getNearestListIndex(self.__nodelistend, randnode)
                    vec = randnode-self.__nodelistend[nind].point
                    vec = vec/np.linalg.norm(vec)

                    # expand tree
                    nearestnode = self.__nodelistend[nind]
                    newnode = copy.deepcopy(nearestnode)
                    newnode.point += self.__expanddis * vec
                    newnode.parent = nind

                    if self.__iscollidedcallback(newnode.point, obstaclelist,
                                                 self.__robot, self.__robotplot, self.__handpkg):
                        sampledpoints.append([newnode.point, True])
                        bswap = True
                        break

                    sampledpoints.append([newnode.point, False])
                    self.__nodelistend.append(newnode)
                    starttreegoal = newnode.point# check goal

                    d = np.linalg.norm(newnode.point - endtreegoal)
                    if d <= self.__expanddis:
                        print("reaching the goal")
                        bswap = False
                        break

                    if animation:
                        drawwspace(self, obstaclelist, randnode)

                if bswap:
                    continue
                else:
                    break

            sampledpoints.append([newnode.point, False])
            self.__nodeliststart.append(newnode)
            endtreegoal = newnode.point

            # check goal
            d = np.linalg.norm(newnode.point - starttreegoal)
            if d <= self.__expanddis:
                print("reaching the goal")
                break

            if animation:
                drawwspace(self, obstaclelist, randnode)

            itercount += 1

        path = []
        lastindex = len(self.__nodelistend) - 1
        # WARNING the first nodelist is a numpyarray
        while self.__nodelistend[lastindex].parent is not None:
            node = self.__nodelistend[lastindex]
            path.append(node.point.tolist())
            lastindex = node.parent
        path.append(self.__end.tolist())
        path = path[::-1]
        lastindex = len(self.__nodeliststart) - 1
        # WARNING the first nodelist is a numpyarray
        while self.__nodeliststart[lastindex].parent is not None:
            node = self.__nodeliststart[lastindex]
            path.append(node.point.tolist())
            lastindex = node.parent
        path.append(self.__start.tolist())

        return [path, sampledpoints]

    def getNearestListIndex(self, nodelist, randnode):
        dlist = [np.linalg.norm(randnode-node.point) for node in nodelist]
        minind = dlist.index(min(dlist))
        return minind

class Node():
    """
    RRT Node
    """

    def __init__(self, point):
        """

        :param point: nd point, numpyarray

        author: weiwei
        date: 20170609
        """

        self.point = point
        self.parent = None


def iscollidedfunc(point, obstacleList, robot = None, robotplot = None, handpkg = None):
    for (obpos, size) in obstacleList:
        d = np.linalg.norm(np.asarray(obpos) - point)
        if d <= size:
            return True  # collision

    return False  # safe

def drawwspace(planner, obstaclelist, randconfiguration=None):
    """
    Draw Graph
    """
    import matplotlib.pyplot as plt
    plt.clf()
    if randconfiguration is not None:
        plt.plot(randconfiguration[0], randconfiguration[1], "^k")
    for node in planner.nodeliststart:
        if node.parent is not None:
            plt.plot([node.point[0], planner.nodeliststart[node.parent].point[0]],
                     [node.point[1], planner.nodeliststart[node.parent].point[1]], '-g')
    for node in planner.nodelistend:
        if node.parent is not None:
            plt.plot([node.point[0], planner.nodelistend[node.parent].point[0]],
                     [node.point[1], planner.nodelistend[node.parent].point[1]], '-b')
    plt.plot([point[0] for (point, size) in obstaclelist], [point[1] for (point, size) in obstaclelist], "ok", ms=size * 20)
    plt.plot(planner.start[0], planner.start[1], "xr")
    plt.plot(planner.end[0], planner.end[1], "xr")
    plt.axis([-2, 15, -2, 15])
    plt.grid(True)
    plt.pause(0.001)

if __name__ == '__main__':
    import matplotlib.pyplot as plt

    # ====Search Path with RRT====
    obstaclelist = [
        ((5, 5), 1),
        ((3, 6), 2),
        ((3, 8), 2),
        ((3, 10), 2),
        ((7, 5), 2),
        ((9, 5), 2)
    ]  # [x,y,size]
    # Set Initial parameters
    rrtc = RRTConnect(start=[0.0, 0.0], goal=[5.0, 10.0], iscollidedfunc = iscollidedfunc,
              jointlimits = [[-2.0, 15.0], [-2.0, 15.0]])

    import time
    tic = time.clock()
    path = rrtc.planning(obstaclelist=obstaclelist, animation=True)
    toc = time.clock()
    print toc-tic

    # Draw final path
    drawwspace(rrtc, obstaclelist)
    plt.plot([point[0] for point in path], [point[1] for point in path], '-r')
    plt.grid(True)
    plt.pause(0.001)  # Need for Mac
    plt.show()