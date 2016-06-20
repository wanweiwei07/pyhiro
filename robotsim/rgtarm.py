import numpy as np
import geomutils.robotmath as rm


def initrgtlj():
    '''
    Init the structure of hiro's rgt arm links and joints

    ## output
    rgtlj:
        a list of dictionaries with each dictionary holding name, mother, child,
    linkpos (link position), linkvec (link vector), rotmat (orientation), rotax (rotation axis of a joint),
    rotangle (rotation angle of the joint around rotax)
    linkend (the end position of a link) is computed using rotmat, rotax, linkpos, and linklen
    lj means link and joint, the joint attached to the link is at the linkend

    ## note
    x is facing forward, y is facing left, z is facing upward
    each element of rgtlj is a dictionary
    rgtlj[i]['linkpos'] indicates the position of a link
    rgtlj[i]['linkvec'] indicates the vector of a link that points from start to end
    rgtlj[i]['rotmat'] indicates the frame of this link
    rgtlj[i]['rotax'] indicates the rotation axis of the link
    rgtlj[i]['rotangle'] indicates the rotation angle of the link around the rotax
    rgtlj[i]['linkend'] indicates the end position of the link (passively computed)

    ## more note:
    rgtlj[1]['linkpos'] is the position of the first joint
    rgtlj[i]['linkend'] is the same as rgtlj[i+1]['linkpos'],
    I am keeping this value for the eef (end-effector)

    ## even more note:
    joint is attached to the linkpos of a link
    for the first link, the joint is fixed and the rotax = 0,0,0

    author: weiwei
    date: 20160615
    '''

    # create a arm with six joints
    rgtlj = [{}]*7

    # the 0th link
    rgtlj[0]['name'] = 'link0'
    rgtlj[0]['mother'] = -1
    rgtlj[0]['child'] = 1
    rgtlj[0]['linkpos'] = np.array([0,0,0])
    rgtlj[0]['linkvec'] = np.array([0,-0.145,0.370296])
    # TODO: consider changing this one into inherentR
    rgtlj[0]['rotangle'] = 0
    rgtlj[0]['rotmat'] = np.array([[1,0,0],
                                    [0,1,0],
                                    [0,0,1]])
    # this is actually useless
    rgtlj[0]['rotax'] = np.array([0,0,0])
    rgtlj[0]['linkend'] = np.dot(rgtlj[0]['rotmat'], rgtlj[0]['linkvec'].reshape((-1,1))).reshape((1,-1))+rgtlj[0]['linkpos']

    # the 1st link
    rgtlj[1]['name'] = 'link1'
    rgtlj[1]['mother'] = 0
    rgtlj[1]['child'] = 2
    rgtlj[1]['linkpos'] = rgtlj[0]['linkend']
    rgtlj[1]['linkvec'] = np.array([0,-0.095,0])
    rgtlj[1]['rotax'] = np.array([0,0,1])
    rgtlj[1]['rotangle'] = 0
    rgtlj[1]['rotmat'] = np.dot(rgtlj[0]['rotmat'], rm.rodmat(rgtlj[1]['rotax'], rgtlj[1]['rotangle']))
    rgtlj[1]['linkend'] = np.dot(rgtlj[1]['rotmat'], rgtlj[1]['linkvec'].reshape((-1,1))).reshape((1,-1))+rgtlj[1]['linkpos']

    # the 2nd link
    rgtlj[2]['name'] = 'link2'
    rgtlj[2]['mother'] = 1
    rgtlj[2]['child'] = 3
    rgtlj[2]['linkpos'] = rgtlj[1]['linkend']
    rgtlj[2]['linkvec'] = np.array([-0.03,0,-0.25])
    rgtlj[2]['rotax'] = np.array([0,1,0])
    rgtlj[2]['rotangle'] = 0
    rgtlj[2]['rotmat'] = np.dot(rgtlj[1]['rotmat'], rm.rodmat(rgtlj[2]['rotax'], rgtlj[2]['rotangle']))
    rgtlj[2]['linkend'] = np.dot(rgtlj[2]['rotmat'], rgtlj[2]['linkvec'].reshape((-1,1))).reshape((1,-1))+rgtlj[2]['linkpos']

    # the 3rd link
    rgtlj[3]['name'] = 'link3'
    rgtlj[3]['mother'] = 2
    rgtlj[3]['child'] = 4
    rgtlj[3]['linkpos'] = rgtlj[2]['linkend']
    rgtlj[3]['linkvec'] = np.array([0,0,0])
    rgtlj[3]['rotax'] = np.array([0,1,0])
    rgtlj[3]['rotangle'] = 0
    rgtlj[3]['rotmat'] = np.dot(rgtlj[2]['rotmat'], rm.rodmat(rgtlj[3]['rotax'], rgtlj[3]['rotangle']))
    rgtlj[3]['linkend'] = np.dot(rgtlj[3]['rotmat'], rgtlj[3]['linkvec'].reshape((-1,1))).reshape((1,-1))+rgtlj[3]['linkpos']

    # the 4th link
    rgtlj[4]['name'] = 'link4'
    rgtlj[4]['mother'] = 3
    rgtlj[4]['child'] = 5
    rgtlj[4]['linkpos'] = rgtlj[3]['linkend']
    rgtlj[4]['linkvec'] = np.array([0,0,-0.235])
    rgtlj[4]['rotax'] = np.array([0,0,1])
    rgtlj[4]['rotangle'] = 0
    rgtlj[4]['rotmat'] = np.dot(rgtlj[3]['rotmat'], rm.rodmat(rgtlj[4]['rotax'], rgtlj[4]['rotangle']))
    rgtlj[4]['linkend'] = np.dot(rgtlj[4]['rotmat'], rgtlj[4]['linkvec'].reshape((-1,1))).reshape((1,-1))+rgtlj[4]['linkpos']

    # the 5th link
    rgtlj[5]['name'] = 'link5'
    rgtlj[5]['mother'] = 4
    rgtlj[5]['child'] = 6
    rgtlj[5]['linkpos'] = rgtlj[4]['linkend']
    rgtlj[5]['linkvec'] = np.array([0,0,-0.09])
    rgtlj[5]['rotax'] = np.array([0,1,0])
    rgtlj[5]['rotangle'] = 0
    rgtlj[5]['rotmat'] = np.dot(rgtlj[4]['rotmat'], rm.rodmat(rgtlj[5]['rotax'], rgtlj[5]['rotangle']))
    rgtlj[5]['linkend'] = np.dot(rgtlj[5]['rotmat'], rgtlj[5]['linkvec'].reshape((-1,1))).reshape((1,-1))+rgtlj[5]['linkpos']

    # the 6th link
    rgtlj[6]['name'] = 'link6'
    rgtlj[6]['mother'] = 5
    rgtlj[6]['child'] = -1
    rgtlj[6]['linkpos'] = rgtlj[5]['linkend']
    rgtlj[6]['linkvec'] = np.array([-0.15,0,0])
    rgtlj[6]['rotax'] = np.array([1,0,0])
    rgtlj[6]['rotangle'] = 0
    rgtlj[6]['rotmat'] = np.dot(rgtlj[5]['rotmat'], rm.rodmat(rgtlj[6]['rotax'], rgtlj[6]['rotangle']))
    rgtlj[6]['linkend'] = np.dot(rgtlj[6]['rotmat'], rgtlj[6]['linkvec'].reshape((-1,1))).reshape((1,-1))+rgtlj[6]['linkpos']

    return rgtlj


def _updatergtlj(rgtlj):
    '''
    Update the structure of hiro's rgt arm links and joints
    Note that this function should not be called explicitly
    It is called automatically by functions like movexxx

    ## output
    rgtlj:
        the datastructure defined in initrgtlj() function

    author: weiwei
    date: 20160615
    '''

    i = 1
    while i!=0:
        j = rgtlj[i]['mother']
        rgtlj[i]['linkpos'] = rgtlj[j]['linkend']
        rgtlj[i]['rotmat'] = np.dot(rgtlj[j]['rotmat'], rm.rodmat(rgtlj[i]['rotax'], rgtlj[i]['rotangle']))
        rgtlj[i]['linkend'] = np.dot(rgtlj[i]['rotmat'], rgtlj[i]['linkvec'].reshape((-1,1))).reshape((1,-1))+rgtlj[i]['linkpos']
        i = rgtlj[i]['child']
    return rgtlj

if __name__=="__main__":
    rgtlj = initrgtlj()

    # show in panda3d
    from direct.showbase.ShowBase import ShowBase
    from panda3d.core import *
    from panda3d.bullet import BulletSphereShape
    import plot.pandageom as pandageom
    import plot.pandactrl as pandactrl
    from direct.filter.CommonFilters import CommonFilters

    base = ShowBase()

    # arrow = genArrow(base, 0.3, 0.02)
    # print arrow.ls()
    # arrow.reparentTo(base.render)
    # arrow = genArrow(base, 0.5)

    # arrowpholder = base.render.attachNewNode("arrowholder1")
    # arrowpholder.setPos(0,0,0)
    # arrow.instanceTo(arrowpholder)
    # arrowpholder2 = base.render.attachNewNode("arrowholder2")
    # arrowpholder2.setPos(2,0,0)
    # arrow.instanceTo(arrowpholder2)

    pandactrl.setCam(base, 0, 3, 2, 'perspective')
    pandactrl.setLight(base)
    pandactrl.setRenderEffect(base)
    pandageom.plotArrow(base, spos=np.array([0,0,0]), epos=np.array([0.5,0,0]), rgba=[1,0,0,1])
    pandageom.plotArrow(base, spos=np.array([0,0,0]), epos=np.array([0,0.5,0]), rgba=[0,1,0,1])
    pandageom.plotArrow(base, spos=np.array([0,0,0]), epos=np.array([0,0,0.5]), rgba=[0,0,1,1])

    # base.disableMouse()
    # lens = OrthographicLens()
    # lens.setFilmSize(3, 3)

    # base.camera.setPos(0, 15, 0)
    # base.camera.lookAt(0, 0, 0)
    # # add mouse control task
    # base.taskMgr.add(pandactrl.mouseTask, "mouseControl", extraArgs=[base])
    # base.trackball.node().lookAt(0, 0, 0)
    # base.disableMouse()
    # base.trackball.node().setPos(0, 10, 0)
    # base.trackball.node().setOrigin(Vec3(0,0,0))
    # base.enableMouse()
    # base.oobe()

    base.run()
