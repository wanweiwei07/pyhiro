import numpy as np

def initrgtarm():
    '''
    Init the structure of hiro's rgt arm

    :return:
    rgtarm, a list of dictionaries with each dictionary holding name, mother, child,
    linkpos (link position), linkvec (link vector), rotmat (orientation), rotax (rotation axis of a joint),
    rotangle (rotation angle of the joint around rotax)
    linkend (the end position of a link) is computed using rotmat, rotax, linkpos, and linklen

    note:
    x is facing forward, y is facing left, z is facing upward
    each element of rgtarm is a dictionary
    rgtarm[i]['linkpos'] indicates the position of a link
    rgtarm[i]['linkvec'] indicates the vector of a link that points from start to end
    rgtarm[i]['rotmat'] indicates the frame of this link
    rgtarm[i]['rotax'] indicates the rotation axis of the link
    rgtarm[i]['rotangle'] indicates the rotation angle of the link around the rotax
    rgtarm[i]['linkend'] indicates the end position of the link (passively computed)

    more note:
    rgtarm[1]['linkpos'] is the position of the first joint
    rgtarm[i]['linkend'] is the same as rgtarm[i+1]['linkpos'],
    I am keeping this value for the eef (end-effector)

    author: weiwei
    date: 20160615
    '''

    # create a arm with six joints
    rgtarm = [None]*6

    # the first link
    rgtarm[0]['name'] = 'link1'
    rgtarm[0]['mother'] = 0
    rgtarm[0]['child'] = 1
    rgtarm[0]['linkpos'] = np.array([0,0,0])
    rgtarm[0]['linkvec'] = np.array([0,-0.145,0.370296])
    rgtarm[0]['rotmat'] = np.array([[1,0,0],
                                    [0,1,0],
                                    [0,0,1]])
    rgtarm[0]['rotax'] = np.array([0,0,1])
    rgtarm[0]['rotangle'] = 0
    rgtarm[0]['linkend'] = np.dot(rgtarm[0]['rotmat'], rgtarm[0]['linkvec'].reshape((-1,1))).reshape((1,-1))+rgtarm[0]['linkpos']

    # the second link
    rgtarm[0]['name'] = 'link1'
    rgtarm[0]['mother'] = 0
    rgtarm[0]['child'] = 1
    rgtarm[0]['linkpos'] = np.array([0,0,0])
    rgtarm[0]['linkvec'] = np.array([0,-0.145,0.370296])
    rgtarm[0]['rotmat'] = np.array([[1,0,0],
                                    [0,1,0],
                                    [0,0,1]])
    rgtarm[0]['rotax'] = np.array([0,0,1])
    rgtarm[0]['rotangle'] = 0
    rgtarm[0]['linkend'] = np.dot(rgtarm[0]['rotmat'], rgtarm[0]['linkvec'].reshape((-1,1))).reshape((1,-1))+rgtarm[0]['linkpos']

    # the third link
    rgtarm[0]['name'] = 'link1'
    rgtarm[0]['mother'] = 0
    rgtarm[0]['child'] = 1
    rgtarm[0]['linkpos'] = np.array([0,0,0])
    rgtarm[0]['linkvec'] = np.array([0,-0.145,0.370296])
    rgtarm[0]['rotmat'] = np.array([[1,0,0],
                                    [0,1,0],
                                    [0,0,1]])
    rgtarm[0]['rotax'] = np.array([0,0,1])
    rgtarm[0]['rotangle'] = 0
    rgtarm[0]['linkend'] = np.dot(rgtarm[0]['rotmat'], rgtarm[0]['linkvec'].reshape((-1,1))).reshape((1,-1))+rgtarm[0]['linkpos']

    rgtarm[0]['name'] = 'link1'
    rgtarm[0]['mother'] = 0
    rgtarm[0]['child'] = 1
    rgtarm[0]['linkpos'] = np.array([0,0,0])
    rgtarm[0]['linkvec'] = np.array([0,-0.145,0.370296])
    rgtarm[0]['rotmat'] = np.array([[1,0,0],
                                    [0,1,0],
                                    [0,0,1]])
    rgtarm[0]['rotax'] = np.array([0,0,1])
    rgtarm[0]['rotangle'] = 0
    rgtarm[0]['linkend'] = np.dot(rgtarm[0]['rotmat'], rgtarm[0]['linkvec'].reshape((-1,1))).reshape((1,-1))+rgtarm[0]['linkpos']

    rgtarm[0]['name'] = 'link1'
    rgtarm[0]['mother'] = 0
    rgtarm[0]['child'] = 1
    rgtarm[0]['linkpos'] = np.array([0,0,0])
    rgtarm[0]['linkvec'] = np.array([0,-0.145,0.370296])
    rgtarm[0]['rotmat'] = np.array([[1,0,0],
                                    [0,1,0],
                                    [0,0,1]])
    rgtarm[0]['rotax'] = np.array([0,0,1])
    rgtarm[0]['rotangle'] = 0
    rgtarm[0]['linkend'] = np.dot(rgtarm[0]['rotmat'], rgtarm[0]['linkvec'].reshape((-1,1))).reshape((1,-1))+rgtarm[0]['linkpos']

    rgtarm[0]['name'] = 'link1'
    rgtarm[0]['mother'] = 0
    rgtarm[0]['child'] = 1
    rgtarm[0]['linkpos'] = np.array([0,0,0])
    rgtarm[0]['linkvec'] = np.array([0,-0.145,0.370296])
    rgtarm[0]['rotmat'] = np.array([[1,0,0],
                                    [0,1,0],
                                    [0,0,1]])
    rgtarm[0]['rotax'] = np.array([0,0,1])
    rgtarm[0]['rotangle'] = 0
    rgtarm[0]['linkend'] = np.dot(rgtarm[0]['rotmat'], rgtarm[0]['linkvec'].reshape((-1,1))).reshape((1,-1))+rgtarm[0]['linkpos']

