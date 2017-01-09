import numpy as np
import math
from scipy import weave


# def rodrigues(axis, theta, mat = None):
#     '''
#     Compute the rodrigues matrix using the given axis and theta
#
#     ## input
#     axis:
#         a 1-by-3 numpy array list
#     theta:
#         angle in degree
#     mat:
#         a 3-by-3 numpy array, rotation matrix if this was not given, users could get it from return
#
#     ## output
#     the mat
#
#     author: weiwei
#     date: 20160615
#     '''
#
#     if mat == None:
#         mat = np.eye(3,3)
#
#     support = "#include <math.h>"
#     code = """
#         double dth = static_cast<double>(theta);
#         dth = dth * 3.1415926/180.0;
#
#         double axis0 = static_cast<double>(axis[0]);
#         double axis1 = static_cast<double>(axis[1]);
#         double axis2 = static_cast<double>(axis[2]);
#         double x = sqrt(axis0*axis0 + axis1*axis1 + axis2*axis2);
#         double a = cos(dth / 2.0);
#         double b = -(axis0 / x) * sin(dth / 2.0);
#         double c = -(axis1 / x) * sin(dth / 2.0);
#         double d = -(axis2 / x) * sin(dth / 2.0);
#
#         mat[0] = a*a + b*b - c*c - d*d;
#         mat[3*1 + 0] = 2 * (b*c - a*d);
#         mat[3*2 + 0] = 2 * (b*d + a*c);
#
#         mat[1] = 2*(b*c+a*d);
#         mat[3*1 + 1] = a*a+c*c-b*b-d*d;
#         mat[3*2 + 1] = 2*(c*d-a*b);
#
#         mat[2] = 2*(b*d-a*c);
#         mat[3*1+ 2] = 2*(c*d+a*b);
#         mat[3*2 + 2] = a*a+d*d-b*b-c*c;
#     """
#
#     weave.inline(code, ['axis', 'theta', 'mat'], support_code = support, libraries = ['m'])
#
#     return mat

def rodrigues(axis, theta):
    """
    Compute the rodrigues matrix using the given axis and theta

    ## input
    axis:
        a 1-by-3 numpy array list
    theta:
        angle in degree
    mat:
        a 3-by-3 numpy array, rotation matrix if this was not given, users could get it from return

    ## output
    the mat

    author: weiwei
    date: 20161220
    """

    theta = theta*math.pi/180
    axis = np.asarray(axis)
    axis = axis/math.sqrt(np.dot(axis, axis))
    a = math.cos(theta/2.0)
    b, c, d = -axis*math.sin(theta/2.0)
    aa, bb, cc, dd = a*a, b*b, c*c, d*d
    bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
    return np.array([[aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac)],
                     [2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab)],
                     [2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc]])


def rotmatfacet(facetnormal, facetfirstpoint, facetsecondpoint):
    '''
    Compute the rotation matrix of a 3D facet using
    facetnormal and the first two points on the facet
    The function uses the concepts defined by Trimesh

    ## input:
    facetnormal:
        the normal of a facet
    facetfirstpoint:
        the first point of the first triangle on the facet
    facetsecondpoint:
        the second point of the first triangle on the facet

    ## output:
    mat:
        a (3,3) numpy matrix

    date: 20160624
    author: weiwei
    '''

    mat = np.eye(3,3)
    mat[2,:] = facetnormal
    mat[0,:] = facetsecondpoint-facetfirstpoint
    mat[0,:] = mat[0,:]/np.linalg.norm(mat[0,:])
    mat[1,:] = np.cross(mat[2,:],mat[0,:])

    return mat

def homoinverse(homomatrix4):
    """
    compute the inverse of a homogeneous transform

    :param homomatrix4:
    :return:

    author: weiwei
    date :20161213
    """

    rotmat = homomatrix4[:3, :3]
    tranvec = homomatrix4[:3, 3]
    invmatrix4 = np.eye(4,4)
    invmatrix4[:3, :3] = np.transpose(rotmat)
    invmatrix4[:3, 3] = -np.dot(np.transpose(rotmat), tranvec)

    return invmatrix4

def transformmat4(matrix4, point):
    """
    do homotransform on point using matrix4

    :param matrix:
    :param point:
    :return:

    author: weiwei
    date: 20161213
    """

    point4 = np.array([point[0], point[1], point[2], 1])
    return np.dot(matrix4, point4)