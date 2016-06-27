import numpy as np
from scipy import weave


def rodmat(axis, theta, mat = None):
    '''
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
    date: 20160615
    '''

    if mat == None:
        mat = np.eye(3,3)

    support = "#include <math.h>"
    code = """
        theta = theta * 3.1415926/180;

        double x = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
        double a = cos(theta / 2.0);
        double b = -(axis[0] / x) * sin(theta / 2.0);
        double c = -(axis[1] / x) * sin(theta / 2.0);
        double d = -(axis[2] / x) * sin(theta / 2.0);

        mat[0] = a*a + b*b - c*c - d*d;
        mat[1] = 2 * (b*c - a*d);
        mat[2] = 2 * (b*d + a*c);

        mat[3*1 + 0] = 2*(b*c+a*d);
        mat[3*1 + 1] = a*a+c*c-b*b-d*d;
        mat[3*1 + 2] = 2*(c*d-a*b);

        mat[3*2 + 0] = 2*(b*d-a*c);
        mat[3*2 + 1] = 2*(c*d+a*b);
        mat[3*2 + 2] = a*a+d*d-b*b-c*c;
    """

    weave.inline(code, ['axis', 'theta', 'mat'], support_code = support, libraries = ['m'])

    return mat


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
    mat[0,:] = mat[:,0]/np.linalg.norm(mat[:,0])
    mat[1,:] = np.cross(mat[2,:],mat[0,:])

    return mat