import numpy as np
from scipy import weave


def rodrigues(axis, theta, mat = None):
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
        double dth = static_cast<double>(theta);
        dth = dth * 3.1415926/180.0;

        int axis0 = static_cast<int>(axis[0]);
        int axis1 = static_cast<int>(axis[1]);
        int axis2 = static_cast<int>(axis[2]);
        double x = sqrt(axis0*axis0 + axis1*axis1 + axis2*axis2);
        double a = cos(dth / 2.0);
        double b = -(axis0 / x) * sin(dth / 2.0);
        double c = -(axis1 / x) * sin(dth / 2.0);
        double d = -(axis2 / x) * sin(dth / 2.0);

        mat[0] = a*a + b*b - c*c - d*d;
        mat[3*1 + 0] = 2 * (b*c - a*d);
        mat[3*2 + 0] = 2 * (b*d + a*c);

        mat[1] = 2*(b*c+a*d);
        mat[3*1 + 1] = a*a+c*c-b*b-d*d;
        mat[3*2 + 1] = 2*(c*d-a*b);

        mat[2] = 2*(b*d-a*c);
        mat[3*1+ 2] = 2*(c*d+a*b);
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