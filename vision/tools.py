from sklearn.neighbors import KDTree
import numpy as np
from sklearn import linear_model

def estimatenormals(points, npoints = 40, method = 'pca'):
    """
    estimate the normals of points

    :param points: an array of [x, y, z]
    :param method: 'pca' or 'ransac', theoretically ransac is more precise when there are more points
    :return: a list of normal vectors

    author: weiwei
    date: 20170714
    """

    pointsnormals = []
    camerapos = np.array([0.0,0.0,0.0])
    kdt = KDTree(points)

    if method == 'pca':
        regionpntidlist = kdt.query(points, k=npoints, return_distance=False)
        for i, pntidlist in enumerate(regionpntidlist):
            regionpnts = points[pntidlist]
            covmat = np.cov(regionpnts.T)
            eigvalues, eigmat = np.linalg.eig(covmat)
            idx = np.argmin(eigvalues)
            eigvec = eigmat[:, idx]
            if np.dot(eigvec, camerapos-points[i]) < 0:
                eigvec = -eigvec
            pointsnormals.append(eigvec)
    elif method == 'ransac':
        # NOTE: this part is not usable due to small npoints
        ransacer = linear_model.RANSACRegressor(linear_model.LinearRegression())
        regionpntidlist = kdt.query(points, k=npoints, return_distance=False)
        for i, pntidlist in enumerate(regionpntidlist):
            XYZ = points[pntidlist]
            ransacer.fit(XYZ[:, 0:2], XYZ[:, 2])
            inlier_mask = ransacer.inlier_mask_

            regionpnts = XYZ[inlier_mask]
            covmat = np.cov(regionpnts.T)
            eigvalues, eigmat = np.linalg.eig(covmat)
            idx = np.argmin(eigvalues)
            eigvec = eigmat[:, idx]
            if np.dot(eigvec, camerapos-points[i]) < 0:
                eigvec = -eigvec
            pointsnormals.append(eigvec)

    return pointsnormals

if __name__=='__main__':
    import os
    import pandaplotutils.pandactrl as pandactrl
    import pandaplotutils.pandageom as pg

    import CADTemp

    base = pandactrl.World(camp=[0,500,3000], lookatp=[0,0,0])

    this_dir, this_filename = os.path.split(__file__)
    objpath = os.path.join(this_dir, "models", "calibtable.stl")

    cadtemp = CADTemp.CADTemp(ompath=objpath, numpointsoververts=200)

    objnp = pg.packpandanp(cadtemp.objtrimesh.vertices,
                                cadtemp.objtrimesh.face_normals,
                                cadtemp.objtrimesh.faces,
                                name='')
    temppnts = cadtemp.pcdtemp
    normals = estimatenormals(temppnts)

    temppntsnp = pg.genPntsnp(temppnts)
    temppntsnp.reparentTo(base.render)
    for i, normal in enumerate(normals):
        pg.plotArrow(base.render, spos = temppnts[i], epos = normal+temppnts[i], thickness = 5, length = 100)

    base.run()