'''
Pointcloud to ellipse library

@author Lukas Huber
@date 2018-02-21

'''

#
from math import sin, cos, pi
import numpy as np


class pointCloudToEllipse():
    def __init__(self, pointCloud, region, a_obstacle):
        
        self.pointCloud = np.zeros((pointCloud.shape))

        for pp in range(pointCloud.shape[1]):
            self.pointCloud[:,pp] = cylindric2cartesian(pointCloud[:,pp])
        
        self.region = region
        self.checkIfInRegion()

        self.a = a_obstacle
        
        self.obs.append(dict())
        self.obs[it_obs]['a'] = a
        self.obs[it_obs]['p'] =  []
        #self.obs[it_obs]['a_sf'] = self.robot_sf + self.r_robot

        self.obs[it_obs]['x0'] =  np.array([0,0])
        self.obs[it_obs]['th_r'] =  0

    def checkIfInRegion():
        insideIndices = []
        for pp in range(self.pointCloud.shape[1]):
            for rr in range(self.region.shape[1]):
                # Check wheter obstacle lies on the left ->
                if rr+1<range(self.region.shape[1]):
                    v = self.region[:,rr+1] - self.region[:,rr]
                else:
                    v = self.region[:,0] - self.region[:,rr]

                v_hat = self.pointCloud - self.region[:,rr]

                if(crossProduct2D(v,v_hat) > 0) :
                    insideInices.append(pp)
    
        # Only keep points inside the range
        self.pointCloud = self.pointCloud[:,insideIndices]

    def crossProduct2D(v1,v2):
        return v1[0]*v2[1]-v1[1]*v2[0]

    def cylindric2cartesian(r,phi):
        return np.array(([r*cos(phi),r*sin(phi)]))


