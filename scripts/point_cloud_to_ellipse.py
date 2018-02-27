#!/usr/bin/env python
'''
Pointcloud to ellipse library

@author Lukas Huber
@date 2018-02-21

'''

from math import sin, cos, pi
import numpy as np

import rospy
import geometry_msgs.msg # ROS message namespaces
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, Vector3Stamped
import tf
import sensor_msgs
import time

# DECLARE GLOBAL VARIBLES --- VERY UNCLEAN-CHANGE THIS!!!!!!!!!!!
#dt = 0.960
DT_ODOM = rospy.Duration.from_sec(-1.0)
DT_ROBO = rospy.Duration.from_sec(-0.5)
r_robot = 0.46

class PointCloudToEllipse():
    a_human = [0.4, 1.1] #[m]
    a_bin = [0.5,0.5] # [m]

    # define robot geometry
    r_robot = 0.46 # [m]

    # safety margin
    robot_sf = 1.2

    # Define workspace where obstacle is expected
    robot_frontLaserFrame = np.array([[   0,  0,-.6,-0.6],
                                      [-0.3,0.3,0.3,-0.3]])

    workspaceRegion = np.array([[-6.5,-8.,-11.5,-9.5],
                                [  -5, -3,   -9,  -9]])

    workspaceRegion_tf_baseLink = workspaceRegion

    def __init__(self):
        self.node = rospy.init_node('laser_subsampler', anonymous=True)
        self.a = self.a_bin
        
        self.obs = dict()
        self.obs['a'] = self.a
        self.obs['p'] =  []
        #self.obs[it_obs]['a_sf'] = self.robot_sf + self.r_robot

        self.obs['x0'] =  np.array([0,0])
        self.obs['th_r'] =  0
        self.t_transform = 0
        self.tf_listener = tf.TransformListener()
        
        self.indexPoints = []


    def run(self):
        self.sub = rospy.Subscriber("/front/scan",sensor_msgs.msg.LaserScan,self.callback)

        self.pubWorkspace = rospy.Publisher("/obstacle/workspace",geometry_msgs.msg.PolygonStamped,queue_size=10)

        self.pubEll = rospy.Publisher("/obstacle/ellipse",geometry_msgs.msg.PolygonStamped,queue_size=10)

        self.pubEll_interPol = rospy.Publisher("/obstacle/ellipseInterpol",geometry_msgs.msg.PolygonStamped,queue_size=10)


        self.pub_roboshadow = rospy.Publisher("/obstacle/robotShadow",geometry_msgs.msg.PolygonStamped,queue_size=10)
        
        self.has_transform = False


    def callback(self, data):
        # Publish the workspace
        workspacePoly = geometry_msgs.msg.PolygonStamped()
        workspacePoly.header = data.header
        workspacePoly.header.frame_id = 'front_laser'
        workspacePoly.polygon.points = []

        for ii in range(self.workspaceRegion.shape[1]):
            newPoint = geometry_msgs.msg.Point32() 
            newPoint.x = self.workspaceRegion_tf_baseLink[0][ii]
            newPoint.y = self.workspaceRegion_tf_baseLink[1][ii]
            newPoint.z = 0  
            workspacePoly.polygon.points.append(newPoint)
        # Publish Workspace
        self.pubWorkspace.publish(workspacePoly)


        # Publish the Robot Square
        roboPoly = geometry_msgs.msg.PolygonStamped()
        roboPoly.header = data.header
        roboPoly.header.frame_id = 'front_laser'
        roboPoly.polygon.points = []

        for ii in range(self.workspaceRegion.shape[1]):
            newPoint = geometry_msgs.msg.Point32() 
            newPoint.x = self.robot_frontLaserFrame[0][ii]
            newPoint.y = self.robot_frontLaserFrame[1][ii]
            newPoint.z = 0  
            roboPoly.polygon.points.append(newPoint)

        # Publish Workspace
        self.pub_roboshadow.publish(roboPoly)

        # Estimate Ellipse
        self.estimateEllipseFromLaserData(data)

        # Publish Ellipse Polygon
        self.pubEll.publish(self.returnPolygon(data))

    def estimateEllipseFromLaserData(self, data):
        meanPhi = data.angle_min + (self.indexPoints[-1]-self.indexPoints[0])/2*data.angle_increment
       
        # The half ellipse which the laser sees in cartesian space
        halfEllipseCart = np.zeros((2,len(self.indexPoints)))
        
        pc_it = 0
        for pp in self.indexPoints:
            ang = data.angle_min + data.angle_increment*pp - meanPhi
            halfEllipseCart = self.cylindric2cartesian(data.ranges[pp],ang)
            pc_it+=1
        
        # Publish the workspace
        workspacePoly = geometry_msgs.msg.PolygonStamped()
        workspacePoly.header = data.header
        workspacePoly.header.frame_id = 'front_laser'
        workspacePoly.polygon.points = []

            
    def returnPolygon(self,data):
        # Only take datapoints between -pi/+pi
        angMin = max(data.angle_min, -pi/2)
        angMax = min(data.angle_max, pi/2)

        #print(angMax)
        #print('ang_min', data.angle_min)
        #print('ang_max', data.angle_max)

        self.pointCloud = np.zeros((2, (angMax-angMin)/data.angle_increment))
        self.indexPoints = []
        #self.pointCloud = np.zeros((2,len(data.ranges)))
        
        pc_it = 0
        for pp in range(len(data.ranges)):
            ang = data.angle_min+data.angle_increment*pp # TODO - make neater..
            #print('ang', ang)
            if(ang<=angMax and ang>=angMin):
                self.pointCloud[:,pc_it] = self.cylindric2cartesian(data.ranges[pp],
                                                                 ang)
                self.indexPoints.append(pc_it)
                pc_it += 1
        
        # TODO: CHECK PROBLEM WITH SCANING RANGE!!!
        #print('pc_it',pc_it)
        #print("pCloud_len",self.pointCloud.shape)
        #print("lenIpoints", len(self.indexPoints))
        #print(self.pointCloud)

        # Move to front_laser frame
        self.tfMap2FrontLaser()

        # Only consider points which are inside a certain region
        self.checkIfInRegion(self.workspaceRegion_tf_baseLink)
        
        # Don't consider points which are below robot
        insideTheRegion = False # The point needs to be outside the robot region
        self.checkIfInRegion(self.robot_frontLaserFrame, insideTheRegion)

        # Ellipse 2
        obstacle = geometry_msgs.msg.PolygonStamped()
        obstacle.header = data.header
        obstacle.header.frame_id = 'front_laser'
        obstacle.polygon.points = []

        for pp in range(self.pointCloud.shape[1]):
            newPoint = geometry_msgs.msg.Point32()  
            newPoint.x = self.pointCloud[0,pp]
            newPoint.y = self.pointCloud[1,pp]
            newPoint.z = 0
 
            obstacle.polygon.points.append(newPoint)
        
        return obstacle


    def checkIfInRegion(self, region, insideConvexRegion=True):
        # Only consider points which are in the convex region
        #region = self.workspaceRegion_tf_baseLink # rename for easier syntax
        
        insideIndices = [] # Contains points to keep (inside convex region)

        N_init = self.pointCloud.shape[1]
        for pp in range(self.pointCloud.shape[1]):
            keepPoint = True
            for rr in range(self.region.shape[1]):
                
                # Check wheter obstacle lies on the left ->
                v = []
                if rr+1 < region.shape[1]:
                    v = region[:,rr+1] - region[:,rr]
                else:
                    v = region[:,0] - region[:,rr]

                v_hat = self.pointCloud[:,pp] - region[:,rr]
                
                #if ((insideConvexRegion and self.crossProduct2D(v,v_hat) < 0) or
                #(not(insideConvexRegion) and 
                #print('point cloud', self.pointCloud[:,pp])
                #print('v', v)
                #print('v_hat', v_hat)
                #print('crossP', self.crossProduct2D(v,v_hat))

                if (self.crossProduct2D(v,v_hat) < 0):
                    keepPoint = False
                    break

            if ((insideConvexRegion and keepPoint) or
                not(insideConvexRegion) and not(keepPoint)):

                insideIndices.append(pp)


        # Only keep points inside the convex region
        #print('pCloud shape', self.pointCloud.shape)
        #print('len iPoints', len(self.indexPoints) )

        self.pointCloud = self.pointCloud[:,insideIndices]
        self.indexPoints =  [self.indexPoints[i] for i in insideIndices]

        
        print('{}/{} in regions.'.format(self.pointCloud.shape[1],N_init))


    def crossProduct2D(self, v1,v2):
        return v1[0]*v2[1]-v1[1]*v2[0]


    def cylindric2cartesian(self, r, phi):
        return np.array(([r*cos(phi),r*sin(phi)]))


    def tfMap2FrontLaser(self):
        target = self.workspaceRegion

        #self.tf_listener = tf.TransformListener()
        ts = TransformStamped()
        self.region = np.zeros((target.shape))

        #try:
        if True:
            #self.tf_listener.waitForTransform("front_laser", "map",rospy.Time(0)-DT_ROBO, rospy.Duration(0.05));

            #### VERYYY UNCLEAAAN!!!! --- SOLVE TIME ISSUE -- no time should be added at the end...
            now  = rospy.Time.now()
            past = rospy.Time.now()-rospy.Duration.from_sec(0.3)

            start = 0
            start = time.clock()
            self.tf_listener.waitForTransformFull("/front_laser", past,
                                              "/map",now, "/odom",
                                              rospy.Duration.from_sec(3.0))
            self.t_transform = time.clock() - start
            (trans,rot) = self.tf_listener.lookupTransformFull("/front_laser", past,
                                                               "/map", now, "/odom")	
            ts.transform.translation.x = trans[0]
            ts.transform.translation.y = trans[1]
            ts.transform.translation.z = trans[2]
            ts.transform.rotation.x = rot[0]
            ts.transform.rotation.y = rot[1]
            ts.transform.rotation.z = rot[2]
            ts.transform.rotation.w = rot[3]

            # Create array
            region = np.zeros((target.shape))
            for pp in range(target.shape[1]):
		target_v3 = Vector3Stamped()
                target_v3.header.stamp = now
		target_v3.header.frame_id = "front_laser"
                target_v3.vector.x = target[0,pp]
                target_v3.vector.y = target[1,pp]
                target_v3.vector.z = 0

                target_transformed = tf2_geometry_msgs.do_transform_vector3(target_v3, ts)

                region[0,pp] = target_transformed.vector.x + trans[0]
                region[1,pp] = target_transformed.vector.y + trans[1]

            
            rospy.loginfo("TF front_laser to map finished. Waited for {} ms.".format(round(self.t_transform*3,3)) )

            self.workspaceRegion_tf_baseLink = region

        #except:
            #rospy.loginfo("No transform found.")

if __name__ == '__main__':
    clf = PointCloudToEllipse()
    clf.run()
    rospy.spin()
