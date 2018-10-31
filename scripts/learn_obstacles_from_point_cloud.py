#!/usr/bin/env python
'''
Pointcloud to obstacle

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
#from sensor_msgs import LaserScan, PointCloud2
import time

# MACHINE LEARNING LIBRARIES 
from sklearn.cluster import DBSCAN
from sklearn.svm import SVR

# DECLARE GLOBAL VARIBLES --- VERY UNCLEAN-CHANGE THIS!!!!!!!!!!!
#dt = 0.960
DT_ODOM = rospy.Duration.from_sec(-1.0)
DT_ROBO = rospy.Duration.from_sec(-0.5)
R_ROBOT = 0.46

class PointCloudToEllipse():
    def __init__(self):
        print('Note starting up ...')
        
        self.node = rospy.init_node('discrete_obstacle_learner', anonymous=True)

        self.await_backLaser = True
        self.await_frontLaser = True
        self.await_lidar = True

        # Data storage
        self.laser_front = [[] for ii in range(3)]
        self.laser_back = [[] for ii in range(3)]
        self.lidar = [[] for ii in range(3)]

        # Create subscribers
        self.sub_laserFront = rospy.Subscriber("/front/scan", sensor_msgs.msg.LaserScan, self.callback_laserFront)
        self.sub_laserFront = rospy.Subscriber('/rear/scan', sensor_msgs.msg.LaserScan, self.callback_laserBack)
        self.sub_laser = rospy.Subscriber('/velodyne_points', sensor_msgs.msg.PointCloud2, self.callback_lidar)

        # Create publishers
        self.pub_vel = rospy.Publisher('/obs', Vector3Stamped, queue_size=5)
        self.pub_points = rospy.Publisher('/pointcloud_reprod', sensor_msgs.msg.PointCloud2, queue_size=5)

        while self.await_backLaser or self.await_frontLaser or self.await_lidar:
            print('zzzz sleepyyyy')
            rospy.sleep(0.1)
        
        print('Entering loop...')

        while not rospy.is_shutdown():
            X_data = np.vstack((self.laser_front[0], 
                                self.laser_front[1], 
                                self.laser_front[2])).T

            db = DBSCAN(eps=0.5, min_samples=10).fit(X_data)
            
            n_clusters_ = len(set(db.labels_))

            print('data', X_data)
            print('N clust', n_clusters_)

            #print('Loop done')
            a=1

            

            rospy.sleep(0.5)

            #self.pub_points.publish()


    def run(self):
        return 0

    def callback_lidar(self, data):
        #print('dta', data.data)
        # N_points = len([data.ranges])
        # data_in_range = (np.array(data.ranges) > data.range_min)*(np.array(data.ranges) < data.range_max)
        self.await_lidar = False
        print('Got new LiDAR data')
        return 0

    def callback_laserFront(self, data):
        N_points = len(data.ranges)
        data_in_range = (np.array(data.ranges) > data.range_min)*(np.array(data.ranges) < data.range_max)
        
        # print('len rang', data_in_range.shape)
        # print('range', data_in_range)
        # print('len arang', np.arange(N_points).shape)
        # print('N_p', N_points)

        a = np.arange(N_points)[data_in_range]
        b = np.arange(N_points)[data_in_range]*data.angle_increment 
        c = np.array(data.ranges)[data_in_range]
        self.laser_front[0] = np.cos(np.arange(N_points)[data_in_range]*data.angle_increment + data.angle_min) * np.array(data.ranges)[data_in_range]
        self.laser_front[1] = np.sin(np.arange(N_points)[data_in_range]*data.angle_increment + data.angle_min) * np.array(data.ranges)[data_in_range]
        self.laser_front[2] = np.zeros(self.laser_front[0].shape[0]) # Laser scan on floor
        
        self.await_frontLaser = False

        print('Got new floor data')
        
        #return 0

    def callback_laserBack(self, data):
        self.await_backLaser = False
        print('Got new floor back data')
        return 0

    def callback_camera(self, data):        
	print('got new cam data')
        return 0


if __name__ == '__main__': # Main function
    try:
        clf = PointCloudToEllipse()
    except rospy.ROSInterruptException: pass


