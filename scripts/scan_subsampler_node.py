#!/usr/bin/env python
import rospy # Import ROS for python
import sensor_msgs.msg # ROS message namespaces
import geometry_msgs.msg # ROS message namespaces

import numpy as np
from math import pi

# Custom libaries
#from point_cloud_to_ellipse import *


class EllipseLaserFilter:
        # Dimensions
        dim = 2

        # Define x and y limits of space to scan in [m]
	#xLim = [-0.5, 0.5]
	#yLIm = [0, 10]

	# define human ellipse geometry
        # LARGER AXIS ALWAYS FIRST!!!!
	a_human = [0.4, 1.1] #[m]
        a_bin = [0.5,0.5] # [m]

	# define robot geometry
	r_robot = 0.46 # [m]

        # safety margin
        robot_sf = 1.2
        
        def __init__(self):
		self.node = rospy.init_node('laser_ellipse_creater', anonymous=True)
		self.window_half_width = 30
		self.obs = []
                self.dt = 0
                
                # Set up obstacles
                self.defineMovingObstacle()
                self.defineStaticBoundaries()
                
                # Remove... 
                self.firstCallback = True


	def run(self):
		self.sub = rospy.Subscriber("/front/scan",sensor_msgs.msg.LaserScan,self.callback)
		#self.pub = rospy.Publisher("/front/convex_scan",sensor_msgs.msg.LaserScan,queue_size=10)

		#self.pubEll = rospy.Publisher("/obstacle/ellipse_sf",geometry_msgs.msg.PolygonStamped,queue_size=10)
		self.pubWall1 = rospy.Publisher("/obstacle/wall1",geometry_msgs.msg.PolygonStamped,queue_size=10)
		self.pubWall1_sf = rospy.Publisher("/obstacle/wall1_sf",geometry_msgs.msg.PolygonStamped,queue_size=10)
		self.pubWall2 = rospy.Publisher("/obstacle/wall2",geometry_msgs.msg.PolygonStamped,queue_size=10)
		self.pubWall2_sf = rospy.Publisher("/obstacle/wall2_sf",geometry_msgs.msg.PolygonStamped,queue_size=10)
                
		self.DS_avoid = rospy.Publisher("/DS/obsAvoidance",geometry_msgs.msg.PolygonStamped,queue_size=10)

        def polarToCart(self,r,theta):
		#return x ,y
		return r*np.cos(theta),r*np.sin(theta)
		  
	def cartToPolar(self,x, y):
		# return r, phi
		return np.sqrt(x**2+y**2), np.atan2(x,y)

        def minDiraAng(ang1, ang2):
                # Check this
                dAng = ang1 - ang2
                if dAng > pi:
                        return dAng - 2*pi
                elif dAng < -pi:
                        return dAng + 2*pi
                else:
                        return dAng
        

        def defineStaticBoundaries(self):
                # Static ellipses to describe boundaries	 
                it_obs = 1
                
                self.obs.append(dict())
                #self.obs[it_obs]['a'] = [10,1]
                self.obs[it_obs]['a'] = [30,0.1]
                self.obs[it_obs]['p'] = [0]
                self.obs[it_obs]['x0'] = np.array(([-5.8,-5]))
                self.obs[it_obs]['a_sf'] = self.robot_sf*self.r_robot
                #self.obs[it_obs]['sf'] = [0*pi/180]
                self.obs[it_obs]['th_r'] = [54*pi/180]

                it_obs = 2
                self.obs.append(dict())
                self.obs[it_obs]['a'] = [30,0.1]
                self.obs[it_obs]['p'] =  [0]
                self.obs[it_obs]['x0'] = np.array([-9.8,-5])
                self.obs[it_obs]['a_sf'] = self.robot_sf*self.r_robot
                #self.obs[it_obs]['sf'] = [0*pi/180]
                self.obs[it_obs]['th_r'] = [57*pi/180]
                
                
        def defineMovingObstacle(self):
                # Human/obstacle moving in space
                it_obs = 0
                self.obs.append(dict())
                self.obs[it_obs]['a'] = [10,1]
		self.obs[it_obs]['p'] =  []
                self.obs[it_obs]['a_sf'] = self.robot_sf + self.r_robot
                
                self.obs[it_obs]['x0'] =  np.array([0,0])
                self.obs[it_obs]['th_r'] =  0
                #self.obs[it_obs]['x0'] =  self .posObs
                #self.obs[it_obs]['th_r'] =  self.orientationObs

        def setStaticEllipses(self,data):
                N_points = 50 # Resolution of boundary

                # Ellipse 1                
                ellipse1 = geometry_msgs.msg.PolygonStamped()
                ellipse1.header = data.header
                ellipse1.header.frame_id = 'map'
                
                obsNumb = 1 # number of first wall
                wall1 = self.ellipsePolygonCreater(data, N_points , obsNumb)
                ellipse1.polygon.points = wall1
                self.pubWall1.publish(ellipse1)

                #ellipse 1 with safety margin
                sfMargin = True
                ellipse1.polygon.points = self.ellipsePolygonCreater(data, N_points , obsNumb, True)
                self.pubWall1_sf.publish(ellipse1)
                
                # Ellipse 2
                ellipse2 = geometry_msgs.msg.PolygonStamped()
                ellipse2.header = data.header
                ellipse2.header.frame_id = 'map'

                obsNumb = 2 # number of first wall
                wall2 = self.ellipsePolygonCreater(data, N_points , obsNumb)
                ellipse2.polygon.points = wall2
                self.pubWall2.publish(ellipse2)

                # sfMargin = True
                ellipse2.polygon.points = self.ellipsePolygonCreater(data, N_points , obsNumb, True)
                self.pubWall2_sf.publish(ellipse2)


        def updateMovingObstacle(self):
                it_obs = 3

                # KALMAN FILTER???? --- For the moment simplyfied
                k_update_vel = 0.2
                k_update_pos = 0.2
                
                # Update postion and velocity
                self.obs[it_obs]['dx'] = ((1-k_update_vel)*self.obs[it_obs]['dx']
                                        + k_update_vel*(self.obs[it_obs]['x0'] -  self.posObs)/dt)
                                       
                integPos = self.obs[it_obs]['dt']*dt+self.obs[it_obs]['x0']
                self.obs[it_obs]['x0'] = ((1-k_update_pos)*self.obs[it_obs]['x0']
                                       + k_update_pos*integPos)

                # Update attitude and orientation
                self.obs[it_obs]['w'] = ((1-k_update_ve)*self.obs[it_obs]['w']
                                      + k_update_vel*minDiraAng(self.obs[it_obs]['th_r'],self.orientationObs)/dt)
                integAtt = self.obs[it_obs]['w']*dt+self.obs[it_obs]['th_r']
                self.obs[it_obs]['th_r'] = ((1-k_update_pos)*self.obs[it_obs]['th_r']
                                       + k_update_pos*integAtt)

                
        def identifyMovingObstacle(self):
                # minimum maximum
                # approx width
                # mean
                # closest point(s) 
                # estimate obstacle
                
                self.orienationObs = 0
                self.positionObs = np.array(([0,0]))
		  
        def ellipsePolygonCreater(self, data, N_points, it_obs, sfMargin=0):
                # create first ellipse
                obsCos = np.cos(self.obs[it_obs]['th_r'])
                obsSin = np.sin(self.obs[it_obs]['th_r'])

                R = [[obsCos, -obsSin],[obsSin, obsCos]]

                deltaPhi = 2*pi/N_points
                #a = self.obs[it_obs]['a']
                a = []
                if sfMargin:
                        #print('a',a)
                        #print('asf', self.obs[it_obs]['a_sf'])
                        for i in range(self.dim):
                                a.append(self.obs[it_obs]['a'][i] + self.obs[it_obs]['a_sf'])
                else:
                        a = self.obs[it_obs]['a']
                p = 1
                ellipseList = []

                # Create new point
                for ii in range(N_points):
                     newPoint = geometry_msgs.msg.Point32()  
                     Phi = ii*deltaPhi-pi
                     x0 = a[0]*np.cos(Phi)
                     y0 = a[1]*np.sin(Phi)

                     newPoint.x = self.obs[it_obs]['x0'][0]+ R[0][0]*x0 + R[0][1]*y0
                     newPoint.y = self.obs[it_obs]['x0'][1]+ R[1][0]*x0 + R[1][1]*y0                   
                     newPoint.z = 0
                     
                     ellipseList.append(newPoint)
                return ellipseList







        def callback(self,data):
                # Modfiy range to convex obstacles
                rospy.loginfo("Number of obstacles %d",3)
                print('New ellipse publish.')
                
                # Observe dangerous points
                if self.firstCallback == True:
                        self.setStaticEllipses(data)
                        #self.firstCallback = False
                        
                # Publish Dangerous points as Polygon
                #self.pubEll.publish(PointCloudToEllipses.returnPolygon() )
                

                


	def test_callback(self,data):
		  d_min = 0
		  d_max = 5

		  test_ranges = np.ones(500)*10
		  test_ranges[0:50] = np.arange(2,3,1/50.0)

		  test_ranges[75:100] = np.arange(3,2,-1/25.0)
		  test_ranges[100:125] = np.arange(2,3,1/25.0)

		  test_ranges[200:300] = np.ones(100)*2.5
		  test_ranges[225:275] = np.arange(1,1.5,0.5/50.0)

		  test_ranges[375:400] = np.arange(2,3,1/25.0)
		  test_ranges[400:425] = np.arange(3,2,-1/25.0)

		  #convex_ranges = self.convexify_ranges_simple(test_ranges,d_min,d_max,data.angle_min,data.angle_increment)
		  convex_ranges,new_max,new_increment = self.convexify_ranges(test_ranges,d_min,d_max,data.angle_min,data.angle_increment)
		  convex_scan = sensor_msgs.msg.LaserScan()
		  convex_scan.header = data.header
		  convex_scan.angle_min = -np.pi/2
		  convex_scan.angle_max = new_max
		  convex_scan.angle_increment = new_increment
		  convex_scan.time_increment = data.time_increment
		  convex_scan.scan_time = data.scan_time
		  convex_scan.range_min = 0
		  convex_scan.range_max = 5
		  convex_scan.ranges = convex_ranges

		  test_scan = sensor_msgs.msg.LaserScan()
		  test_scan.header = data.header
		  test_scan.angle_min = -np.pi/2
		  test_scan.angle_max = np.pi/2
		  test_scan.angle_increment = np.pi/500
		  test_scan.time_increment = data.time_increment
		  test_scan.scan_time = data.scan_time
		  test_scan.range_min = 0
		  test_scan.range_max = 5
		  test_scan.ranges = test_ranges

		  self.pub.publish(convex_scan)
		  self.pub_test.publish(test_scan)

	def test(self):
		  self.sub = rospy.Subscriber("/front/scan",sensor_msgs.msg.LaserScan,self.test_callback)
		  self.pub = rospy.Publisher("/front/convex_scan",sensor_msgs.msg.LaserScan,queue_size=10)
		  self.pub_test = rospy.Publisher("/front/test_scan_raw",sensor_msgs.msg.LaserScan,queue_size=10)


if __name__ == '__main__':
	 clf = EllipseLaserFilter()
	 clf.run()
	 rospy.spin()
