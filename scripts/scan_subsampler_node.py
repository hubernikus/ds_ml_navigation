#!/usr/bin/env python
import rospy
import sensor_msgs.msg
import numpy as np


class ConvexLaserFilter:
	def __init__(self):
		self.node = rospy.init_node('laser_subsampler', anonymous=True)
	def run(self):
		self.sub = rospy.Subscriber("/front/scan",sensor_msgs.msg.LaserScan,self.callback)
		self.pub = rospy.Publisher("/front/convex_scan",sensor_msgs.msg.LaserScan,queue_size=10)
	def convexify_ranges(self,ranges,d_min,d_max,angle_min,angle_increment):
		convex_ranges = list(ranges)
		obstacle_stack = []
		found_obstacles = []
		#Identify obtacles
		for i,d in enumerate(convex_ranges):
			out_of_range = ( d < d_min or d > d_max)
			if out_of_range:
				#Hit free saace
				if len(obstacle_stack) == 0:
					#Contiguous free space
					continue
				else:
					#End of obstacle
					#rospy.loginfo("POP %d",i)
					found_obstacles.append(obstacle_stack)
					obstacle_stack = []
			else:
				#rospy.loginfo("HIT %d",i)
				#Hit obstacle
				while len(obstacle_stack) > 1 and obstacle_stack[-1][1] > d:
					#Convexify
					#rospy.loginfo("POP %d",i)
					obstacle_stack.pop()
				#Add point to obstacle
				obstacle_stack.append((i,d))
		found_obstacles.append(obstacle_stack)
		#Modfiy range to convex obstacles
		rospy.loginfo("obstacle count %d",len(found_obstacles))
		for o in found_obstacles:
			for i in np.arange(1,len(o)):
				lower_index = o[i-1][0]
				upper_index = o[i][0]
				x_l,y_l = self.polarToCart(o[i-1][1],lower_index*angle_increment+angle_min)
				x_u,y_u = self.polarToCart(o[i][1],upper_index*angle_increment+angle_min)
				index_distance = upper_index - lower_index
				dist = np.sqrt((x_l-x_u)**2+(y_l-y_u)**2)
				slope = dist/index_distance
				intercept = y_l - slope * x_l
				if index_distance >1 :
					#The obstacles is more than 2 rays wide
					for j in np.arange(1,index_distance):
						theta = (lower_index+j)*angle_increment+angle_min
						convex_ranges[j+lower_index] = intercept/(np.sin(theta) - slope * np.cos(theta)) 
		rospy.loginfo(np.sum(np.array(ranges)-np.array(convex_ranges)))
		return tuple(convex_ranges)
	def polarToCart(self,r,theta):
		return r*np.cos(theta),r*np.sin(theta)
	def callback(self,data):
		d_min = data.range_min
		rospy.loginfo("%f",d_min)
		d_max = data.range_max
		ranges =  data.ranges
		convex_ranges = self.convexify_ranges(ranges,d_min,d_max,data.angle_min,data.angle_increment)
		convex_scan = sensor_msgs.msg.LaserScan()
		convex_scan.header = data.header
		convex_scan.angle_min = data.angle_min
		convex_scan.angle_max = data.angle_max
		convex_scan.angle_increment = data.angle_increment
		convex_scan.time_increment = data.time_increment
		convex_scan.scan_time = data.scan_time
		convex_scan.range_min = data.range_min
		convex_scan.range_max = data.range_max
		convex_scan.ranges = convex_ranges
		self.pub.publish(convex_scan)	

if __name__ == '__main__':
    clf = ConvexLaserFilter()
    clf.run()
    rospy.spin()