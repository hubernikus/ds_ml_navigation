#!/usr/bin/env python
import rospy
import sensor_msgs.msg
import numpy as np



class ConvexLaserFilter:
	def __init__(self):
		self.node = rospy.init_node('laser_subsampler', anonymous=True)
		self.window_half_width = 30
	def run(self):
		self.sub = rospy.Subscriber("/front/scan",sensor_msgs.msg.LaserScan,self.callback)
		self.pub = rospy.Publisher("/front/convex_scan",sensor_msgs.msg.LaserScan,queue_size=10)
	def convexify_ranges_simple(self,ranges,d_min,d_max,angle_min,angle_increment):
		range_array = np.array(ranges)
		convex_ranges = []
		for i in np.arange(0,len(ranges)):
			if i < self.window_half_width*2:
				convex_ranges.append(np.min(range_array[0:i+self.window_half_width]))
			elif i >= len(ranges)-self.window_half_width*2:
				convex_ranges.append(np.min(range_array[i-self.window_half_width:len(ranges)]))
			else:
				convex_ranges.append(np.min(range_array[i-self.window_half_width:i+self.window_half_width]))
		return tuple(convex_ranges)
	def convexify_ranges(self,ranges,d_min,d_max,angle_min,angle_increment):
		convex_ranges = np.array(ranges)
		mask = np.mod(np.arange(len(ranges)),5)
		mask = np.where(mask == 0)
		convex_ranges = convex_ranges[mask]
		size = convex_ranges.size
		print(size)
		new_increment = angle_increment * 5
		new_max = angle_min + new_increment * size
		obstacle_stack = []
		found_obstacles = []
		#free_space_count = 0
		#Identify obtacles
		for i,d in enumerate(convex_ranges):
			if(i < 6):
				convex_ranges[i] = 10
				continue
			out_of_range = ( d < d_min or d > d_max)
			if out_of_range:
				#Hit free space
				if len(obstacle_stack) == 0:
					#Contiguous free space
					continue
				else:
					#End of obstacle
					#rospy.loginfo("APPEND %d",i)
					#free_space_count = free_space_count +1
					#if free_space_count > 3:
					found_obstacles.append(obstacle_stack)
					obstacle_stack = []
					#free_space_count=0
			else:
				#rospy.loginfo("HIT %d",i)
				#Hit obstacle
				while len(obstacle_stack) > 1 and obstacle_stack[-1][1] >= d:
					#Convexify
					#rospy.loginfo("POP %d",i)
					obstacle_stack.pop()
				#if len(obstacle_stack) > 1 && 
				#Add point to obstacle
				obstacle_stack.append((i,d))
		if len(obstacle_stack) > 1:
			found_obstacles.append(obstacle_stack)
		#Modfiy range to convex obstacles
		rospy.loginfo("obstacle count %d",len(found_obstacles))
		convex_zero = np.ones(len(convex_ranges))*20
		for o in found_obstacles:
			for i in np.arange(1,len(o)):
				lower_index = o[i-1][0]
				upper_index = o[i][0]
				x_l,y_l = self.polarToCart(o[i-1][1],lower_index*new_increment+angle_min)
				x_u,y_u = self.polarToCart(o[i][1],upper_index*new_increment+angle_min)
				index_distance = upper_index - lower_index
				#dist = np.sqrt((x_l-x_u)**2+(y_l-y_u)**2)
				#slope = dist/index_distance
				slope = (y_u-y_l)/(x_u-x_l)
				intercept = y_l - slope * x_l
				#diff = o[i-1][1]-o[i][1]
				#increment = diff/index_distance
				if index_distance >1 :
					#diff = o[i-1][1]-o[i][1]
					#increment = diff/index_distance
					#The obstacles is more than 2 rays wide
					for j in np.arange(lower_index+1,upper_index):
						theta = j*new_increment+angle_min
						x_intercept = -intercept/(slope - np.sin(theta)/np.cos(theta))
						y_intercept = slope*x_intercept+intercept
						convex_ranges[j] = np.sqrt(x_intercept**2+y_intercept**2)
						#print(np.sin(theta) - slope * np.cos(theta))
						#convex_ranges[j] =  convex_ranges[lower_index] - (j-lower_index)*increment
		#rospy.loginfo(np.sum(np.array(ranges)-np.array(convex_ranges)))
		return tuple(convex_ranges.tolist()),new_max,new_increment
	def polarToCart(self,r,theta):
		return r*np.cos(theta),r*np.sin(theta)
	def callback(self,data):
		d_min = data.range_min
		#rospy.loginfo("%f",d_min)
		d_max = data.range_max
		ranges =  data.ranges
		#convex_ranges = self.convexify_ranges(ranges,0.2,2,data.angle_min,data.angle_increment)
		#convex_ranges = self.convexify_ranges_simple(ranges,d_min,d_max,data.angle_min,data.angle_increment)
		convex_ranges,new_max,new_increment = self.convexify_ranges(ranges,d_min,5,data.angle_min,data.angle_increment)
		convex_scan = sensor_msgs.msg.LaserScan()
		convex_scan.header = data.header
		convex_scan.angle_min = data.angle_min
		convex_scan.angle_max = new_max
		convex_scan.angle_increment = new_increment
		convex_scan.time_increment = data.time_increment
		convex_scan.scan_time = data.scan_time
		convex_scan.range_min = data.range_min
		convex_scan.range_max = data.range_max
		#convex_scan.ranges = ranges
		convex_scan.ranges = convex_ranges
		self.pub.publish(convex_scan)	
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
    clf = ConvexLaserFilter()
    #clf.test()
    clf.run()
    rospy.spin()