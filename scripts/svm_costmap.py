#!/usr/bin/python
import rospy 
import std_msgs.msg
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import ChannelFloat32

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData

import numpy as np

from sklearn import svm
from sklearn.linear_model import LogisticRegression
import time

class KLRsk(object):
    def __init__(self,C = 5, gamma = 0.005):
        self.C = C
        self.gamma = gamma
        self.pred_norm_rdy= False
        self.model = LogisticRegression(C=C)

   def computeKernel(self,X):
        M1 = self.X.reshape((self.X.shape[0],self.X.shape[1],1))
        M1 = np.repeat(M1,X.shape[0],2)
        M2 = X.T.reshape((1,X.shape[1],X.shape[0]))
        kernel = np.exp(-1/self.gamma*(np.sum((M1-M2)**2,axis=1)))
        return kernel.T
        
    def fit(self,X,y):
        print("start fit")
        start = time.time()
        self.X = X
        self.y = y
        Xk = self.computeKernel(X)
        self.model.fit(Xk,y)
        self.alphas = np.array(self.model.coef_).T
        self.intercept = self.model.intercept_
        end = time.time()
        print("stop fit",end-start)
                
    def predict(self,X):
        to_pred = self.computeKernel(X)
        return self.model.predict(to_pred)

    def predict_proba(self,X):
        to_pred = self.computeKernel(X)
        return self.model.predict_proba(to_pred)[:,1]

    def computeGradientFX(self,x_raw):
        x = x_raw.reshape((1,-1))
        k = np.dot(self.computeKernel(x),self.alphas)
        num = np.exp(-1*(k+self.intercept))
        denom =  (1+np.exp(-1*(k+self.intercept)))**2
        return (num*self.f_prime(x,self.alphas,self.X)) /denom

    def f_prime(self,points,w,X):
        out = np.zeros((points.shape[0],2))
        for i in np.arange((points.shape[0])):
            d = w*(2.0/self.gamma)*(X - points[i,:])
            k = self.computeKernel(np.reshape(points[i,:],(1,2)))
            out[i] = np.dot(k,d);
        return out


class SVMCostMap:
    def __init__(self):
        self.node = rospy.init_node('laser_to_svm_subsampler', anonymous=True)
        self.counter = 0
        self.clf = svm.SVC(kernel="rbf",C=10,gamma=0.2)
        
    def run(self):
        self.sub = rospy.Subscriber("/front/convex_scan",LaserScan,self.callback)
        self.pub_free = rospy.Publisher("/front/free",PointCloud,queue_size=10)
        self.pub_full = rospy.Publisher("/front/full",PointCloud,queue_size=10)
        self.cm_grid = rospy.Publisher("/front/grid",OccupancyGrid,queue_size=10)
        
    def polarToCart(self,r,theta):
        return r*np.cos(theta),r*np.sin(theta)

    def callback(self,data):        
        boundary = 0.50
        sample_distance = 0.2
        angle_min = data.angle_min
        angle_increment = data.angle_increment
        base_scan = data.ranges
        pc_free = PointCloud()#(217*3)
        pc_full = PointCloud()#(217*3)
        pc = PointCloud()
        # pcfr = np.asarray(pc_free)
        # pcfu = np.asarray(pc_full)
        number_of_pixels = 217*3
        # create an empty list of correct size
        pc_free.points = [None] * number_of_pixels
        pc_full.points = [None] * number_of_pixels
        #pc.points = [None] * number_of_pixels
        pc.channels = ChannelFloat32()
        pc.channels.values = []
        pc.channels.name = "intensity"
        for i,r in enumerate(data.ranges):
            free = r - boundary
            full = r + boundary
            theta = angle_min + i * angle_increment
            for j in [0,1,2]:
                x,y = self.polarToCart(free-sample_distance*j,theta)
                pc_free.points[i*3+j] = Point32(x,y,0)
                # pc.points[i*6+j] = Point32(x,y,0)
                #pc.channels.values[i*6+j] = 100.0
                x,y = self.polarToCart(full+sample_distance*j,theta)
                pc_full.points[i*3+j] = Point32(x,y,0)
                # pc.points[i*6+j+3] = Point32(x,y,0)
            # pc.channels.values.append(100)
            # pc.channels.values.append(100)
            # pc.channels.values.append(100)
            # pc.channels.values.append(200)
            # pc.channels.values.append(200)
            # pc.channels.values.append(200)
        pc_free.header = std_msgs.msg.Header()
        pc_free.header.stamp = rospy.Time.now()
        pc_free.header.frame_id = "front_laser"
        pc_full.header = std_msgs.msg.Header()
        pc_full.header.stamp = rospy.Time.now()
        pc_full.header.frame_id = "front_laser"
        # print(pc.channels.values )
        # pc.header = std_msgs.msg.Header()
        # pc.header.stamp = rospy.Time.now()
        # pc.header.frame_id = "front_laser"
        self.pub_free.publish(pc_free)
        # self.pub_full.publish(pc)
        self.pub_full.publish(pc_full)
        #print(np.asarray(pc))
        if self.counter < 40:
            self.counter = self.counter +1
            return
        self.counter = 0
        self.build_cm(pc_free,pc_full)
    def build_cm(self,free,full):
        X = np.zeros((217*3*2,2))
        y = np.zeros((217*3*2,1))
        for i,p in enumerate(free.points):
            X[i,:] = [p.x,p.y]
            y[i] = -1
        for i,p in enumerate(full.points):
            X[i+217*3,:] = [p.x,p.y]
            y[i] = 1
        self.clf.fit(X,y)
        xx, yy = np.meshgrid(np.linspace(0, 5, 10), np.linspace(-5,5, 20))
        Z = self.clf.decision_function(np.c_[xx.ravel(), yy.ravel()])#,w,X,gamma)
        Z = Z.reshape(xx.shape).flatten()
        print(Z.shape)
        Z = Z-np.min(Z)
        Z = Z / np.max(Z)
        Z = np.ceil(Z*100).tolist()
        print(np.max(Z),np.min(Z))
        grid = OccupancyGrid()
        grid.header = std_msgs.msg.Header()
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = "front_laser"
        grid.info = MapMetaData()

        grid.data = Z
        self.cm_grid.publish(Z)
        
if __name__ == '__main__':
    scmvm = SVMCostMap()
    #clf.test()
    scmvm.run()
    rospy.spin()




