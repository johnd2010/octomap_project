#!/usr/bin/env python

import rospy
import cv2 as cv
import ros_numpy
import time
import numpy as np
from nav_msgs.msg import OccupancyGrid
import argparse

class octomap_smoother(object):
    def __init__(self,ns="uav1"):
        rospy.init_node('octomap_smoother', anonymous=True)
        # self.rate = rospy.Rate(1)
        self.ns = ns
        self.occ_grid = None
        self.smooth_occ = None
        self.occ_TOPIC = "/"+ns+"/octree_projected_map"
        occ_TOPIC_pub = "/"+ns+"/octree_projected_map_smooth"
        kernel_size = 5
        self.kernel = np.ones((kernel_size,kernel_size),np.float32)/(kernel_size*kernel_size)
        self.pcl_sub = rospy.Subscriber(self.occ_TOPIC, OccupancyGrid, self.occCallback)
        self.pub_pcl = rospy.Publisher(occ_TOPIC_pub, OccupancyGrid,queue_size=5)

    def occCallback(self,occ):
        if occ is not None:            
            smooth_occ = occ
            image = np.reshape(np.asarray(smooth_occ.data,dtype=np.float32),(smooth_occ.info.height,smooth_occ.info.width))
            smooth_occ.data = np.asarray(cv.filter2D(cv.filter2D(image,-1,self.kernel),-1,self.kernel),dtype=np.int8).reshape(-1)
            smooth_occ.data = np.where(smooth_occ.data<0,-1,0)
            self.pub_pcl.publish(smooth_occ)    
    
    def return_occ_grid(self):
            return self.occ_grid
            
            
    def run(self):
        while not rospy.is_shutdown():
            pass            
            # smooth_occ = self.return_occ_grid()
            # if smooth_occ is not None:
            #     image = np.reshape(np.asarray(smooth_occ.data,dtype=np.float32),(smooth_occ.info.height,smooth_occ.info.width))
            #     smooth_occ.data = np.asarray(cv.filter2D(image,-1,self.kernel),dtype=np.int8).reshape(-1)
            #     smooth_occ.data = np.where(smooth_occ.data<0,-1,0)
            #     self.pub_pcl.publish(smooth_occ)    
            # self.rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Example usage: python octomap_smoother.py uav1')
    parser.add_argument('robot_name', type=str, help='pass robot name etc')
    args, unknown = parser.parse_known_args()
    obj = octomap_smoother(args.robot_name)
    obj.run()
