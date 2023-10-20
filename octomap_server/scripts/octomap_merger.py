#!/usr/bin/env python

import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import PointCloud2
import argparse

class octomap_merger(object):
    def __init__(self,ns="uav1"):
        
        self.ns = ns
        print(ns)
        self.NAMESPACES = ["uav29","uav30"]
        self.NAMESPACES.remove(ns)
        self.transform = None
        self.pcl = None
        self.pcl2 = None
        self.pcl3 = None
        self.pcl_full = None
        
        pcl_TOPIC = "/"+ns+"/octomap_global_vis/octomap_point_cloud_centers"
        other_pcl_TOPIC = ["/"+ns1+"/octomap_global_vis/octomap_point_cloud_centers" for ns1 in self.NAMESPACES]
        pcl_local_TOPIC = "/"+ns+"/full_map"
        
        self.GLOBAL_ORIGIN = self.ns+"/world_origin"

        self.pcl_sub = rospy.Subscriber(pcl_TOPIC, PointCloud2, self.pclCallback, queue_size=5)
        self.pcl_sub2 = rospy.Subscriber(other_pcl_TOPIC[0], PointCloud2, self.pclCallback2, queue_size=5)
        # self.pcl_sub = rospy.Subscriber(other_pcl_TOPIC[1], PointCloud2, self.pclCallback3, queue_size=5)
        self.pub_pcl = rospy.Publisher(pcl_local_TOPIC, PointCloud2, queue_size=10)

    def pclCallback(self,pcl):
        if pcl is not None:
            self.pcl = ros_numpy.point_cloud2.pointcloud2_to_array(pcl)
    def pclCallback2(self,pcl):
        if pcl is not None:
            self.pcl2 = ros_numpy.point_cloud2.pointcloud2_to_array(pcl)
    def pclCallback3(self,pcl):
        if pcl is not None:
            self.pcl3 = ros_numpy.point_cloud2.pointcloud2_to_array(pcl)

    def run(self):
        while not rospy.is_shutdown():
            if self.pcl is not None :
                self.pcl_full = self.pcl
                if self.pcl2 is not None:
                    self.pcl_full =np.concatenate((self.pcl_full,self.pcl2))
                    rospy.loginfo_once(["Merging PCL from :",self.NAMESPACES[0]])
                if self.pcl3 is not None:
                    self.pcl_full =np.concatenate((self.pcl_full,self.pcl3))
                    rospy.loginfo_once(["Merging PCL from :",self.NAMESPACES[1]])
                # print(self.pcl_full)
                self.pub_pcl.publish(ros_numpy.point_cloud2.array_to_pointcloud2(self.pcl_full,stamp=rospy.Time.now(),frame_id=self.GLOBAL_ORIGIN))
                rospy.loginfo("*****************Published*****************")

if __name__ == '__main__':
    rospy.init_node('octomap_merger', anonymous=True)
    parser = argparse.ArgumentParser(description='Example usage: python octomap_trimmer.py uav1')
    parser.add_argument('robot_name', type=str, help='pass robot name etc')
    args, unknown = parser.parse_known_args()
    obj = octomap_merger(args.robot_name)
    obj.run()
