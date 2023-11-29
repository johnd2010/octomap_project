#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid 
import numpy as np
import argparse
from visualization_msgs.msg import Marker

class octomap_area_calc(object):
    def __init__(self,ns="uav1"):
        self.ns = ns
        self.marker = Marker()
        self.occ =  True
        self.area_explored = None
        self.percentage_explored = None
        self.header = None
        self.occ_map_topic = "/"+ns+"/octree_projected_map"
        self.occ_map_area = "/"+ns+"/octree_projected_map_area"
        self.pcl_sub = rospy.Subscriber(self.occ_map_topic, OccupancyGrid, self.occCallback, queue_size=5)
        self.pub_occ_area = rospy.Publisher(self.occ_map_area, Marker, queue_size=10)
        
    
    def occCallback(self,occ_map_msg):
        if occ_map_msg is not None:            
            self.header = occ_map_msg.header            
            self.area_explored = occ_map_msg.info.width*occ_map_msg.info.height*occ_map_msg.info.resolution*occ_map_msg.info.resolution
            occ_map_msg = np.asarray(occ_map_msg.data)
            self.percentage_explored = (1 - (np.size(np.where(occ_map_msg==-1))/(np.size(occ_map_msg))))*100
    
    def generate_marker(self,marker_id=0,scale=1.5,xyzw=[0,0,0,1],rgba=[1,1,0,1],marker_type=Marker.TEXT_VIEW_FACING):
        
        self.marker.id = marker_id
        self.marker.type = marker_type
        self.marker.header = self.header
        self.marker.pose.orientation.x = 0  #Replace with arrows points away from shepherd
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1
        
        self.marker.scale.x = scale
        self.marker.scale.y = scale
        self.marker.scale.z = scale
        
        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 20
        self.marker.pose.position.z = 10
        
        self.marker.color.r = rgba[0]
        self.marker.color.g = rgba[1]
        self.marker.color.b = rgba[2]
        self.marker.color.a = rgba[3]
        self.marker.text = "UAV : "+self.ns + "\nAREA EXPLORED : " + f'{self.area_explored:.3f}' + "\nPERCENTAGE EXPLORED : " + f'{self.percentage_explored:.3f}'         
        
    
    def run(self):
        while not rospy.is_shutdown():
            if self.area_explored is not None and self.percentage_explored is not None:
                self.generate_marker()
                self.pub_occ_area.publish(self.marker)


if __name__ == '__main__':
    rospy.init_node('octomap_area_calc', anonymous=True)
    parser = argparse.ArgumentParser(description='Example usage: python octomap_area_calc.py uav1')
    parser.add_argument('robot_name', type=str, help='pass robot name etc')
    args, unknown = parser.parse_known_args()
    obj = octomap_area_calc(args.robot_name)
    obj.run()