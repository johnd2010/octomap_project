#!/usr/bin/env python

import rospy
import tf2_ros
import cv2 as cv
import tf2_geometry_msgs #import the packages first
from tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose,PoseStamped, PoseArray, Point
import argparse
import ros_numpy
import numpy as np
from copy import deepcopy

class occgrid_merger(object):
    def __init__(self,ns):
        self.namespaces = ns
        self.octree_processor_object = {}
        self.transform = {}
        self.occgrid_merged_TOPIC = ["/"+namespace+"/occgrid_merger/octree_projected_map_merged" for namespace in self.namespaces]        
        self.pose_vertices = ["/"+namespace+"/pose_vertices" for namespace in self.namespaces]
        self.pub_occ_merged = [rospy.Publisher(topic, OccupancyGrid, queue_size=10) for topic in self.occgrid_merged_TOPIC ]
        
        self.pub_pose_vertices = [rospy.Publisher(topic, PoseArray, queue_size=10) for topic in self.pose_vertices ]
        for namespace in self.namespaces:
            print(namespace)
            self.octree_processor_object[namespace]=octomap_processor(namespace)

    def create_pose(self,x, y, z):
        pose = Pose()
        pose.position = Point(x, y, z)
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
        return pose
    
    def merge_occupancy_grid(self):
        occ_grid_dict = {}

        vertices = np.zeros((len(self.namespaces),4))
        for iter,namespace in enumerate(self.namespaces):
            smooth_occ_grid = self.octree_processor_object[namespace].return_smooth_occ_grid()
            if smooth_occ_grid is not None:
                occ_grid_dict[namespace] = {}
                occ_grid_dict[namespace]["occ_map"]=smooth_occ_grid
                # P = PoseArray()
                # P.header=smooth_occ_grid.header
                origin_x = smooth_occ_grid.info.origin.position.x
                origin_y = smooth_occ_grid.info.origin.position.y
                resolution = smooth_occ_grid.info.resolution
                height = smooth_occ_grid.info.height * smooth_occ_grid.info.resolution 
                width = smooth_occ_grid.info.width * smooth_occ_grid.info.resolution 
                vertices[iter,:] = np.asarray([origin_x-width,origin_x,origin_y-height,origin_y])
                occ_grid_dict[namespace]["vertices"]=vertices[iter,:]
                
                # P.poses.append(self.create_pose(origin_x,origin_y,0))
                # P.poses.append(self.create_pose(origin_x-width,origin_y,0))
                # P.poses.append(self.create_pose(origin_x,origin_y-height,0))
                # P.poses.append(self.create_pose(origin_x-width,origin_y-height,0))
                # self.pub_pose_vertices[iter].publish(P)
            else:
                return
        x = [np.min(vertices[:,:2][:]),np.max(vertices[:,:2][:])]
        y = [np.min(vertices[:,2:][:]),np.max(vertices[:,2:][:])]
        h = int((y[1] - y[0])/resolution)
        w = int((x[1] - x[0])/resolution)
        cells = None
        for iter,namespace in enumerate(self.namespaces):
            if smooth_occ_grid is not None:                
                # P = PoseArray()
                # P.header=smooth_occ_grid.header
                # P.poses.append(self.create_pose(x[0],y[0],0))
                # P.poses.append(self.create_pose(x[0],y[1],0))
                # P.poses.append(self.create_pose(x[1],y[0],0))
                # P.poses.append(self.create_pose(x[1],y[1],0))
                # self.pub_pose_vertices[iter].publish(P)
                merged = deepcopy(occ_grid_dict[namespace]["occ_map"])
                merged.info.origin.position.x = x[1]
                merged.info.origin.position.y = y[1]
                merged.info.height = h
                merged.info.width = w
                merged.data = -1*np.ones((h*w),dtype=np.int8)    
                merged = self.get_origin_cell_map(merged,occ_grid_dict)

                self.pub_occ_merged[iter].publish(merged)   
    
    def get_origin_cell_map(self,merged,occ_grid_dict):
        for key in occ_grid_dict.keys():
            diff_x = int(( merged.info.origin.position.x - occ_grid_dict[key]["occ_map"].info.origin.position.x)/merged.info.resolution )
            diff_y = int((merged.info.origin.position.y - occ_grid_dict[key]["occ_map"].info.origin.position.y)/merged.info.resolution )
            reshaped_data = np.reshape(merged.data,(merged.info.height,merged.info.width))
            # print([merged.info.height,merged.info.width])
            # print(diff_x)
            # print(diff_y)
            reshaped_data[diff_y,diff_x]=0
            # reshaped_data[diff_y,diff_x]=0
            size = np.shape(reshaped_data[diff_y:diff_y+occ_grid_dict[key]["occ_map"].info.height,diff_x:diff_x+occ_grid_dict[key]["occ_map"].info.width])
            reshaped_data[diff_y:diff_y+occ_grid_dict[key]["occ_map"].info.height,diff_x:diff_x+occ_grid_dict[key]["occ_map"].info.width] = np.reshape(occ_grid_dict[key]["occ_map"].data,size)
            merged.data = np.asarray(reshaped_data.reshape(-1),dtype=np.int8)
        return merged
        
        
    
    def run(self):
        while not rospy.is_shutdown():
            for namespace in self.namespaces:
                self.octree_processor_object[namespace].publish_local_pcl()
                self.octree_processor_object[namespace].publish_local_occ_grid()
                # self.octree_processor_object[namespace].publish_local_occ_grid_global()
            self.merge_occupancy_grid()
            
            



class octomap_processor(object):
    def __init__(self,ns="uav1"):
        self.ns = ns
        self.transform = {}
        self.transform["global_2_local"] = None
        self.transform["local_2_global"] = None

        self.smooth_occ_grid = None
        self.smooth_local =  None
        self.posestamped = PoseStamped()
        self.pcl = None
        self.occgrid = None
        self.LOCAL_FRAME = self.ns+"/local_origin"
        self.GLOBAL_FRAME = self.ns+"/world_origin"
        kernel_size = 5
        self.kernel = np.ones((kernel_size,kernel_size),np.float32)/(kernel_size*kernel_size)
        self.get_transform()
        print("got transform")

        pcl_TOPIC = "/"+ns+"/octomap_global_vis/octomap_point_cloud_centers"
        pcl_local_TOPIC = "/"+ns+"/local_pcl"
        self.pcl_sub = rospy.Subscriber(pcl_TOPIC, PointCloud2, self.pclCallback, queue_size=5)
        self.pub_pcl = rospy.Publisher(pcl_local_TOPIC, PointCloud2, queue_size=10)
        # pcl_local_TOPIC_smooshed = "/"+ns+"/local_pcl_smooshed"

        occgrid_TOPIC = "/"+ns+"/octree_projected_map"
        occgrid_world_TOPIC = "/"+ns+"/octree_projected_map_smooth"
        self.occ_sub = rospy.Subscriber(occgrid_TOPIC, OccupancyGrid, self.occCallback, queue_size=5)
        self.pub_occ = rospy.Publisher(occgrid_world_TOPIC, OccupancyGrid, queue_size=10)
        # occgrid_merged_TOPIC_global = "/"+ns+"/occgrid_merger/octree_projected_map_merged_global" 
        # self.pub_occ_merged_global = rospy.Publisher(occgrid_merged_TOPIC_global , OccupancyGrid, queue_size=10) 

    def occCallback(self,occ):
        if occ is not None:            
            smooth_occ = occ
            image = np.reshape(np.asarray(smooth_occ.data,dtype=np.float32),(smooth_occ.info.height,smooth_occ.info.width))
            smooth_occ.data = np.asarray(cv.filter2D(cv.filter2D(image,-1,self.kernel),-1,self.kernel),dtype=np.int8).reshape(-1)
            smooth_occ.data = np.where(smooth_occ.data<0,-1,0)
            self.smooth_local = deepcopy(smooth_occ)  #local frame occ grid

            self.posestamped.pose = smooth_occ.info.origin
            self.posestamped.header = smooth_occ.header            
            world_origin = tf2_geometry_msgs.do_transform_pose(self.posestamped,self.transform["local_2_global"])
            smooth_occ.info.origin = world_origin.pose
            smooth_occ.header.frame_id = self.GLOBAL_FRAME
            self.smooth_occ_grid = deepcopy(smooth_occ) #global frame occ grid
    
    def pclCallback(self,pcl):
        if pcl is not None:
            self.pcl = pcl
    
    def get_transform(self):    
        tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        while True:
            try:
                self.transform["global_2_local"] = tf_buffer.lookup_transform(self.LOCAL_FRAME,self.GLOBAL_FRAME, #source frame
                                                rospy.Time(0), #get the tf at first available time
                                                rospy.Duration(1.0))
                self.transform["local_2_global"] = tf_buffer.lookup_transform(self.GLOBAL_FRAME,self.LOCAL_FRAME, #source frame
                                                rospy.Time(0), #get the tf at first available time
                                                rospy.Duration(1.0))
                break
            except:
                pass
    
    def return_smooth_occ_grid(self):
        return self.smooth_occ_grid
    
    def publish_local_pcl(self):
        if self.pcl is not None and self.transform["global_2_local"] is not None:
            local_pcl = do_transform_cloud(self.pcl,self.transform["global_2_local"] )
            self.pub_pcl.publish(local_pcl)
    
    def publish_local_occ_grid(self):
        if self.smooth_local is not None:
            self.pub_occ.publish(self.smooth_local)   
    
    # def publish_local_occ_grid_global(self):
    #     if self.return_smooth_occ_grid() is not None:
    #         self.pub_occ_merged_global.publish(self.smooth_occ_grid)  
    
    

if __name__ == '__main__':
    rospy.init_node('occgrid_merger', anonymous=True)
    parser = argparse.ArgumentParser(description='Example usage: python occgrid_merger.py uavnames')
    parser.add_argument('--nargs', nargs='+')
    uav_names = []
    for _, value in parser.parse_args()._get_kwargs():
        if value is not None:
            uav_names.append(value)
    obj = occgrid_merger(uav_names[0])
    obj.run()


