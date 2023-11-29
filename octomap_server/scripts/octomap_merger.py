#!/usr/bin/env python

import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import argparse

class octomap_merger(object):
    def __init__(self,ns="uav1"):
        
        self.ns = ns
        self.color = np.asarray([0,1,0,1])
        self.color_other = np.asarray([1,0,0,1])
        if self.ns == "uav1":
            self.color = np.asarray([1,0,0,1])
            self.color_other = np.asarray([0,1,0,1])

        self.NAMESPACES = ["uav1"]
        self.NAMESPACES.remove(ns)
        self.transform = None
        self.pcl = None
        self.pcl2 = None
        self.pcl3 = None
        self.pcl_full = None
        
        pcl_TOPIC = "/"+ns+"/octomap_global_vis/octomap_point_cloud_centers"
        if len(self.NAMESPACES)>0:
            other_pcl_TOPIC = ["/"+ns1+"/octomap_global_vis/octomap_point_cloud_centers" for ns1 in self.NAMESPACES]
        pcl_local_TOPIC = "/"+ns+"/full_map"
        
        self.GLOBAL_ORIGIN = self.ns+"/world_origin"

        self.pcl_sub = rospy.Subscriber(pcl_TOPIC, PointCloud2, self.pclCallback, queue_size=5)
        if len(self.NAMESPACES)>0:
            self.pcl_sub = rospy.Subscriber(other_pcl_TOPIC[0], PointCloud2, self.pclCallback2, queue_size=5)
        # self.pcl_sub = rospy.Subscriber(other_pcl_TOPIC[1], PointCloud2, self.pclCallback3, queue_size=5)
        self.pub_pcl = rospy.Publisher(pcl_local_TOPIC, PointCloud2, queue_size=10)

    def pclCallback(self,pcl):
        if pcl is not None:
            self.pcl = self.point_cloud_converter_color(pcl,self.color)
            
    def pclCallback2(self,pcl):
        if pcl is not None:
            self.pcl2 = self.point_cloud_converter_color(pcl,self.color_other)

    def run(self):
        while not rospy.is_shutdown():
            if self.pcl is not None :
                self.pcl_full = ros_numpy.point_cloud2.pointcloud2_to_array(self.pcl)
                if self.pcl2 is not None and len(self.NAMESPACES)>0:
                    self.pcl_full =np.concatenate((self.pcl_full,ros_numpy.point_cloud2.pointcloud2_to_array(self.pcl2)))
                    rospy.loginfo_once(["Merging PCL from :",self.NAMESPACES[0]])

                # print(self.pcl_full)
                self.pub_pcl.publish(ros_numpy.point_cloud2.array_to_pointcloud2(self.pcl_full,stamp=rospy.Time.now(),frame_id=self.GLOBAL_ORIGIN))
                # rospy.loginfo("*****************Published*****************")
    
    def point_cloud_converter_color(self, pcl,color):
        """ Creates a point cloud message.
        Args:
            points: Nx4 array of xyz positions (m) and intensity
            parent_frame: frame in which the point cloud is defined
        Returns:
            sensor_msgs/PointCloud2 message
        """
        header = pcl.header
        width = pcl.width
        pcl = np.asarray(ros_numpy.point_cloud2.pointcloud2_to_array(pcl).tolist())        
        color = np.tile(color,(width,1))
        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize
        data = np.append(pcl,color,axis=1).astype(dtype).tobytes()
        fields = [PointField(
            name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyzrgba')]
        return PointCloud2(
            header=header,
            height=1,
            width=width,
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 7),
            row_step=(itemsize * 7 * pcl.shape[0]),
            data=data
        )



if __name__ == '__main__':
    rospy.init_node('octomap_merger', anonymous=True)
    parser = argparse.ArgumentParser(description='Example usage: python octomap_trimmer.py uav1')
    parser.add_argument('robot_name', type=str, help='pass robot name etc')
    args, unknown = parser.parse_known_args()
    obj = octomap_merger(args.robot_name)
    obj.run()