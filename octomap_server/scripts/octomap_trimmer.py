#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs #import the packages first
from tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2
import argparse

class octomap_trimmer(object):
    def __init__(self,ns="uav1"):
        self.ns = ns
        self.transform = None
        self.pcl = None
        # pcl_TOPIC = "/"+ns+"/octomap_global_vis/octomap_point_cloud_centers"
        pcl_TOPIC = "/"+ns+"/full_map"
        pcl_local_TOPIC = "/"+ns+"/local_pcl"
        self.LOCAL_ORIGIN = self.ns+"/local_origin"
        self.GLOBAL_ORIGIN = self.ns+"/world_origin"
        self.pcl_sub = rospy.Subscriber(pcl_TOPIC, PointCloud2, self.pclCallback, queue_size=5)
        self.pub_pcl = rospy.Publisher(pcl_local_TOPIC, PointCloud2, queue_size=10)

    def pclCallback(self,pcl):
        if pcl is not None:
            self.pcl = pcl
    
    def get_transform(self):    
        tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        while True:
            try:
                self.transform = tf_buffer.lookup_transform(self.LOCAL_ORIGIN,self.GLOBAL_ORIGIN, #source frame
                                                rospy.Time(0), #get the tf at first available time
                                                rospy.Duration(1.0))
                break
            except:
                pass
         
    def run(self):
        while not rospy.is_shutdown():
            if self.pcl is not None and self.transform is not None:
                local_pcl = do_transform_cloud(self.pcl,self.transform )
                self.pub_pcl.publish(local_pcl)


if __name__ == '__main__':
    rospy.init_node('octomap_trimmer', anonymous=True)
    parser = argparse.ArgumentParser(description='Example usage: python octomap_trimmer.py uav1')
    parser.add_argument('robot_name', type=str, help='pass robot name etc')
    args, unknown = parser.parse_known_args()
    obj = octomap_trimmer(args.robot_name)
    obj.get_transform()
    print("got transform")
    obj.run()