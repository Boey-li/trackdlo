#!/usr/bin/env python3

import rospy
from ros_numpy import point_cloud2
from sensor_msgs.msg import PointCloud2

results = []

def callback(results_pc):
    global results

    pc_data = point_cloud2.pointcloud2_to_array(results_pc)
    results = point_cloud2.get_xyz_points(pc_data)
    print("================================")
    print(results)
    
    

        
if __name__ == "__main__":
    
    rospy.init_node('extract')
    results_pc_sub = rospy.Subscriber('/trackdlo/results_pc', PointCloud2, callback=callback)

    rospy.spin()