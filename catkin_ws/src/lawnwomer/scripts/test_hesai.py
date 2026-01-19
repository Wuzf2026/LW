#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: Wuzf
@date: 2026.02.04
@brief Test Hesai JT128 LiDAR data reading
"""
import rospy
from sensor_msgs.msg import PointCloud2
from lawnwomer.msg import RTKData

LIDAR_IP = "192.168.1.201"
LIDAR_PORT = 2368

def hesai_test():
    rospy.init_node("hesai_test_node", anonymous=True)
    pub = rospy.Publisher("/pointcloud_data", PointCloud2, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # Simulate point cloud data (replace with actual SDK call)
        pcl_msg = PointCloud2()
        pcl_msg.header.stamp = rospy.Time.now()
        pcl_msg.header.frame_id = "base_link"
        pub.publish(pcl_msg)
        rospy.loginfo("Published point cloud data")
        rate.sleep()

if __name__ == "__main__":
    try:
        hesai_test()
    except rospy.ROSInterruptException:
        pass