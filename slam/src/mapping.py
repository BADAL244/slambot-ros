#! /usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from slam.msg import Custom
import tf
import math
import numpy as np

rospy.init_node('map_node', anonymous=True)

all_scans = []
all_tfs = []
scan = None
curr_scan_world_tf_x = 0
curr_scan_world_tf_y = 0
gmap = OccupancyGrid()
resolution = 1
width = 288
height = 288
count = 0

def update_map():
    count = count + 1
    if count < 2:
        return
    #Get curr total tf of car
    #Use total tf of car to get tf of each point on scan
    #Use scan tf to place on grid 
    trans = tf.transformations.euler_from_quaternion(orientation)

    #matrix to get scan point tf
    matrix = np.array([[np.cos(euler[2]), np.sin(euler[2])],
        [-mp.sin(euler[2]), np.cos(euler[2])]])

    #iterate over each scan point
    for i in range(360):
        wall = np.dot(matrix, np.array([0, (car_range + curr_scan[i]) // resolution])) + np.array([off_x, off_y])

        #put on grid
        grid[int(obstacle[0]), int(obstacle[1])] = int(100)

        #put in map
        for i in range(width*height):
            gmap.data[i] = grid.flat[i]
        return

def callback_sub(result):
    tf = result.pose
    scan = result.scan
    all_scans.append(scan)
    all_tfs.append(tf)
    rospy.loginfo("GETS HERE")
    curr_scan_world_tf += tf #total tf of scan till now

    update_map()
    gmap.header.stamp = rospy.Time.now()
    rospy.loginfo("I heard scan in MAP  %s",str(scan.ranges)[1:-1])
    last_scan_pub(scan)
    map_pub(gmap)

#Subscribers
icp_sub = rospy.Subscriber('icp', Custom, callback_sub)

#Publishers
map_pub = rospy.Publisher('map', OccupancyGrid, queue_size = 5)
last_scan_pub = rospy.Publisher('last_scan', LaserScan, queue_size = 5)



if __name__ == '__main_':
    grid = np.ndarray((width,height), buffer=np.zeros((width,height), dtype=np.int),dtype=np.int)
    grid.fill(int(-1))

    gmap.info.origin.position.x = -width//2*resolution
    gmap.info.origin.position.y = -height//2*resolution
    gmap.info.resolution = resolution
    gmap.info.width = width
    gmap.info.height = height
    gmap.data = range(width*height)

    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
