#! /usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from slam.msg import Custom
import tf
import math
import numpy as np

rospy.init_node('map_node')

all_scans = []
all_tfs = []
scan = None
tf = None
curr_scan_world_tf = Pose()
gmap = OccupancyGrid()
resolution = 1
width = 288
height = 288
count = 0

def update_map():
    global count
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
    global curr_scan_world_tf
    rospy.loginfo("Mapping got something from ICP")
    tf = result.pose
    scan = result.scan
    all_scans.append(scan)
    if tf is not None:
        all_tfs.append(tf)
        curr_scan_world_tf += tf #total tf of scan till now

    update_map()
    gmap.header.stamp = rospy.Time.now()
    last_scan_pub.publish(scan)
    map_pub.publish(gmap)
    return

#Subscribers
icp_sub = rospy.Subscriber('icp', Custom, callback_sub)

#Publishers
map_pub = rospy.Publisher('map', OccupancyGrid, queue_size = 5)
last_scan_pub = rospy.Publisher('last_scan', LaserScan, queue_size = 5)



if __name__ == '__main__':
    grid = np.ndarray((width,height), buffer=np.zeros((width,height), dtype=np.int),dtype=np.int)
    grid.fill(int(-1))

    gmap.info.origin.position.x = -width//2*resolution
    gmap.info.origin.position.y = -height//2*resolution
    gmap.info.resolution = resolution
    gmap.info.width = width
    gmap.info.height = height
    gmap.data = range(width*height)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            pass
