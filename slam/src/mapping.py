#! /usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
from slam.msg import Custom
import tf
import math
import numpy as np
import os
import requests
import json
import urllib2

rospy.init_node('map_node')
server = 'http://CHANGE_ME:3001'

all_scans = []
all_tfs = []
scan = None
tf = None
curr_scan_world_tf = Pose() # all values in inches
gmap = OccupancyGrid()
count = 0
height = 288
width = 288
resolution = 0.0254

def mapToServer():
    payload = {
        'data': gmap.data
    }
    to = server + '/data'
    r = requests.post(to, data=payload)
    return

def update_map():
    global scan, height, width, resolution, gmap, curr_scan_world_tf, tf
    global all_tfs, all_scans
    global count
    count = count + 1
    if count < 2:
        return

    for i in range(360):
        if np.isinf(scan.ranges[i]):
            continue
        theta = np.deg2rad(i)
        #get robot to scan point pose, in metres
        scan_robo_tf = Pose()
        scan_robo_tf.position = Point()
        scan_robo_tf.orientation = Quaternion()
        #convert to inches
        scan_robo_tf.position.x = scan.ranges[i]*np.cos(theta)*39.37
        scan_robo_tf.position.y = scan.ranges[i]*np.sin(theta)*39.37
        scan_robo_tf.position.z = 0
        scan_robo_tf.orientation.x = 0
        scan_robo_tf.orientation.y = 0
        scan_robo_tf.orientation.z = 0
        scan_robo_tf.orientation.w = 0

        #add car world pose and car scan pose to get scan world Pose
        total = Pose()
        total = addPose(scan_robo_tf, curr_scan_world_tf)
        #add to grid
        #xpoz and ypoz are in inches, add 144 inches to place in middle of grid
        xpoz = int(total.position.x) + 144
        ypoz = int(total.position.y) + 144

        if xpoz < 0 or ypoz < 0:
            rospy.loginfo("GRID POSITIOn MIN %s %s",str(xpoz),str(ypoz))
        elif xpoz >= 288 or ypoz >= 288:
            continue
        else:
            grid[xpoz, ypoz] = int(100)
            for i in range(5):
                for j in range(5):
                    grid[int(curr_scan_world_tf.position.x+i+144),
                        int(curr_scan_world_tf.position.y+j+144)] = int(0)
            #rospy.loginfo(curr_scan_world_tf.position.x)
            #rospy.loginfo(curr_scan_world_tf.position.y)
            #hallway_cells =find_bresenham_points(xpoz,ypoz,
             #       int(scan_robo_tf.position.x),int(scan_robo_tf.position.y))
            #for b_point in hallway_cells:
             #   grid[b_point[0], b_point[1]] = int(0)
    for i in range(width*height):
        gmap.data[i] = grid.flat[i]
    mapToServer()
    return

def find_bresenham_points(x1,y1,x2,y2):
    slope = (y2-y1)//(x2-x1)
    intercept = y2 - slope*x2
    result = []
    if x1>x2:
        x1,x2=x2,x1
    if y1>y2:
        y1,y2=y2,y1
    for x in range(x1,x2):
        for y in range(y1,y2):
            result.append([x,y])
    return result


def addPose(A,B):
    C = Pose()
    C.position = Point()
    C.orientation = Quaternion()
    C.position.x = A.position.x + B.position.x
    C.position.y = A.position.y + B.position.y
    C.position.z = A.position.z + B.position.z
    C.orientation.x = A.orientation.x + B.orientation.x
    C.orientation.y = A.orientation.y + B.orientation.y
    C.orientation.z = A.orientation.z + B.orientation.z
    C.orientation.w = A.orientation.w + B.orientation.w
    return C

def callback_sub(result):
    global curr_scan_world_tf,scan,tf,all_scans,all_tfs
    tf = result.pose
    scan = result.scan
    all_scans.append(scan)
    if tf is not None:
        all_tfs.append(tf)
        curr_scan_world_tf = addPose(curr_scan_world_tf, tf)
        #rospy.loginfo(tf.position.x)
        #rospy.loginfo(tf.position.y)
        #rospy.loginfo(curr_scan_world_tf.position.x)
        #rospy.loginfo(curr_scan_world_tf.position.y)
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
    rospy.loginfo("started mapping")
    grid = np.ndarray((width,height), buffer=np.zeros((width,height), dtype=np.int),dtype=np.int)
    grid.fill(int(-1))

    gmap.info.origin.position.x = 0
    gmap.info.origin.position.y = 0
    gmap.info.resolution = 0.0254
    gmap.info.width = 288
    gmap.info.height = 288
    gmap.data = range(width*height)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            pass
