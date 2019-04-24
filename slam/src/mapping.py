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

rospy.init_node('map_node')
server = os.getenv('SERVER')

all_scans = []
all_tfs = []
scan = None
tf = None
curr_scan_world_tf = Pose()
gmap = OccupancyGrid()
count = 0
height = 144
width = 144
resolution = 1

def mapToServer():
    payload = {
        'data': gmap
    }
    to = server + '/data'
    r = requests.post(to, data=payload)
    print (r)
    return

def update_map():
    global scan, height, width, resolution, gmap, curr_scan_world_tf, tf
    global all_tfs, all_scans
    global count
    count = count + 1
    if count < 2:
        return

    rospy.loginfo("Scen is %s", str(scan.ranges)[1:-1])
    for i in range(360):
        if np.isinf(scan.ranges[i]):
            continue
        theta = np.deg2rad(i)
        #get robot to scan point pose
        scan_robo_tf = Pose()
        scan_robo_tf.position = Point()
        scan_robo_tf.orientation = Quaternion()
        scan_robo_tf.position.x = scan.ranges[i]*np.cos(theta)
        scan_robo_tf.position.y = scan.ranges[i]*np.sin(theta)
        scan_robo_tf.position.z = 0
        scan_robo_tf.orientation.x = 0
        scan_robo_tf.orientation.y = 0
        scan_robo_tf.orientation.z = 0
        scan_robo_tf.orientation.w = 0

        #add car world pose and car scan pose to get scan world Pose
        total = Pose()
        total = addPose(scan_robo_tf, curr_scan_world_tf)
        #add to grid
        #144 because we want to start in the middle of the grid
        rospy.loginfo("xpoz: %s, ypoz: %s", str(total.position.x*39.37),
                str(total.position.y*39.37))
        xpoz = int (total.position.x*39.37) + 72
        ypoz = int (total.position.y*39.37) + 72

        if xpoz < 0 or ypoz < 0:
            rospy.loginfo("GRID POSITION IS LESS THAN 0")
        elif xpoz >= 144 or ypoz >= 144:
            continue
        else:
            grid[xpoz, ypoz] = int(100)

            rospy.loginfo("In grid X: %s, Y: %s", str(int(xpoz)),str(int(ypoz)))
    for i in range(width*height):
        gmap.data[i] = grid.flat[i]
    mapToServer()
    return


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
    #rospy.loginfo("Mapping got something from ICP %s",str(result.scan.ranges)[1:-1])
    tf = result.pose
    rospy.loginfo("Mapping X: %s, Y: %s", str(tf.position.x),str(tf.position.y))
    scan = result.scan
    all_scans.append(scan)
    if tf is not None:
        all_tfs.append(tf)
        curr_scan_world_tf = addPose(curr_scan_world_tf, tf) #total tf of scan till now
        rospy.loginfo("Curr Scan World Tf x: %s, y:%s",str(curr_scan_world_tf.position.x),str(curr_scan_world_tf.position.y))
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
    gmap.info.width = 144
    gmap.info.height = 144
    gmap.data = range(width*height)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            pass
