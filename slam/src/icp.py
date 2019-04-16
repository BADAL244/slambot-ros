#! /usr/bin/env python
import numpy as np
import rospy
import tf
import math
import turtlesim.msg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from slam.msg import Custom

rospy.init_node('icp_node')

gmap = {}
lidarScan = {}
B = None
T = None
A = None
count = 0

def ICP():
    global count, T, A, B
    rospy.loginfo("count %s",str(count))
    if count < 1:
        msg = Custom()
        msg.pose = Pose()
        msg.scan = B
        return msg

    return_scan = B
    #make matrices from scans

    src = []
    dst = []
    #TODO resolution vs lidar measurements
    for i in range(360):
        if not np.isinf(A.ranges[i]):
            x = np.cos(np.deg2rad(i))*A.ranges[i] + 90
            y = np.sin(np.deg2rad(i))*A.ranges[i] + 90
            src.append([x,y])
        else:
            src.append([10000,10000])
        if not np.isinf(B.ranges[i]):
            x1 = np.cos(np.deg2rad(i))*B.ranges[i] + 90
            y1 = np.sin(np.deg2rad(i))*B.ranges[i] + 90
            dst.append([x1,y1])
        else:
            dst.append([10000,10000])

    A_mat = np.matrix(src)
    B_mat = np.matrix(dst)

    #subtract the means of both matrices
    A_mat = A_mat - np.mean(A_mat, axis=0)
    B_mat = B_mat - np.mean(B_mat, axis=0)

    #convergence params
    iterations = 50
    rospy.loginfo(str(A_mat.shape)[1:-1])
    rospy.loginfo(str(B_mat.shape)[1:-1])
    #start iterations
    for i in range(iterations):
        T, residual, rank, sv = np.linalg.lstsq(A_mat, B_mat) #Ax-B minimize
        if np.allclose(A_mat, np.dot(B_mat, T), 0.001, 0.01, True):
            break
    rospy.loginfo("T shapre %s",str(T.shape)[1:-1])
    #return matrix and final transform
    #TODO make T into Pose
    msg = Custom()
    msg.scan = return_scan
    msg.pose = T
    return msg

def getPointInScan(lidarScan,i):
    pt = lidarScan[i]
    return math.sqrt(math.pow(pt.x, 2) + math.pow(pt.y,2))

def findClosestPoint(A,B,i):
    pt = getPointInScan(A,i);
    pt2 = [];
    lowestDist = 999999999;
    for i in range(360):
        testPoint = getPointInScan(B,i)
        dist = math.sqrt(math.pow(pt.x - testPoint.x, 2) + math.pow(pt.y - testPoint.y, 2))
        if dist < lowestDist:
            lowestDist = dist
            pt2 = testPoint
    return pt2

def subscriber_map(scan):
    global A, count
    count = count + 1
    A = scan
    if A is not None:
        rospy.loginfo("I heard scan A %s",str(A.ranges)[1:-1])
    else:
        rospy.loginfo("A is None")
    return

def subscriber_encoder(tf):
    global T
    T = tf
    return

def subscriber_rplidar(scan):
    global B
    B = scan
    #rospy.loginfo("I heard scan B %s",str(B.ranges)[1:-1])
    msg = Custom()
    msg = ICP()
    #rospy.loginfo("ScAN B %s",str(msg.scan.ranges)[1:-1])
    icp_pub.publish(msg)
    return

#Subscribers
map_sub = rospy.Subscriber('last_scan', LaserScan, subscriber_map)
enc_sub = rospy.Subscriber('encoder', Pose, subscriber_encoder)
lidar_sub = rospy.Subscriber('scan', LaserScan, subscriber_rplidar)

#Publisher
icp_pub = rospy.Publisher('icp', Custom, queue_size = 20)

if __name__ == '__main__':
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            pass
