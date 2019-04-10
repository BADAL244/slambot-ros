#! /usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid 
from std_msgs.msg import String

rospy.init_node('icp_node', anonymous=True)

gmap = {}
lidarScan = {}
B = None
T = None
A = None

def ICP(A=None,B=None,T=None):
    #make matrices from scans

    src = []
    dst = []
    for i in range(360):
        pt = getPointInScan(A,i)
        src = src.append([pt.x, pt.y, 1])
        pt2 = findClosestPoint(A,B,i)
        dst = dst.append([pt2.x, pt2.y, 1])

    A = np.transpose(np.matrix(src))
    B = np.transpose(np.matrix(dst))

    #subtract the means of both matrices
    A = A - np.mean(A, axis=0)
    B = B - np.mean(B, axis=0)
    if T is not None:
        A = np.dot(T, A)

    #convergence params
    dmax = 0.001
    iterations = 50
    initial_distance = 0

    #start iterations
    for i in range(iterations):
        T, distance = np.linalg.lstsq(A, B)[0:2] #Ax-B minimize
        A = np.dot(T, A)

        #check mean error
        mean_distance = np.mean(distance)
        if np.abs(initial_distance - mean_distance) < dmax:
            break
        initial_distance = mean_distance

    #return matrix and final transform
    return (T, A)

def getPointInScan(lidarScan,i):
    pt = lidarScan[i]
    return Math.sqrt(Math.pow(pt.x, 2) + Math.pow(pt.y,2))

def findClosestPoint(A,B,i):
    pt = getPointInScan(A,i);
    pt2 = [];
    lowestDist = 999999999;
    for i in range(360):
        testPoint = getPointInScan(B,i)
        dist = Math.sqrt(Math.pow(pt.x - testPoint.x, 2) + Math.pow(pt.y - testPoint.y, 2))
        if dist < lowestDist:
            lowestDist = dist
            pt2 = testPoint
    return pt2

def subscriber_map(scan):
    global B
    B = scan
def subscriber_encoder(tf):
    global T
    T = tf
def subscriber_rplidar(scan):
    global A
    A = scan

#Subscribers
lidar_sub = rospy.Subscriber('rplidar', LaserScan, subscriber_rplidar)
map_sub = rospy.Subscriber('last_scan', LaserScan, subscriber_map)
enc_sub = rospy.Subscriber('encoder', tf, subscriber_encoder)

#Publisher
icp_pub = rospy.Publisher('icp', {tf, LaserScan}, queue_size = 5)

if __name__ == '__main__':

    while not rospy.is_shutdown():
        try:
            icp_pub.publish(ICP())
        except rospy.ROSInterruptException:
            pass
