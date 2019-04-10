#TODO ros nodes talking to each other
#TODO test icp
#TODO make mapping

#! /usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import String

map = {}
lidarScan = {}
B = None
T = None
A = None

def ICP(A,B,T=None):
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



def talker():
    publisher = rospy.Publisher('Transform', String, queue_size=10)
    rospy.init_node('icp', anonymous=True)
    rate = rospy.Rate(25)
    while not rospy.is_shutdown():
        subscriber()
        (tf,new_scan) = ICP(A,B,T)
        publisher.publish((tf,new_scan))
        rate.sleep()
        try:
            talker()
        except rospy.ROSInterruptException:
            pass

def subscriber_map(scan):
    B = scan
def subscriber_encoder(tf):
    T = tf
def subscriber_rplidar(data):
    A = data

def subscriber():
    rospy.Subscriber('rplidar', String, subscriber_rplidar)
    rospy.Subscriber('map', String, subscriber_map)
    rospy.Subscriber('encoder', String, subscriber_encoder)
    rospy.spin()

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

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
