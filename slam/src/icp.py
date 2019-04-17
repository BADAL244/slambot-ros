#! /usr/bin/env python
import numpy as np
from sklearn.neighbors import NearestNeighbors
import rospy
import tf
import math
import turtlesim.msg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from slam.msg import Custom

rospy.init_node('icp_node')

gmap = {}
lidarScan = {}
B = None
T = None
A = None
count = 0

def transform(A,B):
    m = A.shape[1]
    centre_A = np.mean(A, axis=0)
    centre_B = np.mean(B, axis=0)

    AA = A - centre_A
    BB = B - centre_B

    #rotattion
    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    #translation
    t = centre_B.T - np.dot(R, centre_A.T)

    #transformation
    T = np.identity(m+1)
    T[:m, :m] = R
    T[:m, m] = t

    return T,R,t

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

    init_a = []
    init_b = []
    #TODO resolution vs lidar measurements
    for i in range(360):
        if not np.isinf(A.ranges[i]):
            x = np.cos(np.deg2rad(i))*A.ranges[i] + 90
            y = np.sin(np.deg2rad(i))*A.ranges[i] + 90
            init_a.append([x,y])
        else:
            init_a.append([10000,10000])
        if not np.isinf(B.ranges[i]):
            x1 = np.cos(np.deg2rad(i))*B.ranges[i] + 90
            y1 = np.sin(np.deg2rad(i))*B.ranges[i] + 90
            init_b.append([x1,y1])
        else:
            init_b.append([10000,10000])

    B_as_ndarray = np.asarray(init_b)
    A_as_ndarray = np.asarray(init_a)

    m = A_as_ndarray.shape[1] #should be 2

    src = np.ones((m+1, 360))
    dst = np.ones((m+1, 360))
    src[:m,:] = np.copy(A_as_ndarray.T)
    dst[:m,:] = np.copy(B_as_ndarray.T)

    prev_error = 0
    #start iterations
    for i in range(50):

        #find nearest neighbour
        machine = NearestNeighbors(n_neighbors=1)
        machine.fit(dst[:m,:].T)
        res1, res2 = machine.kneighbors(src[:m,:].T,return_distance=True)
        distances = res1.ravel()
        indices = res2.ravel()

        T,R,t = transform(src[:m,:].T, dst[:m,indices].T)
        src = np.dot(T, src)

        mean_error = np.mean(distances)
        if np.abs(prev_error - mean_error) < 0.001:
            break
        prev_error = mean_error

    T,R,t = transform(A_as_ndarray, src[:m,:].T)
    rospy.loginfo("This is T %s",str(T)[1:-1])
    rospy.loginfo("This is R %s",str(R)[1:-1])
    rospy.loginfo("This is t %s",str(t)[1:-1])
    #return matrix and final transform
    #TODO make T into Pose
    msg = Custom()
    msg.scan = return_scan
    msg.pose = Pose()
    msg.pose.position = Point()
    msg.pose.orientation = Quaternion()
    msg.pose.position.x = t[0]
    msg.pose.position.y = t[1]
    msg.pose.position.z = 0
    msg.pose.orientation.x = R[0][0]
    msg.pose.orientation.y = R[0][1]
    msg.pose.orientation.z = R[1][0]
    msg.pose.orientation.w = R[1][1]

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
    #if A is not None:
        #rospy.loginfo("I heard scan A %s",str(A.ranges)[1:-1])
    #else:
        #rospy.loginfo("A is None")
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
