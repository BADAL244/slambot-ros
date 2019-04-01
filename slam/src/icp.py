#TODO setup ros nodes

import numpy as np
import rospy
from std_msgs.msg import String

def getPointInScan(x):
    return 2

def FindClosestPointAndNormal(T,P,Q):
    return 2

def make_matrices(A,B):
    for i in range(1, 359):
        P = getPointInScan(i)
        (N, Q) = FindClosestPointAndNormal(A, B, i)

    return A, B

def ICP(A,B,T):
    #make matrices from scans
    A, B = make_matrices(A, B)

    #subtract the means of both matrices
    A = A - np.mean(A, axis=0)
    B = B - np.mean(B, axis=0)
    A = np.dot(T, A)

    #convergence params
    dmax = 0.001
    iterations = 50
    initial_distance = 0

    #start iterations
    for i in range(iterations):
        T, distance = np.linalg.lstsq(A, B)[0:2]
        A = np.dot(T, A)

        #check mean error
        mean_distance = np.mean(distance)
        if np.abs(initial_distance - mean_distance) < dmax:
            break
        initial_distance = mean_distance

    #return matrix and final transform
    return (T, A)



def talker():
    publisher = rospy.Publisher('mapping', String, queue_size=10)
    rospy.init_node('icp', anonymous=True)
    rate = rospy.Rate(25)
    while not rospy.is_shutdown():
        result = "hello world"
        publisher.publish(result)
        rate.sleep()
        try:
            talker()
        except rospy.ROSInterruptException:
            pass

def subscriber():
    rospy.init_node('icp')
    rospy.Subscriber('rplidar', String, ICP)
    rospy.spin()



import rospy
from std_msgs.msg import String

map = {{}}
lidarScan = {}

# TODO: where do you get map data from?

def getMapData():
    map = data.data
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    rospy.Subscriber("MapData", String, callback)
    lidarScan = rospy.Subscriber("RPLidar", String, callback)

def talker():
    pub = rospy.Publisher('xD', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        hello = "hi"
        rospy.loginfo(hello)
        pub.publish(hello)
        rate.sleep()

def getPointInScanA(i):
    pt = lidarScan[i]
    return Math.sqrt(Math.pow(pt.x, 2) + Math.pow(pt.y,2))

def findClosestPointInNormal(A,B,i):
    pt = A[i];
    pt2 = [];
    lowestDist = 999999999;
    for testPoint in B:
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
