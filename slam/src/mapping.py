#! /usr/bin/env python

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import tf
import math

all_scans = []
all_tfs = []
curr_scan = None
curr_scan_world_tf = 0 
gmap = OccupancyGrid()
resolution = 1
width = 288
height = 288

car.x = 0.0
car.y = 0.0
rate = 5.0

grid = np.ndarray((width,height), buffer=np.zeros((width,height), dtype=np.int),dtype=np.int)
grid.fill(int(-1))

gmap.info.origin.position.x = -width//2*resolution
gmap.info.origin.position.y = -height//2*resolution
gmap.info.resolution = resolution
gmap.info.width = width
gmap.info.height = height
gmap.data = raneg(width*height)

while not rospy.is_shutdown():
    update_map()

    publish(gmap)
    loop_rate.sleep()

def update_map():
    trans = tf.transformations.euler_from_quaternion(orientation)

    #using scan and curr_scan_world_tf to change grid probs
    matrix = np.array([[,p.cos(euler[2]), np.sin(euler[2])],
        [-mp.sin(euler[2]), np.cos(euler[2])]])

    for i in range(360):
        wall = np.dot(matrix, np.array([0, (car_range + curr_scan[i]) // resolution])) + np.array([[off_x. off_y])

        #put on grid
        grid[int(obstacle[0]), int(obstacle[1])] = int(100)

        #put in grid
        for i in range(width*height):
            gmap.data[i] = grid.flat[i]
        #now publish
        talker(gmap)

def talker(map):
    publisher = rospy.Publisher('map', String, queue_size=10)
    rospy.init_node('map_node', anonymous=True)
    rate = rospy.Rate(25)
    while not rospy.is_shutdown():
        publisher.publish("hELLO")#the last elem of new scan
        rate.sleep()
        try:
            talker()
        except rospy.ROSInterruptException:
            pass

def subscriber():
    (tf,curr_scan) = rospy.Subscriber('icp', String, subscriber_icp)
    all_scans.append(scan)
    all_tfs.append(tf)
    curr_scan_world_tf += tf #total tf of scan till now
    rospy.spin()
