#!/usr/bin/env python
# coding:utf-8
from detect import get_maze_map_pose
import rospy
from std_msgs.msg import String
import cv2
from itertools import permutations


rospy.init_node('treasure_position_publisher')
pub = rospy.Publisher('treasure_location', String, queue_size=1)
cap = cv2.VideoCapture(0)
print("webcam found!")
ret, frame = cap.read()
treasure_pos = [(3,3)]
rate = rospy.Rate(1)
#stage one: Waiting until we got the map scanned to locate the treasures
while not rospy.is_shutdown():
    ret, frame = cap.read()
    if ret:
        location, img = get_maze_map_pose(frame,display=True)
        location = set(location)
        if len(location) == 8:
            treasure_pos = location
            break
        else:
            print("Could not acess webcam!")

treasure_str = '[' + ','.join([str(pos) for pos in treasure_pos]) + ']'
pub.publish(treasure_str)
rospy.signal_shutdown("Treasure recognition completed and data published.")