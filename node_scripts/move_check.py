#!/usr/bin/env python

import rospy
import numpy as np
from jsk_recognition_msgs.msg import *
from std_msgs.msg import *

def moved_callback(msg):
  now_box = msg.boxes[0]
  global pre_box
  global arr
  if pre_box == 0:
    norm = 0
  else :
    xx = now_box.pose.position.x - pre_box.pose.position.x
    yy = now_box.pose.position.y - pre_box.pose.position.y
    vec = np.array((xx, yy))
    norm = np.linalg.norm(vec, ord=2)
  arr = np.delete(arr, 0)
  arr = np.append(arr, norm)
  move_norm = np.linalg.norm(arr, ord=2)
  print(move_norm)
  pre_box = now_box
  if move_norm > 0.01:
    pub.publish(1)
  else:
    pub.publish(2)

rospy.init_node("move_check")
pre_box = 0
arr = np.array((0))
sub = rospy.Subscriber("/segmentation_decomposer/boxes", BoundingBoxArray, moved_callback)
pub = rospy.Publisher("move_check", Int8, queue_size=1)
rospy.spin()
