#! /usr/bin/env python

import rospy
import actionlib
from behavior_tree_core.msg import *

if __name__=='__main__':
   
  rospy.init_node('client_test')
  client = actionlib.SimpleActionClient('grasp', BTAction)
  print("wait start") 
  client.wait_for_server()
  print("wait end") 
  
  goal = BTGoal()
  goal.parameter = 1
  client.send_goal(goal)
  client.wait_for_result(rospy.Duration.from_sec(5.0))
  
  result = client.get_result()
  print("result", result)
