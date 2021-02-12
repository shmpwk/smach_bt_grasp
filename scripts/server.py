#! /usr/bin/env python

import rospy
import actionlib
import behavior_tree_core.msg

class BTAction(object):
  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, behavior_tree_core.msg.BTAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
    
  def execute_cb(self, goal):
    print("cb!")
    # helper variables
    r = rospy.Rate(1)
    rospy.loginfo('Action executed')
    print("goal:", goal)
    result = self._as.get_default_result()
    self._as.set_succeeded(result)

    # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
    r.sleep()

if __name__ == '__main__':
  rospy.init_node('grasp')
  BTAction(rospy.get_name())
  rospy.spin()
