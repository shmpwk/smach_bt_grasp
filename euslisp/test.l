#! /usr/bin/env roseus

(ros::load-ros-manifest "actionlib_msgs")
(ros::roseus-add-msgs "std_msgs")
(ros::roseus "grasp_server")

(defun main () 
  (setq msg (one-shot-subscribe "/move_check"
                  std_msgs::Int8
                  :timeout 1000000))
  (print "=====")
  (print msg)
  (setq move-status (send msg :data))
  (print "move sub")       
  (print move-status)
)
(do-until-key
  (main)
  (ros::spin)
)

