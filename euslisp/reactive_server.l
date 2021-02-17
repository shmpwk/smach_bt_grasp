#! /usr/bin/env roseus

(ros::load-ros-manifest "actionlib_msgs")
(ros::load-ros-manifest "behavior_tree_core")
(ros::roseus "grasp_server")
(require "package://grasp_demo/euslisp/grasp_primitive.l")

(defun start-cond-cb (server goal)
  (print "start check")
  (start-cond)                  
  (setq msg (send server :result :status *start-cond*))
  (send server :set-succeeded msg)
  (unix:sleep 1)
  (print "start check end")                   
)

(defun init-cb (server goal)
  (print "initilize")
  (pr2-pregrasp-pose)                   
  (setq input (send goal :goal :parameter))
  (setq fmsg (send server :feedback :status 1))
  (send server :publish-feedback fmsg)
  (setq msg (send server :result :status 1))
  (send server :set-succeeded msg)
  (unix:sleep 1)
  (print "pregrasp end")                   
)


(defun grasp-cb (server goal)
  (print "grasp start")
  (grasp-target-callback)                   
  (setq input (send goal :goal :parameter))
  (setq fmsg (send server :feedback :status 1))
  (send server :publish-feedback fmsg)
  (setq msg (send server :result :status 1))
  (send server :set-succeeded msg)
  (unix:sleep 1)
  (print "grasp end")                   
)

(defun check-grasp-cb (server goal)
  (print "check grasp")
  (check-grasp-cond)                   
  (setq input (send goal :goal :parameter))
  ;;(setq fmsg (send server :feedback :status 1))
  ;;(send server :publish-feedback fmsg)
  (setq msg (send server :result :status 1))
  (send server :set-succeeded msg)
)
(defun finish-grasped-cb (server goal) 
  (setq grasped (finish-grasped))
  (print grasped)
  (setq msg (send server :result :status grasped))
  (send server :set-succeeded msg)
)


(defun pick-cb (server goal)
  (print "pick")
  (pick)                   
  (setq input (send goal :goal :parameter))
  (setq fmsg (send server :feedback :status 1))
  (send server :publish-feedback fmsg)
  (setq msg (send server :result :status 1))
  (send server :set-succeeded msg)
  (unix:sleep 1)
  (print "pick end")
)

(defun post-cb (server goal)
  (print "place")
  (postgrasp)                   
  (setq input (send goal :goal :parameter))
  (setq fmsg (send server :feedback :status 1))
  (send server :publish-feedback fmsg)
  (setq msg (send server :result :status 1))
  (send server :set-succeeded msg)
  (unix:sleep 1)
)

(defun bt-start ()
  (setq *start* (instance ros::simple-action-server :init "started" behavior_tree_core::BTAction :execute-cb 'start-cond-cb))
  (print "started!!")                   
)

(defun bt-init ()
  (setq *init* (instance ros::simple-action-server :init "init" behavior_tree_core::BTAction :execute-cb 'init-cb))
  (print "initialize!!")                   
)

(defun bt-grasp ()
  (setq *grasp* (instance ros::simple-action-server :init "grasp" behavior_tree_core::BTAction :execute-cb 'grasp-cb))
  (print "exec grasp!!")                   
)
;;(defun bt-finish ()
;;  (setq *finish* (instance ros::simple-action-server :init "finish" behavior_tree_core::BTAction :execute-cb 'finish-cb))
;;  (print "finish!!")                   
;;)


(defun bt-check ()
  (setq *check* (instance ros::simple-action-server :init "check_grasp" behavior_tree_core::BTAction :execute-cb 'check-grasp-cb))
  (print "check grasp!!")                   
)

(defun bt-grasped ()
  (setq *grasped* (instance ros::simple-action-server :init "grasped" behavior_tree_core::BTAction :execute-cb 'finish-grasped-cb))
  (print "finish grasped?")                   
)

(defun bt-pick ()
  (setq *pick* (instance ros::simple-action-server :init "pick" behavior_tree_core::BTAction :execute-cb 'pick-cb))
  (print "pick!!")                   
)

(defun bt-post ()
  (setq *post* (instance ros::simple-action-server :init "place" behavior_tree_core::BTAction :execute-cb 'post-cb))
  (print "place!!")                   
)

(bt-start)
(bt-init)
(bt-grasp)
(bt-check)
(bt-grasped)
(bt-pick)
(bt-post)

(do-until-key
  (send *start* :worker)
  (send *init* :worker)
  (send *grasp* :worker)
  (send *check* :worker)
  (send *grasped* :worker)
  (send *pick* :worker)
  (send *post* :worker)
;;  (ros::spin)  
  (ros::spin-once)
  (ros::sleep)
)

