;; test-async-join-state-machine-actionlib.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "roseus_smach")
(ros::roseus "test_async_join_state_machine_actionlib")
(require :state-machine-actionlib "package://roseus_smach/src/state-machine-actionlib.l")
(require :state-machine-utils "package://roseus_smach/src/state-machine-utils.l")
(ros::rate 10)

(defun int->multiply10actiongoal (val)
  (instance roseus_smach::Multiply10ActionGoal :init
                  :goal :value val))


(defun make-async-join-state-machine ()
  (let ((sm (instance state-machine :init))
        (mul10-client (instance ros::simple-action-client :init
                                "multiply10" roseus_smach::Multiply10Action))
        random-int-value)
    (actionlib-client-state :multiply-10
     mul10-client :timeout 20 :retry t :key 'int->multiply10actiongoal)
               sm))
(make-async-join-state-machine)
(print "=================")
