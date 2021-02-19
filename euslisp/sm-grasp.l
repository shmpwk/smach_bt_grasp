#!/usr/bin/env roseus
(load "package://roseus_smach/src/state-machine.l")
(require "package://grasp_demo/euslisp/grasp_primitive.l")

(setq count 0)
(defun grasp-smach ()
  (let ((sm (instance state-machine :init)))
    (send sm :add-node (instance state :init :initialize 'pr2-pregrasp-pose))
    (send sm :add-node (instance state :init :recognize 'grasp-target-callback))
    (send sm :add-node (instance state :init :reach-grip 'reaching-grip))
    (send sm :add-node (instance state :init :pick 'pick))
    ;;(send sm :add-node (instance state :init :post-grasp 'postgrasp))
    ;; goal-states are generated in this method
    (send sm :goal-state (list :success :fail))


    ;; select a node as start-node
    (send sm :start-state :initialize)
    ;; from and to nodes are selected by name or symbol
    (send sm :add-transition :initialize :recognize t)
    (send sm :add-transition :recognize :reach-grip t)
    (send sm :add-transition :recognize :fail nil)
    ;;(send sm :add-transition :grasp-check :fail 2)
    ;;(send sm :add-transition :grasp-check :pick 1)
    (send sm :add-transition :reach-grip :pick t)
    (send sm :add-transition :pick :success t)
    (send sm :add-transition :pick :fail nil)
    sm ))

