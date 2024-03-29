#!/usr/bin/env roseus
;; Author: Shumpei Wakabayashi <wakabayashi@jsk.imi.i.u-tokyo.ac.jp>
;; for all learning grasp system
(defvar *grasp-bbox* "/segmentation_decomposer/boxes")
(defvar *grasp-target-bounding-box-topic* "/bounding_box_marker/selected_box")
(defvar *grasp-status-topic* "/tabletop_object_grasp_status")
(defvar *base-frame-id* "/base_footprint")
(defvar *start-cond* 2)
(defvar *target-obj* nil)
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "jsk_rviz_plugins")
(ros::roseus-add-msgs "geometry_msgs")
(require "models/arrow-object.l")

(ros::roseus "pr2_tabletop_object_grasp_node")
(ros::rate 10)

(require :pr2-interface "package://pr2eus/pr2-interface.l")

(defun start-cond (&rest args)
  (setq *grasp-state* "start check!")               
  (publish-grasp-state)
  (publish-info "start check!")                   
  )

(defun pr2-pregrasp-pose (&rest args)
  (setq *grasp-state* "initilize!")               
  (publish-grasp-state)
  (publish-info "initilize!")                   
  (send *pr2* :angle-vector #f(299.865 55.4297 53.3196 102.662 -121.542 125.071 -87.0418 -48.2131 -32.0168 -20.2598 -67.6931 -45.3044 -72.9084 -96.2568 -100.018 4.1047 54.75))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  (setq *co-endeffector-vis* (arrow))
  (setq *co-ik-target* (arrow))
  (setq *pregrasp-point* (arrow))
  (setq *grasp-point* (arrow))
  (setq *box-arrow* (arrow))
  (setq *finish-grasped* 2)
  (objects (list *co-endeffector-vis* *co-ik-target* *pr2*))
  (setq *start-cond* 1)
  t)
(defun finish-grasped (&rest args)
  (print "finish grasped?")
  (print *finish-grasped*)          
  (return-from finish-grasped *finish-grasped*))
  

(defun publish-status (msg-type fmt)
  (let ((color
         (case msg-type
          ('error (instance std_msgs::ColorRGBA :init
                            :r 1.0 :g 0.0 :b 0.0 :a 1.0))
          ('warn (instance std_msgs::ColorRGBA :init
                           :r 1.0 :g 1.0 :b 0.0 :a 1.0))
          (t (instance std_msgs::ColorRGBA :init
                       :r (/ 157.0 255) :g (/ 204.0 255) :b (/ 224.0 255) :a 1.0)))))
    (unless (ros::get-topic-publisher *grasp-status-topic*)
      (ros::advertise *grasp-status-topic* jsk_rviz_plugins::OverlayText 5)
      (unix:sleep 1))

    
    (ros::publish *grasp-status-topic*
                  (instance jsk_rviz_plugins::OverlayText :init
                            :width 1920 :height 400
                            :left 0 :top 0
                            :line_width 10 :text_size 50
                            :fg_color color
                            :text fmt))))

(defun publish-info (&rest args)
  (let ((s (format nil (car args) (cdr args))))
    (ros::ros-info s)
    (publish-status 'info s)))
(defun publish-warn (&rest args)
  (let ((s (format nil (car args) (cdr args))))
    (ros::ros-warn s)
    (publish-status 'warn s)))
(defun publish-error (&rest args)
  (let ((s (format nil (car args) (cdr args))))
    (ros::ros-error s)
    (publish-status 'error s)))

(defun vector3-> (applier v)
  (funcall applier
         (* (send v :x) 1000.0)
         (* (send v :y) 1000.0)
         (* (send v :z) 1000.0)))

(defun bounding-box->cube (msg)
  (progn
    (let ((cds (ros::tf-pose->coords (send msg :pose)))
          (d (vector3-> 'list (send msg :dimensions)))
          (base->parent (send *tfl* :lookup-transform
                              *base-frame-id* (send msg :header :frame_id)
                              (ros::time 0))))
      (send *tfb* :send-transform
            (send (send base->parent :copy-worldcoords)
                  :transform cds)
            *base-frame-id* "target_object" (send msg :header :stamp))
      (print base->parent)
      (setq *d_xyz* d)
      (send (apply #'make-cube d)
            :transform
            (send (send base->parent :copy-worldcoords)
                  :transform cds)))
    )
  )

(defun moved-callback (msg)
   (unless msg
     (publish-info "target point is nil")
     (return-from moved-callback nil))
   (setq *target-obj* (bounding-box->cube (elt (send msg :boxes) 0)))
    (unless *target-obj*
      (publish-info "target point is nil")
      (return-from grasp-target-callback nil))
    (unless *grasping-object-p*
        (progn
          (setq *pre-target-obj* *target-obj*))
          (setq *buf* (float-vector 0 0 0))
      (setq *grasping-object-p* t))
    (setq delta-move (v- (send *target-obj* :worldpos)
                         (send *pre-target-obj* :worldpos)))
    (setq *buf* (v+ *buf* delta-move))  
    (print "delta-move")
    (print delta-move)
    (print "buf")
    (print (norm *buf*))
    
    (if (> (norm *buf*) 5)
      (progn
         (setq *moved-status* 1)
         (setq *buf* (float-vector 0 0 0)))
      (setq *moved-status* 2))
    (setq *pre-target-obj* *target-obj*)
)

(defun grasp-target-callback (&rest args)
  (setq msg (one-shot-subscribe *grasp-bbox*
                  jsk_recognition_msgs::BoundingBoxArray
                  :timeout 1000000))
  (print "clicked")
  (setq *target-obj* (bounding-box->cube (elt (send msg :boxes) 0)))

    (unless *target-obj*
      (publish-info "target point is nil")
      (return-from grasp-target-callback nil))
    (if *grasping-object-p*
        (progn
          (publish-warn "cancel grasping...")
          (send *ri* :stop-motion)
          (send *ri* :stop-grasp *rarm* :wait t)
          (setq *grasping-object-p* nil))
      (setq *grasping-object-p* t))
    (unless
	(progn
	  (send *pr2* *rarm* :inverse-kinematics
		(make-coords :pos (v+ (send *target-obj* :worldpos)
				       ;;(float-vector 0 (- (/ (car *d_xyz*) -2) 10) 30)
                                      (float-vector 0 -120 0))
			     ;;:rpy #f(1.54 1.37 -1.54))
			     :rpy #f(1.54 0 0))
		:rotation-axis :t))
      (publish-error "out of reach...")
      (return-from grasp-target-callback nil))

    (publish-info "openning gripper...")
  
    (setq *grasp-state* "gripper-opening")
    (publish-grasp-state)
    (send *ri* :stop-grasp *rarm* :wait t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (setq *grasp-state* "grasp-approaching")
    (publish-grasp-state)
    (publish-info "reaching...")
    ;;(send *pr2* *rarm* :move-end-pos #f(0 10 -24) :world)
    (send *pr2* *rarm* :move-end-pos #f(0 140 0) :world)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
 t 
)

(defun reaching-grip (&rest args)   
    (setq *grasp-state* "grasp-closing")
    (publish-grasp-state)
    (publish-info "grasping...")
    (send *ri* :start-grasp *rarm*)
    (send *ri* :wait-interpolation)
    (let ((grasp-result (send *ri* :start-grasp *rarm* )))
      (send *ri* :wait-interpolation)
      (print grasp-result)
      (unless (< 1 grasp-result)
        (publish-error "failed to grasp ~A" grasp-result)
        (send *ri* :stop-grasp *rarm* :wait t)
        (publish-info "back to pre grasp...")
        (return-from reaching-grip nil)))
   t
)
(defun check-grasp-cond (&rest args)
    (send *ri* :start-grasp *rarm*)
    (send *ri* :wait-interpolation)
    (let ((grasp-result (send *ri* :start-grasp *rarm*)))
      (send *ri* :wait-interpolation)
      (print grasp-result)
      (if (< 5 grasp-result)   ;;for gazebo (< 0.1 grasp-result)                          
        (progn                                                     
          (setq *grasp-state* "gripper width is greater than 0!")               
          (publish-grasp-state)
          (publish-info "gripper width is greater than 0!")                   
          (setq *check-grasp* 1)
          ;;(return-from grasp-target-callback t))
          )
        (progn                                          
          (setq *grasp-state* "gripper width is almost 0!")      
          (publish-grasp-state)
          (publish-info "gripper width is almost 0")          
          (setq *check-grasp* 2)
          ;;(return-from grasp-target-callback nil))
          )))
)
  
(defun pick (&rest args)
    (setq *grasp-state* "picking")
    (publish-grasp-state)
    (publish-info "picking up...")
    (send *pr2* *rarm* :move-end-pos #f(0 0 250) :world)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (setq *finish-grasped* 1)
;;    t
;;)


;;(defun postgrasp (&rest args)    
    (setq *grasp-state* "placing")
    (publish-grasp-state)
    (publish-info "placing ...")
    (send *pr2* *rarm* :move-end-pos #f(0 0 -230) :world)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (print "save judge")
    ;; open glipper
    (send *ri* :stop-grasp *rarm* :wait t)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (send *pr2* *rarm* :move-end-pos #f(0 0 100) :world)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (publish-info "pre grasp pose")
    (setq *grasp-state* "returning")
    (publish-grasp-state)
    ;;(pr2-pregrasp-pose)
    (publish-info "click tabletop object!")
    t
  )

(defun wait-for-grasp-target (&rest args)
  (ros::subscribe *grasp-bbox*
                  jsk_recognition_msgs::BoundingBoxArray #'moved-callback)
  (publish-info "click tabletop object!")
  (setq *moved-status* 2)
)

(defun publish-grasp-state ()
  (ros::publish "grasp_state" (instance std_msgs::String :init :data *grasp-state*))) ;change from "/grasp_state" to "grasp_state"
(setq *grasping-object-p* nil)
(setq *rarm* :rarm)
(setq *tfl* (instance ros::transform-listener :init))
(setq *tfb* (instance ros::transform-broadcaster :init))
(ros::advertise "/grasp_state" std_msgs::String 1)
(setq *grasp-state* "waiting")
(pr2-init)


;;(wait-for-grasp-target)
;(defun demo ()
;;  (pr2-pregrasp-pose)
;;  ;;(wait-for-grasp-target)
;;  (ros::spin))
;;(demo)


