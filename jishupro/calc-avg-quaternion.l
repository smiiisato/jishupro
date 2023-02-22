#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::roseus "avg-imu")

(ros::advertise "/avg/quaternion" sensor_msgs::Imu 10) ;;length
(ros::rate 10)

;;number of msg
(setq length 0)
;;sum of msg
(setq sum-x 0.0)
(setq sum-y 0.0)
(setq sum-z 0.0)
;;(setq sum-w 0.0)

;;set call-back function
(defun calc-avg-cb (msg)
  (let (x y z w avg-x avg-y avg-z avg-w
	   (avg-msg (instance sensor_msgs::Imu :init)))
    (send avg-msg :header :frame_id (send msg :header :frame_id))
    (send avg-msg :header :stamp (send msg :header :stamp))
    (setq orientation (send msg :orientation))
    (setq header (send msg :header))
    ;;subscribe imu msg
    (if (send msg :orientation)
	      (progn
		(setq x (send orientation :x))
		(setq y (send orientation :y))
		(setq z (send orientation :z))
		(setq w (send orientation :w))
		;;print
		(format t "quaternion (~A, ~A, ~A, ~A)\n" x y z w)
		(setq sum-x (+ sum-x x))
		(setq sum-y (+ sum-y y))
		(setq sum-z (+ sum-z z))
		(setq sum-w (+ sum-w w))
		(setq length (+ length 1))
		))
	(progn
	  (ros::ros-warn "no quaternion\n")
	  )
	(ros::ros-info (format nil "length is ~A" length))
    ;;calculate average
    (if (>= length 10)
	(progn
	  (setq avg-x (/ sum-x 10.0))
	  (setq avg-y (/ sum-y 10.0))
	  (setq avg-z (/ sum-z 10.0))
	  (setq avg-w (/ sum-w 10.0))
	  (setq length 0)
	  ;;initialize sum
	  (setq sum-x 0)
	  (setq sum-y 0)
	  (setq sum-z 0)
	  (setq sum-w 0)

	  (send avg-msg :orientation :x avg-x)
	  (send avg-msg :orientation :y avg-y)
	  (send avg-msg :orientation :z avg-z)
	  (send avg-msg :orientation :w avg-w)

	  (send avg-msg :linear_acceleration :x (send msg :linear_acceleration :x))
	  (send avg-msg :linear_acceleration :y (send msg :linear_acceleration :y))
	  (send avg-msg :linear_acceleration :z (send msg :linear_acceleration :z))

	  ;;publish
	  (ros::publish "/avg/quaternion" avg-msg)
	  )
      (progn
	(ros::ros-warn "")
	)
      )
    ))
;;subscribe msg
(ros::subscribe "/imu" sensor_msgs::Imu #'calc-avg-cb)

(warn "(main)~%")
(defun main()
  (do-until-key
   (ros::spin-once)
   (ros::sleep)
   ))

(main)

