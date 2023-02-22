#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::roseus "avg-imu")

(ros::advertise "/avg/linear/acceleration" sensor_msgs::Imu 10) ;;length
(ros::rate 20)

;;number of msg
(setq length 0)
;;sum of msg
(setq sum-x 0.0)
(setq sum-y 0.0)
(setq sum-z 0.0)
;;number of sample
(setq sample-num 20.0)

;;set call-back function
(defun calc-avg-cb (msg)
  (let (x y z avg-x avg-y avg-z
	   (avg-msg (instance sensor_msgs::Imu :init)))
    (send avg-msg :header :frame_id (send msg :header :frame_id))
    (send avg-msg :header :stamp (send msg :header :stamp))
    (setq linear-acc (send msg :linear_acceleration))
    (setq header (send msg :header))
    ;;subscribe imu msg
    (if (send msg :linear_acceleration)
	      (progn
		(setq x (send linear-acc :x))
		(setq y (send linear-acc :y))
		(setq z (send linear-acc :z))
		;;print
	;;	(format t "quaternion (~A, ~A, ~A)\n" x y z)
		(setq sum-x (+ sum-x x))
		(setq sum-y (+ sum-y y))
		(setq sum-z (+ sum-z z))
		(setq length (+ length 1))
		))
	(progn
	  (ros::ros-warn "no acceleration")
	  )
	(ros::ros-info (format nil "length is ~A" length))
    ;;calculate average
    (if (>= length sample-num)
	(progn
	  (setq avg-x (/ sum-x sample-num))
	  (setq avg-y (/ sum-y sample-num))
	  (setq avg-z (/ sum-z sample-num))
	  (setq length 0)
	  ;;initialize sum
	  (setq sum-x 0)
	  (setq sum-y 0)
	  (setq sum-z 0)

	  (send avg-msg :linear_acceleration :x avg-x)
	  (send avg-msg :linear_acceleration :y avg-y)
	  (send avg-msg :linear_acceleration :z avg-z)

	  ;;publish
	  (ros::publish "/avg/linear/acceleration" avg-msg)
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

