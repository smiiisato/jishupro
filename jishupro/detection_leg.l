#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::roseus "detect-leg")

(ros::advertise "/motor/command" std_msgs::Int64 1) ;;length
(ros::rate 5)

(setq *pose-number* 0)

;;pre-defined 1~20pose
(setq *pose1* (float-vector 0 0 0 0))
(setq *pose2* (float-vector 0 0 0 0))
(setq *pose3* (float-vector 0 0 0 0))
(setq *pose4* (float-vector 0 0 0 0))
(setq *pose5* (float-vector 0 0 0 0))
(setq *pose6* (float-vector 0 0 0 0))
(setq *pose7* (float-vector 0 0 0 0))
(setq *pose8* (float-vector 0 0 0 0))
(setq *pose9* (float-vector 0 0 0 0))
(setq *pose10* (float-vector 0 0 0 0))
(setq *pose11* (float-vector 0 0 0 0))
(setq *pose12* (float-vector 0 0 0 0))
(setq *pose13* (float-vector 0 0 0 0))
(setq *pose14* (float-vector 0 0 0 0))
(setq *pose15* (float-vector 0 0 0 0))
(setq *pose16* (float-vector 0 0 0 0))
(setq *pose17* (float-vector 0 0 0 0))
(setq *pose18* (float-vector 0 0 0 0))
(setq *pose19* (float-vector 0 0 0 0))
(setq *pose20* (float-vector 0 0 0 0))


(defun motor-command-cb (msg)
  (let (motor-command-msg (instance std_msgs::Int64 :inti))
    (setq orientation (send msg :orientation))
    ;;subscribe avg-msg
    (if (send msg :orientation)
	(progn
	  (setq x (send orientation :x))
	  (setq y (send orientation :y))
	  (setq z (send orientation :z))
	  (setq w (send orientation :w))

	  ;;print
	  (format t "avg-quaternion (~A, ~A, ~A, ~A)\n" x y z w)

	  ;;calculate error


	  ;;detect number of the state

	  (setq motor-command-msg :data *pose-number*)
	  ;;publish msg
	  (ros::publish "/motor/command" motor-command-msg)
	  )
      (progn
	(ros::ros-warn "")
	)
      )
    )
  )
	  


(ros::subscribe "/avg/quaternion" sensor_msgs::Imu #'motor-command-cb)

(warn "(main)~%")
(defun main()
  (do-until-key
   (ros::spin-once)
   (ros::sleep)
   ))

(main)
