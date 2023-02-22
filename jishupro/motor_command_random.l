#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::roseus "motor-command")

(ros::advertise "/motor/command01" std_msgs::Int64 1) ;;length
(ros::advertise "/motor/command02" std_msgs::Int64 1)
(ros::advertise "/motor/command03" std_msgs::Int64 1)
(ros::advertise "/motor/command04" std_msgs::Int64 1)
(ros::advertise "/motor/command07" std_msgs::Int64 1)
(ros::advertise "/motor/command08" std_msgs::Int64 1)
(ros::rate 20)


(defun motor-command-cb (msg)
  (let ((motor-command01-msg (instance std_msgs::Int64 :init))
	(motor-command02-msg (instance std_msgs::Int64 :init))
	(motor-command03-msg (instance std_msgs::Int64 :init))
	(motor-command04-msg (instance std_msgs::Int64 :init))
	(motor-command07-msg (instance std_msgs::Int64 :init))
	(motor-command08-msg (instance std_msgs::Int64 :init)))

    (cond ((= *pose-num* 0)
	   
	   (ros::publish "/avg/linear/acceleration" avg-msg)
	   (ros::publish "/avg/linear/acceleration" avg-msg)
	   (ros::publish "/avg/linear/acceleration" avg-msg)
	   (ros::publish "/avg/linear/acceleration" avg-msg)
	   ))


