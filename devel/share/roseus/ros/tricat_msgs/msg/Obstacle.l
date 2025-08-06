;; Auto-generated. Do not edit!


(when (boundp 'tricat_msgs::Obstacle)
  (if (not (find-package "TRICAT_MSGS"))
    (make-package "TRICAT_MSGS"))
  (shadow 'Obstacle (find-package "TRICAT_MSGS")))
(unless (find-package "TRICAT_MSGS::OBSTACLE")
  (make-package "TRICAT_MSGS::OBSTACLE"))

(in-package "ROS")
;;//! \htmlinclude Obstacle.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass tricat_msgs::Obstacle
  :super ros::object
  :slots (_begin _end ))

(defmethod tricat_msgs::Obstacle
  (:init
   (&key
    ((:begin __begin) (instance geometry_msgs::Point :init))
    ((:end __end) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _begin __begin)
   (setq _end __end)
   self)
  (:begin
   (&rest __begin)
   (if (keywordp (car __begin))
       (send* _begin __begin)
     (progn
       (if __begin (setq _begin (car __begin)))
       _begin)))
  (:end
   (&rest __end)
   (if (keywordp (car __end))
       (send* _end __end)
     (progn
       (if __end (setq _end (car __end)))
       _end)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _begin
    (send _begin :serialization-length)
    ;; geometry_msgs/Point _end
    (send _end :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _begin
       (send _begin :serialize s)
     ;; geometry_msgs/Point _end
       (send _end :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _begin
     (send _begin :deserialize buf ptr-) (incf ptr- (send _begin :serialization-length))
   ;; geometry_msgs/Point _end
     (send _end :deserialize buf ptr-) (incf ptr- (send _end :serialization-length))
   ;;
   self)
  )

(setf (get tricat_msgs::Obstacle :md5sum-) "41e958a799f3b38d26f0715b99f1cec2")
(setf (get tricat_msgs::Obstacle :datatype-) "tricat_msgs/Obstacle")
(setf (get tricat_msgs::Obstacle :definition-)
      "geometry_msgs/Point begin
geometry_msgs/Point end
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :tricat_msgs/Obstacle "41e958a799f3b38d26f0715b99f1cec2")


