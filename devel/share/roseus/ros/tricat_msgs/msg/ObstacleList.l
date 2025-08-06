;; Auto-generated. Do not edit!


(when (boundp 'tricat_msgs::ObstacleList)
  (if (not (find-package "TRICAT_MSGS"))
    (make-package "TRICAT_MSGS"))
  (shadow 'ObstacleList (find-package "TRICAT_MSGS")))
(unless (find-package "TRICAT_MSGS::OBSTACLELIST")
  (make-package "TRICAT_MSGS::OBSTACLELIST"))

(in-package "ROS")
;;//! \htmlinclude ObstacleList.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass tricat_msgs::ObstacleList
  :super ros::object
  :slots (_lidar_header _obstacle ))

(defmethod tricat_msgs::ObstacleList
  (:init
   (&key
    ((:lidar_header __lidar_header) (instance std_msgs::Header :init))
    ((:obstacle __obstacle) ())
    )
   (send-super :init)
   (setq _lidar_header __lidar_header)
   (setq _obstacle __obstacle)
   self)
  (:lidar_header
   (&rest __lidar_header)
   (if (keywordp (car __lidar_header))
       (send* _lidar_header __lidar_header)
     (progn
       (if __lidar_header (setq _lidar_header (car __lidar_header)))
       _lidar_header)))
  (:obstacle
   (&rest __obstacle)
   (if (keywordp (car __obstacle))
       (send* _obstacle __obstacle)
     (progn
       (if __obstacle (setq _obstacle (car __obstacle)))
       _obstacle)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _lidar_header
    (send _lidar_header :serialization-length)
    ;; tricat_msgs/Obstacle[] _obstacle
    (apply #'+ (send-all _obstacle :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _lidar_header
       (send _lidar_header :serialize s)
     ;; tricat_msgs/Obstacle[] _obstacle
     (write-long (length _obstacle) s)
     (dolist (elem _obstacle)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _lidar_header
     (send _lidar_header :deserialize buf ptr-) (incf ptr- (send _lidar_header :serialization-length))
   ;; tricat_msgs/Obstacle[] _obstacle
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obstacle (let (r) (dotimes (i n) (push (instance tricat_msgs::Obstacle :init) r)) r))
     (dolist (elem- _obstacle)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get tricat_msgs::ObstacleList :md5sum-) "e58b9c6e530a428f4aad1beec1ae1b08")
(setf (get tricat_msgs::ObstacleList :datatype-) "tricat_msgs/ObstacleList")
(setf (get tricat_msgs::ObstacleList :definition-)
      "std_msgs/Header lidar_header
tricat_msgs/Obstacle[] obstacle
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: tricat_msgs/Obstacle
geometry_msgs/Point begin
geometry_msgs/Point end
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :tricat_msgs/ObstacleList "e58b9c6e530a428f4aad1beec1ae1b08")


