;; Auto-generated. Do not edit!


(when (boundp 'tricat_msgs::Sensor_total)
  (if (not (find-package "TRICAT_MSGS"))
    (make-package "TRICAT_MSGS"))
  (shadow 'Sensor_total (find-package "TRICAT_MSGS")))
(unless (find-package "TRICAT_MSGS::SENSOR_TOTAL")
  (make-package "TRICAT_MSGS::SENSOR_TOTAL"))

(in-package "ROS")
;;//! \htmlinclude Sensor_total.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass tricat_msgs::Sensor_total
  :super ros::object
  :slots (_header _position_ned _psi ))

(defmethod tricat_msgs::Sensor_total
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:position_ned __position_ned) (instance geometry_msgs::Point :init))
    ((:psi __psi) (instance std_msgs::Float64 :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _position_ned __position_ned)
   (setq _psi __psi)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:position_ned
   (&rest __position_ned)
   (if (keywordp (car __position_ned))
       (send* _position_ned __position_ned)
     (progn
       (if __position_ned (setq _position_ned (car __position_ned)))
       _position_ned)))
  (:psi
   (&rest __psi)
   (if (keywordp (car __psi))
       (send* _psi __psi)
     (progn
       (if __psi (setq _psi (car __psi)))
       _psi)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point _position_ned
    (send _position_ned :serialization-length)
    ;; std_msgs/Float64 _psi
    (send _psi :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point _position_ned
       (send _position_ned :serialize s)
     ;; std_msgs/Float64 _psi
       (send _psi :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Point _position_ned
     (send _position_ned :deserialize buf ptr-) (incf ptr- (send _position_ned :serialization-length))
   ;; std_msgs/Float64 _psi
     (send _psi :deserialize buf ptr-) (incf ptr- (send _psi :serialization-length))
   ;;
   self)
  )

(setf (get tricat_msgs::Sensor_total :md5sum-) "cfffd2feabde8d02a0b65cd795f0e07d")
(setf (get tricat_msgs::Sensor_total :datatype-) "tricat_msgs/Sensor_total")
(setf (get tricat_msgs::Sensor_total :definition-)
      "std_msgs/Header header
geometry_msgs/Point position_ned
std_msgs/Float64 psi

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
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: std_msgs/Float64
float64 data
")



(provide :tricat_msgs/Sensor_total "cfffd2feabde8d02a0b65cd795f0e07d")


