;; Auto-generated. Do not edit!


(when (boundp 'tricat_msgs::WPList)
  (if (not (find-package "TRICAT_MSGS"))
    (make-package "TRICAT_MSGS"))
  (shadow 'WPList (find-package "TRICAT_MSGS")))
(unless (find-package "TRICAT_MSGS::WPLIST")
  (make-package "TRICAT_MSGS::WPLIST"))

(in-package "ROS")
;;//! \htmlinclude WPList.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass tricat_msgs::WPList
  :super ros::object
  :slots (_header _WP_data ))

(defmethod tricat_msgs::WPList
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:WP_data __WP_data) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _WP_data __WP_data)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:WP_data
   (&rest __WP_data)
   (if (keywordp (car __WP_data))
       (send* _WP_data __WP_data)
     (progn
       (if __WP_data (setq _WP_data (car __WP_data)))
       _WP_data)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; tricat_msgs/WP[] _WP_data
    (apply #'+ (send-all _WP_data :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; tricat_msgs/WP[] _WP_data
     (write-long (length _WP_data) s)
     (dolist (elem _WP_data)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; tricat_msgs/WP[] _WP_data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _WP_data (let (r) (dotimes (i n) (push (instance tricat_msgs::WP :init) r)) r))
     (dolist (elem- _WP_data)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get tricat_msgs::WPList :md5sum-) "f6d33a8e2e98aba038773acbcf874e13")
(setf (get tricat_msgs::WPList :datatype-) "tricat_msgs/WPList")
(setf (get tricat_msgs::WPList :definition-)
      "std_msgs/Header header
tricat_msgs/WP[] WP_data
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
MSG: tricat_msgs/WP
std_msgs/Float64 x
std_msgs/Float64 y
std_msgs/String type
std_msgs/UInt16 num
std_msgs/UInt16 range
std_msgs/Bool arrive
================================================================================
MSG: std_msgs/Float64
float64 data
================================================================================
MSG: std_msgs/String
string data

================================================================================
MSG: std_msgs/UInt16
uint16 data

================================================================================
MSG: std_msgs/Bool
bool data
")



(provide :tricat_msgs/WPList "f6d33a8e2e98aba038773acbcf874e13")


