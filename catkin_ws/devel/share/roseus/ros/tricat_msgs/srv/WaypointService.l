;; Auto-generated. Do not edit!


(when (boundp 'tricat_msgs::WaypointService)
  (if (not (find-package "TRICAT_MSGS"))
    (make-package "TRICAT_MSGS"))
  (shadow 'WaypointService (find-package "TRICAT_MSGS")))
(unless (find-package "TRICAT_MSGS::WAYPOINTSERVICE")
  (make-package "TRICAT_MSGS::WAYPOINTSERVICE"))
(unless (find-package "TRICAT_MSGS::WAYPOINTSERVICEREQUEST")
  (make-package "TRICAT_MSGS::WAYPOINTSERVICEREQUEST"))
(unless (find-package "TRICAT_MSGS::WAYPOINTSERVICERESPONSE")
  (make-package "TRICAT_MSGS::WAYPOINTSERVICERESPONSE"))

(in-package "ROS")





(defclass tricat_msgs::WaypointServiceRequest
  :super ros::object
  :slots ())

(defmethod tricat_msgs::WaypointServiceRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass tricat_msgs::WaypointServiceResponse
  :super ros::object
  :slots (_waypoint_list ))

(defmethod tricat_msgs::WaypointServiceResponse
  (:init
   (&key
    ((:waypoint_list __waypoint_list) (instance tricat_msgs::WPList :init))
    )
   (send-super :init)
   (setq _waypoint_list __waypoint_list)
   self)
  (:waypoint_list
   (&rest __waypoint_list)
   (if (keywordp (car __waypoint_list))
       (send* _waypoint_list __waypoint_list)
     (progn
       (if __waypoint_list (setq _waypoint_list (car __waypoint_list)))
       _waypoint_list)))
  (:serialization-length
   ()
   (+
    ;; tricat_msgs/WPList _waypoint_list
    (send _waypoint_list :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; tricat_msgs/WPList _waypoint_list
       (send _waypoint_list :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; tricat_msgs/WPList _waypoint_list
     (send _waypoint_list :deserialize buf ptr-) (incf ptr- (send _waypoint_list :serialization-length))
   ;;
   self)
  )

(defclass tricat_msgs::WaypointService
  :super ros::object
  :slots ())

(setf (get tricat_msgs::WaypointService :md5sum-) "bbca728dc1a971940e618ed2a281f3e2")
(setf (get tricat_msgs::WaypointService :datatype-) "tricat_msgs/WaypointService")
(setf (get tricat_msgs::WaypointService :request) tricat_msgs::WaypointServiceRequest)
(setf (get tricat_msgs::WaypointService :response) tricat_msgs::WaypointServiceResponse)

(defmethod tricat_msgs::WaypointServiceRequest
  (:response () (instance tricat_msgs::WaypointServiceResponse :init)))

(setf (get tricat_msgs::WaypointServiceRequest :md5sum-) "bbca728dc1a971940e618ed2a281f3e2")
(setf (get tricat_msgs::WaypointServiceRequest :datatype-) "tricat_msgs/WaypointServiceRequest")
(setf (get tricat_msgs::WaypointServiceRequest :definition-)
      "# Request
---
# Response
WPList waypoint_list

================================================================================
MSG: tricat_msgs/WPList
std_msgs/Header header
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
")

(setf (get tricat_msgs::WaypointServiceResponse :md5sum-) "bbca728dc1a971940e618ed2a281f3e2")
(setf (get tricat_msgs::WaypointServiceResponse :datatype-) "tricat_msgs/WaypointServiceResponse")
(setf (get tricat_msgs::WaypointServiceResponse :definition-)
      "# Request
---
# Response
WPList waypoint_list

================================================================================
MSG: tricat_msgs/WPList
std_msgs/Header header
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
")



(provide :tricat_msgs/WaypointService "bbca728dc1a971940e618ed2a281f3e2")


