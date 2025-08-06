; Auto-generated. Do not edit!


(cl:in-package tricat_msgs-srv)


;//! \htmlinclude WaypointService-request.msg.html

(cl:defclass <WaypointService-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass WaypointService-request (<WaypointService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WaypointService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WaypointService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tricat_msgs-srv:<WaypointService-request> is deprecated: use tricat_msgs-srv:WaypointService-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WaypointService-request>) ostream)
  "Serializes a message object of type '<WaypointService-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WaypointService-request>) istream)
  "Deserializes a message object of type '<WaypointService-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WaypointService-request>)))
  "Returns string type for a service object of type '<WaypointService-request>"
  "tricat_msgs/WaypointServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WaypointService-request)))
  "Returns string type for a service object of type 'WaypointService-request"
  "tricat_msgs/WaypointServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WaypointService-request>)))
  "Returns md5sum for a message object of type '<WaypointService-request>"
  "bbca728dc1a971940e618ed2a281f3e2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WaypointService-request)))
  "Returns md5sum for a message object of type 'WaypointService-request"
  "bbca728dc1a971940e618ed2a281f3e2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WaypointService-request>)))
  "Returns full string definition for message of type '<WaypointService-request>"
  (cl:format cl:nil "# Request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WaypointService-request)))
  "Returns full string definition for message of type 'WaypointService-request"
  (cl:format cl:nil "# Request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WaypointService-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WaypointService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'WaypointService-request
))
;//! \htmlinclude WaypointService-response.msg.html

(cl:defclass <WaypointService-response> (roslisp-msg-protocol:ros-message)
  ((waypoint_list
    :reader waypoint_list
    :initarg :waypoint_list
    :type tricat_msgs-msg:WPList
    :initform (cl:make-instance 'tricat_msgs-msg:WPList)))
)

(cl:defclass WaypointService-response (<WaypointService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WaypointService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WaypointService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tricat_msgs-srv:<WaypointService-response> is deprecated: use tricat_msgs-srv:WaypointService-response instead.")))

(cl:ensure-generic-function 'waypoint_list-val :lambda-list '(m))
(cl:defmethod waypoint_list-val ((m <WaypointService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tricat_msgs-srv:waypoint_list-val is deprecated.  Use tricat_msgs-srv:waypoint_list instead.")
  (waypoint_list m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WaypointService-response>) ostream)
  "Serializes a message object of type '<WaypointService-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'waypoint_list) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WaypointService-response>) istream)
  "Deserializes a message object of type '<WaypointService-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'waypoint_list) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WaypointService-response>)))
  "Returns string type for a service object of type '<WaypointService-response>"
  "tricat_msgs/WaypointServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WaypointService-response)))
  "Returns string type for a service object of type 'WaypointService-response"
  "tricat_msgs/WaypointServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WaypointService-response>)))
  "Returns md5sum for a message object of type '<WaypointService-response>"
  "bbca728dc1a971940e618ed2a281f3e2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WaypointService-response)))
  "Returns md5sum for a message object of type 'WaypointService-response"
  "bbca728dc1a971940e618ed2a281f3e2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WaypointService-response>)))
  "Returns full string definition for message of type '<WaypointService-response>"
  (cl:format cl:nil "# Response~%WPList waypoint_list~%~%================================================================================~%MSG: tricat_msgs/WPList~%std_msgs/Header header~%tricat_msgs/WP[] WP_data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: tricat_msgs/WP~%std_msgs/Float64 x~%std_msgs/Float64 y~%std_msgs/String type~%std_msgs/UInt16 num~%std_msgs/UInt16 range~%std_msgs/Bool arrive~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/UInt16~%uint16 data~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WaypointService-response)))
  "Returns full string definition for message of type 'WaypointService-response"
  (cl:format cl:nil "# Response~%WPList waypoint_list~%~%================================================================================~%MSG: tricat_msgs/WPList~%std_msgs/Header header~%tricat_msgs/WP[] WP_data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: tricat_msgs/WP~%std_msgs/Float64 x~%std_msgs/Float64 y~%std_msgs/String type~%std_msgs/UInt16 num~%std_msgs/UInt16 range~%std_msgs/Bool arrive~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/UInt16~%uint16 data~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WaypointService-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'waypoint_list))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WaypointService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'WaypointService-response
    (cl:cons ':waypoint_list (waypoint_list msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'WaypointService)))
  'WaypointService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'WaypointService)))
  'WaypointService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WaypointService)))
  "Returns string type for a service object of type '<WaypointService>"
  "tricat_msgs/WaypointService")