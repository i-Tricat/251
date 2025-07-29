; Auto-generated. Do not edit!


(cl:in-package tricat_msgs-msg)


;//! \htmlinclude WPList.msg.html

(cl:defclass <WPList> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (WP_data
    :reader WP_data
    :initarg :WP_data
    :type (cl:vector tricat_msgs-msg:WP)
   :initform (cl:make-array 0 :element-type 'tricat_msgs-msg:WP :initial-element (cl:make-instance 'tricat_msgs-msg:WP))))
)

(cl:defclass WPList (<WPList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WPList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WPList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tricat_msgs-msg:<WPList> is deprecated: use tricat_msgs-msg:WPList instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <WPList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tricat_msgs-msg:header-val is deprecated.  Use tricat_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'WP_data-val :lambda-list '(m))
(cl:defmethod WP_data-val ((m <WPList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tricat_msgs-msg:WP_data-val is deprecated.  Use tricat_msgs-msg:WP_data instead.")
  (WP_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WPList>) ostream)
  "Serializes a message object of type '<WPList>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'WP_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'WP_data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WPList>) istream)
  "Deserializes a message object of type '<WPList>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'WP_data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'WP_data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'tricat_msgs-msg:WP))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WPList>)))
  "Returns string type for a message object of type '<WPList>"
  "tricat_msgs/WPList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WPList)))
  "Returns string type for a message object of type 'WPList"
  "tricat_msgs/WPList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WPList>)))
  "Returns md5sum for a message object of type '<WPList>"
  "f6d33a8e2e98aba038773acbcf874e13")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WPList)))
  "Returns md5sum for a message object of type 'WPList"
  "f6d33a8e2e98aba038773acbcf874e13")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WPList>)))
  "Returns full string definition for message of type '<WPList>"
  (cl:format cl:nil "std_msgs/Header header~%tricat_msgs/WP[] WP_data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: tricat_msgs/WP~%std_msgs/Float64 x~%std_msgs/Float64 y~%std_msgs/String type~%std_msgs/UInt16 num~%std_msgs/UInt16 range~%std_msgs/Bool arrive~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/UInt16~%uint16 data~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WPList)))
  "Returns full string definition for message of type 'WPList"
  (cl:format cl:nil "std_msgs/Header header~%tricat_msgs/WP[] WP_data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: tricat_msgs/WP~%std_msgs/Float64 x~%std_msgs/Float64 y~%std_msgs/String type~%std_msgs/UInt16 num~%std_msgs/UInt16 range~%std_msgs/Bool arrive~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/UInt16~%uint16 data~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WPList>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'WP_data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WPList>))
  "Converts a ROS message object to a list"
  (cl:list 'WPList
    (cl:cons ':header (header msg))
    (cl:cons ':WP_data (WP_data msg))
))
