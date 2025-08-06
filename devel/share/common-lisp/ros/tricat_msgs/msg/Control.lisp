; Auto-generated. Do not edit!


(cl:in-package tricat_msgs-msg)


;//! \htmlinclude Control.msg.html

(cl:defclass <Control> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (servo_s
    :reader servo_s
    :initarg :servo_s
    :type std_msgs-msg:UInt16
    :initform (cl:make-instance 'std_msgs-msg:UInt16))
   (servo_p
    :reader servo_p
    :initarg :servo_p
    :type std_msgs-msg:UInt16
    :initform (cl:make-instance 'std_msgs-msg:UInt16))
   (thruster_p
    :reader thruster_p
    :initarg :thruster_p
    :type std_msgs-msg:UInt16
    :initform (cl:make-instance 'std_msgs-msg:UInt16))
   (thruster_s
    :reader thruster_s
    :initarg :thruster_s
    :type std_msgs-msg:UInt16
    :initform (cl:make-instance 'std_msgs-msg:UInt16)))
)

(cl:defclass Control (<Control>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Control>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Control)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tricat_msgs-msg:<Control> is deprecated: use tricat_msgs-msg:Control instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tricat_msgs-msg:header-val is deprecated.  Use tricat_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'servo_s-val :lambda-list '(m))
(cl:defmethod servo_s-val ((m <Control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tricat_msgs-msg:servo_s-val is deprecated.  Use tricat_msgs-msg:servo_s instead.")
  (servo_s m))

(cl:ensure-generic-function 'servo_p-val :lambda-list '(m))
(cl:defmethod servo_p-val ((m <Control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tricat_msgs-msg:servo_p-val is deprecated.  Use tricat_msgs-msg:servo_p instead.")
  (servo_p m))

(cl:ensure-generic-function 'thruster_p-val :lambda-list '(m))
(cl:defmethod thruster_p-val ((m <Control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tricat_msgs-msg:thruster_p-val is deprecated.  Use tricat_msgs-msg:thruster_p instead.")
  (thruster_p m))

(cl:ensure-generic-function 'thruster_s-val :lambda-list '(m))
(cl:defmethod thruster_s-val ((m <Control>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tricat_msgs-msg:thruster_s-val is deprecated.  Use tricat_msgs-msg:thruster_s instead.")
  (thruster_s m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Control>) ostream)
  "Serializes a message object of type '<Control>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'servo_s) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'servo_p) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'thruster_p) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'thruster_s) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Control>) istream)
  "Deserializes a message object of type '<Control>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'servo_s) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'servo_p) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'thruster_p) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'thruster_s) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Control>)))
  "Returns string type for a message object of type '<Control>"
  "tricat_msgs/Control")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Control)))
  "Returns string type for a message object of type 'Control"
  "tricat_msgs/Control")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Control>)))
  "Returns md5sum for a message object of type '<Control>"
  "b0e90ce225824dce6ea0018e8f0cbd3f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Control)))
  "Returns md5sum for a message object of type 'Control"
  "b0e90ce225824dce6ea0018e8f0cbd3f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Control>)))
  "Returns full string definition for message of type '<Control>"
  (cl:format cl:nil "std_msgs/Header header~%std_msgs/UInt16 servo_s~%std_msgs/UInt16 servo_p~%std_msgs/UInt16 thruster_p~%std_msgs/UInt16 thruster_s~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/UInt16~%uint16 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Control)))
  "Returns full string definition for message of type 'Control"
  (cl:format cl:nil "std_msgs/Header header~%std_msgs/UInt16 servo_s~%std_msgs/UInt16 servo_p~%std_msgs/UInt16 thruster_p~%std_msgs/UInt16 thruster_s~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: std_msgs/UInt16~%uint16 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Control>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'servo_s))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'servo_p))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'thruster_p))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'thruster_s))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Control>))
  "Converts a ROS message object to a list"
  (cl:list 'Control
    (cl:cons ':header (header msg))
    (cl:cons ':servo_s (servo_s msg))
    (cl:cons ':servo_p (servo_p msg))
    (cl:cons ':thruster_p (thruster_p msg))
    (cl:cons ':thruster_s (thruster_s msg))
))
