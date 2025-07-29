; Auto-generated. Do not edit!


(cl:in-package tricat_msgs-msg)


;//! \htmlinclude Sensor_total.msg.html

(cl:defclass <Sensor_total> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (position_ned
    :reader position_ned
    :initarg :position_ned
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (psi
    :reader psi
    :initarg :psi
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64)))
)

(cl:defclass Sensor_total (<Sensor_total>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Sensor_total>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Sensor_total)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tricat_msgs-msg:<Sensor_total> is deprecated: use tricat_msgs-msg:Sensor_total instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Sensor_total>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tricat_msgs-msg:header-val is deprecated.  Use tricat_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'position_ned-val :lambda-list '(m))
(cl:defmethod position_ned-val ((m <Sensor_total>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tricat_msgs-msg:position_ned-val is deprecated.  Use tricat_msgs-msg:position_ned instead.")
  (position_ned m))

(cl:ensure-generic-function 'psi-val :lambda-list '(m))
(cl:defmethod psi-val ((m <Sensor_total>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tricat_msgs-msg:psi-val is deprecated.  Use tricat_msgs-msg:psi instead.")
  (psi m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Sensor_total>) ostream)
  "Serializes a message object of type '<Sensor_total>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position_ned) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'psi) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Sensor_total>) istream)
  "Deserializes a message object of type '<Sensor_total>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position_ned) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'psi) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Sensor_total>)))
  "Returns string type for a message object of type '<Sensor_total>"
  "tricat_msgs/Sensor_total")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Sensor_total)))
  "Returns string type for a message object of type 'Sensor_total"
  "tricat_msgs/Sensor_total")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Sensor_total>)))
  "Returns md5sum for a message object of type '<Sensor_total>"
  "cfffd2feabde8d02a0b65cd795f0e07d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Sensor_total)))
  "Returns md5sum for a message object of type 'Sensor_total"
  "cfffd2feabde8d02a0b65cd795f0e07d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Sensor_total>)))
  "Returns full string definition for message of type '<Sensor_total>"
  (cl:format cl:nil "std_msgs/Header header~%geometry_msgs/Point position_ned~%std_msgs/Float64 psi~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Sensor_total)))
  "Returns full string definition for message of type 'Sensor_total"
  (cl:format cl:nil "std_msgs/Header header~%geometry_msgs/Point position_ned~%std_msgs/Float64 psi~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Sensor_total>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position_ned))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'psi))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Sensor_total>))
  "Converts a ROS message object to a list"
  (cl:list 'Sensor_total
    (cl:cons ':header (header msg))
    (cl:cons ':position_ned (position_ned msg))
    (cl:cons ':psi (psi msg))
))
