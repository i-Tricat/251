; Auto-generated. Do not edit!


(cl:in-package tricat_msgs-msg)


;//! \htmlinclude ObstacleList.msg.html

(cl:defclass <ObstacleList> (roslisp-msg-protocol:ros-message)
  ((lidar_header
    :reader lidar_header
    :initarg :lidar_header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (obstacle
    :reader obstacle
    :initarg :obstacle
    :type (cl:vector tricat_msgs-msg:Obstacle)
   :initform (cl:make-array 0 :element-type 'tricat_msgs-msg:Obstacle :initial-element (cl:make-instance 'tricat_msgs-msg:Obstacle))))
)

(cl:defclass ObstacleList (<ObstacleList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObstacleList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObstacleList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tricat_msgs-msg:<ObstacleList> is deprecated: use tricat_msgs-msg:ObstacleList instead.")))

(cl:ensure-generic-function 'lidar_header-val :lambda-list '(m))
(cl:defmethod lidar_header-val ((m <ObstacleList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tricat_msgs-msg:lidar_header-val is deprecated.  Use tricat_msgs-msg:lidar_header instead.")
  (lidar_header m))

(cl:ensure-generic-function 'obstacle-val :lambda-list '(m))
(cl:defmethod obstacle-val ((m <ObstacleList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tricat_msgs-msg:obstacle-val is deprecated.  Use tricat_msgs-msg:obstacle instead.")
  (obstacle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObstacleList>) ostream)
  "Serializes a message object of type '<ObstacleList>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'lidar_header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'obstacle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'obstacle))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObstacleList>) istream)
  "Deserializes a message object of type '<ObstacleList>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'lidar_header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'obstacle) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'obstacle)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'tricat_msgs-msg:Obstacle))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObstacleList>)))
  "Returns string type for a message object of type '<ObstacleList>"
  "tricat_msgs/ObstacleList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObstacleList)))
  "Returns string type for a message object of type 'ObstacleList"
  "tricat_msgs/ObstacleList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObstacleList>)))
  "Returns md5sum for a message object of type '<ObstacleList>"
  "e58b9c6e530a428f4aad1beec1ae1b08")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObstacleList)))
  "Returns md5sum for a message object of type 'ObstacleList"
  "e58b9c6e530a428f4aad1beec1ae1b08")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObstacleList>)))
  "Returns full string definition for message of type '<ObstacleList>"
  (cl:format cl:nil "std_msgs/Header lidar_header~%tricat_msgs/Obstacle[] obstacle~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: tricat_msgs/Obstacle~%geometry_msgs/Point begin~%geometry_msgs/Point end~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObstacleList)))
  "Returns full string definition for message of type 'ObstacleList"
  (cl:format cl:nil "std_msgs/Header lidar_header~%tricat_msgs/Obstacle[] obstacle~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: tricat_msgs/Obstacle~%geometry_msgs/Point begin~%geometry_msgs/Point end~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObstacleList>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'lidar_header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'obstacle) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObstacleList>))
  "Converts a ROS message object to a list"
  (cl:list 'ObstacleList
    (cl:cons ':lidar_header (lidar_header msg))
    (cl:cons ':obstacle (obstacle msg))
))
