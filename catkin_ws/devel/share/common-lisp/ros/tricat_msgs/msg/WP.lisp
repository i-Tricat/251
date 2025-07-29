; Auto-generated. Do not edit!


(cl:in-package tricat_msgs-msg)


;//! \htmlinclude WP.msg.html

(cl:defclass <WP> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (y
    :reader y
    :initarg :y
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (type
    :reader type
    :initarg :type
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String))
   (num
    :reader num
    :initarg :num
    :type std_msgs-msg:UInt16
    :initform (cl:make-instance 'std_msgs-msg:UInt16))
   (range
    :reader range
    :initarg :range
    :type std_msgs-msg:UInt16
    :initform (cl:make-instance 'std_msgs-msg:UInt16))
   (arrive
    :reader arrive
    :initarg :arrive
    :type std_msgs-msg:Bool
    :initform (cl:make-instance 'std_msgs-msg:Bool)))
)

(cl:defclass WP (<WP>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WP>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WP)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tricat_msgs-msg:<WP> is deprecated: use tricat_msgs-msg:WP instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <WP>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tricat_msgs-msg:x-val is deprecated.  Use tricat_msgs-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <WP>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tricat_msgs-msg:y-val is deprecated.  Use tricat_msgs-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <WP>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tricat_msgs-msg:type-val is deprecated.  Use tricat_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'num-val :lambda-list '(m))
(cl:defmethod num-val ((m <WP>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tricat_msgs-msg:num-val is deprecated.  Use tricat_msgs-msg:num instead.")
  (num m))

(cl:ensure-generic-function 'range-val :lambda-list '(m))
(cl:defmethod range-val ((m <WP>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tricat_msgs-msg:range-val is deprecated.  Use tricat_msgs-msg:range instead.")
  (range m))

(cl:ensure-generic-function 'arrive-val :lambda-list '(m))
(cl:defmethod arrive-val ((m <WP>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tricat_msgs-msg:arrive-val is deprecated.  Use tricat_msgs-msg:arrive instead.")
  (arrive m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WP>) ostream)
  "Serializes a message object of type '<WP>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'x) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'y) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'type) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'num) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'range) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'arrive) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WP>) istream)
  "Deserializes a message object of type '<WP>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'x) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'y) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'type) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'num) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'range) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'arrive) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WP>)))
  "Returns string type for a message object of type '<WP>"
  "tricat_msgs/WP")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WP)))
  "Returns string type for a message object of type 'WP"
  "tricat_msgs/WP")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WP>)))
  "Returns md5sum for a message object of type '<WP>"
  "e86d166212e6ad49f8865c7a6bb74d08")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WP)))
  "Returns md5sum for a message object of type 'WP"
  "e86d166212e6ad49f8865c7a6bb74d08")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WP>)))
  "Returns full string definition for message of type '<WP>"
  (cl:format cl:nil "std_msgs/Float64 x~%std_msgs/Float64 y~%std_msgs/String type~%std_msgs/UInt16 num~%std_msgs/UInt16 range~%std_msgs/Bool arrive~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/UInt16~%uint16 data~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WP)))
  "Returns full string definition for message of type 'WP"
  (cl:format cl:nil "std_msgs/Float64 x~%std_msgs/Float64 y~%std_msgs/String type~%std_msgs/UInt16 num~%std_msgs/UInt16 range~%std_msgs/Bool arrive~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/UInt16~%uint16 data~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WP>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'x))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'y))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'type))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'num))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'range))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'arrive))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WP>))
  "Converts a ROS message object to a list"
  (cl:list 'WP
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':type (type msg))
    (cl:cons ':num (num msg))
    (cl:cons ':range (range msg))
    (cl:cons ':arrive (arrive msg))
))
