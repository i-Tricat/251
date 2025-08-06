; Auto-generated. Do not edit!


(cl:in-package tricat_msgs-msg)


;//! \htmlinclude Obstacle.msg.html

(cl:defclass <Obstacle> (roslisp-msg-protocol:ros-message)
  ((begin
    :reader begin
    :initarg :begin
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (end
    :reader end
    :initarg :end
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass Obstacle (<Obstacle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Obstacle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Obstacle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tricat_msgs-msg:<Obstacle> is deprecated: use tricat_msgs-msg:Obstacle instead.")))

(cl:ensure-generic-function 'begin-val :lambda-list '(m))
(cl:defmethod begin-val ((m <Obstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tricat_msgs-msg:begin-val is deprecated.  Use tricat_msgs-msg:begin instead.")
  (begin m))

(cl:ensure-generic-function 'end-val :lambda-list '(m))
(cl:defmethod end-val ((m <Obstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tricat_msgs-msg:end-val is deprecated.  Use tricat_msgs-msg:end instead.")
  (end m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Obstacle>) ostream)
  "Serializes a message object of type '<Obstacle>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'begin) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'end) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Obstacle>) istream)
  "Deserializes a message object of type '<Obstacle>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'begin) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'end) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Obstacle>)))
  "Returns string type for a message object of type '<Obstacle>"
  "tricat_msgs/Obstacle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Obstacle)))
  "Returns string type for a message object of type 'Obstacle"
  "tricat_msgs/Obstacle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Obstacle>)))
  "Returns md5sum for a message object of type '<Obstacle>"
  "41e958a799f3b38d26f0715b99f1cec2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Obstacle)))
  "Returns md5sum for a message object of type 'Obstacle"
  "41e958a799f3b38d26f0715b99f1cec2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Obstacle>)))
  "Returns full string definition for message of type '<Obstacle>"
  (cl:format cl:nil "geometry_msgs/Point begin~%geometry_msgs/Point end~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Obstacle)))
  "Returns full string definition for message of type 'Obstacle"
  (cl:format cl:nil "geometry_msgs/Point begin~%geometry_msgs/Point end~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Obstacle>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'begin))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'end))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Obstacle>))
  "Converts a ROS message object to a list"
  (cl:list 'Obstacle
    (cl:cons ':begin (begin msg))
    (cl:cons ':end (end msg))
))
