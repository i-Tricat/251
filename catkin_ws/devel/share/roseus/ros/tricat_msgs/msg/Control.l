;; Auto-generated. Do not edit!


(when (boundp 'tricat_msgs::Control)
  (if (not (find-package "TRICAT_MSGS"))
    (make-package "TRICAT_MSGS"))
  (shadow 'Control (find-package "TRICAT_MSGS")))
(unless (find-package "TRICAT_MSGS::CONTROL")
  (make-package "TRICAT_MSGS::CONTROL"))

(in-package "ROS")
;;//! \htmlinclude Control.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass tricat_msgs::Control
  :super ros::object
  :slots (_header _servo_s _servo_p _thruster_p _thruster_s ))

(defmethod tricat_msgs::Control
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:servo_s __servo_s) (instance std_msgs::UInt16 :init))
    ((:servo_p __servo_p) (instance std_msgs::UInt16 :init))
    ((:thruster_p __thruster_p) (instance std_msgs::UInt16 :init))
    ((:thruster_s __thruster_s) (instance std_msgs::UInt16 :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _servo_s __servo_s)
   (setq _servo_p __servo_p)
   (setq _thruster_p __thruster_p)
   (setq _thruster_s __thruster_s)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:servo_s
   (&rest __servo_s)
   (if (keywordp (car __servo_s))
       (send* _servo_s __servo_s)
     (progn
       (if __servo_s (setq _servo_s (car __servo_s)))
       _servo_s)))
  (:servo_p
   (&rest __servo_p)
   (if (keywordp (car __servo_p))
       (send* _servo_p __servo_p)
     (progn
       (if __servo_p (setq _servo_p (car __servo_p)))
       _servo_p)))
  (:thruster_p
   (&rest __thruster_p)
   (if (keywordp (car __thruster_p))
       (send* _thruster_p __thruster_p)
     (progn
       (if __thruster_p (setq _thruster_p (car __thruster_p)))
       _thruster_p)))
  (:thruster_s
   (&rest __thruster_s)
   (if (keywordp (car __thruster_s))
       (send* _thruster_s __thruster_s)
     (progn
       (if __thruster_s (setq _thruster_s (car __thruster_s)))
       _thruster_s)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; std_msgs/UInt16 _servo_s
    (send _servo_s :serialization-length)
    ;; std_msgs/UInt16 _servo_p
    (send _servo_p :serialization-length)
    ;; std_msgs/UInt16 _thruster_p
    (send _thruster_p :serialization-length)
    ;; std_msgs/UInt16 _thruster_s
    (send _thruster_s :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; std_msgs/UInt16 _servo_s
       (send _servo_s :serialize s)
     ;; std_msgs/UInt16 _servo_p
       (send _servo_p :serialize s)
     ;; std_msgs/UInt16 _thruster_p
       (send _thruster_p :serialize s)
     ;; std_msgs/UInt16 _thruster_s
       (send _thruster_s :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; std_msgs/UInt16 _servo_s
     (send _servo_s :deserialize buf ptr-) (incf ptr- (send _servo_s :serialization-length))
   ;; std_msgs/UInt16 _servo_p
     (send _servo_p :deserialize buf ptr-) (incf ptr- (send _servo_p :serialization-length))
   ;; std_msgs/UInt16 _thruster_p
     (send _thruster_p :deserialize buf ptr-) (incf ptr- (send _thruster_p :serialization-length))
   ;; std_msgs/UInt16 _thruster_s
     (send _thruster_s :deserialize buf ptr-) (incf ptr- (send _thruster_s :serialization-length))
   ;;
   self)
  )

(setf (get tricat_msgs::Control :md5sum-) "b0e90ce225824dce6ea0018e8f0cbd3f")
(setf (get tricat_msgs::Control :datatype-) "tricat_msgs/Control")
(setf (get tricat_msgs::Control :definition-)
      "std_msgs/Header header
std_msgs/UInt16 servo_s
std_msgs/UInt16 servo_p
std_msgs/UInt16 thruster_p
std_msgs/UInt16 thruster_s
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
MSG: std_msgs/UInt16
uint16 data

")



(provide :tricat_msgs/Control "b0e90ce225824dce6ea0018e8f0cbd3f")


