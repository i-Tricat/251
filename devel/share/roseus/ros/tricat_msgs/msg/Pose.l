;; Auto-generated. Do not edit!


(when (boundp 'tricat_msgs::Pose)
  (if (not (find-package "TRICAT_MSGS"))
    (make-package "TRICAT_MSGS"))
  (shadow 'Pose (find-package "TRICAT_MSGS")))
(unless (find-package "TRICAT_MSGS::POSE")
  (make-package "TRICAT_MSGS::POSE"))

(in-package "ROS")
;;//! \htmlinclude Pose.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass tricat_msgs::Pose
  :super ros::object
  :slots (_header _x _y _psi _u _v _r ))

(defmethod tricat_msgs::Pose
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:x __x) (instance std_msgs::Float64 :init))
    ((:y __y) (instance std_msgs::Float64 :init))
    ((:psi __psi) (instance std_msgs::Float64 :init))
    ((:u __u) (instance std_msgs::Float64 :init))
    ((:v __v) (instance std_msgs::Float64 :init))
    ((:r __r) (instance std_msgs::Float64 :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _x __x)
   (setq _y __y)
   (setq _psi __psi)
   (setq _u __u)
   (setq _v __v)
   (setq _r __r)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:x
   (&rest __x)
   (if (keywordp (car __x))
       (send* _x __x)
     (progn
       (if __x (setq _x (car __x)))
       _x)))
  (:y
   (&rest __y)
   (if (keywordp (car __y))
       (send* _y __y)
     (progn
       (if __y (setq _y (car __y)))
       _y)))
  (:psi
   (&rest __psi)
   (if (keywordp (car __psi))
       (send* _psi __psi)
     (progn
       (if __psi (setq _psi (car __psi)))
       _psi)))
  (:u
   (&rest __u)
   (if (keywordp (car __u))
       (send* _u __u)
     (progn
       (if __u (setq _u (car __u)))
       _u)))
  (:v
   (&rest __v)
   (if (keywordp (car __v))
       (send* _v __v)
     (progn
       (if __v (setq _v (car __v)))
       _v)))
  (:r
   (&rest __r)
   (if (keywordp (car __r))
       (send* _r __r)
     (progn
       (if __r (setq _r (car __r)))
       _r)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; std_msgs/Float64 _x
    (send _x :serialization-length)
    ;; std_msgs/Float64 _y
    (send _y :serialization-length)
    ;; std_msgs/Float64 _psi
    (send _psi :serialization-length)
    ;; std_msgs/Float64 _u
    (send _u :serialization-length)
    ;; std_msgs/Float64 _v
    (send _v :serialization-length)
    ;; std_msgs/Float64 _r
    (send _r :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; std_msgs/Float64 _x
       (send _x :serialize s)
     ;; std_msgs/Float64 _y
       (send _y :serialize s)
     ;; std_msgs/Float64 _psi
       (send _psi :serialize s)
     ;; std_msgs/Float64 _u
       (send _u :serialize s)
     ;; std_msgs/Float64 _v
       (send _v :serialize s)
     ;; std_msgs/Float64 _r
       (send _r :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; std_msgs/Float64 _x
     (send _x :deserialize buf ptr-) (incf ptr- (send _x :serialization-length))
   ;; std_msgs/Float64 _y
     (send _y :deserialize buf ptr-) (incf ptr- (send _y :serialization-length))
   ;; std_msgs/Float64 _psi
     (send _psi :deserialize buf ptr-) (incf ptr- (send _psi :serialization-length))
   ;; std_msgs/Float64 _u
     (send _u :deserialize buf ptr-) (incf ptr- (send _u :serialization-length))
   ;; std_msgs/Float64 _v
     (send _v :deserialize buf ptr-) (incf ptr- (send _v :serialization-length))
   ;; std_msgs/Float64 _r
     (send _r :deserialize buf ptr-) (incf ptr- (send _r :serialization-length))
   ;;
   self)
  )

(setf (get tricat_msgs::Pose :md5sum-) "7faf8df64baddc9fd0c262f0be990408")
(setf (get tricat_msgs::Pose :datatype-) "tricat_msgs/Pose")
(setf (get tricat_msgs::Pose :definition-)
      "std_msgs/Header header
std_msgs/Float64 x
std_msgs/Float64 y
std_msgs/Float64 psi
std_msgs/Float64 u
std_msgs/Float64 v
std_msgs/Float64 r
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
MSG: std_msgs/Float64
float64 data
")



(provide :tricat_msgs/Pose "7faf8df64baddc9fd0c262f0be990408")


