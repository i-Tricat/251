;; Auto-generated. Do not edit!


(when (boundp 'tricat_msgs::WP)
  (if (not (find-package "TRICAT_MSGS"))
    (make-package "TRICAT_MSGS"))
  (shadow 'WP (find-package "TRICAT_MSGS")))
(unless (find-package "TRICAT_MSGS::WP")
  (make-package "TRICAT_MSGS::WP"))

(in-package "ROS")
;;//! \htmlinclude WP.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass tricat_msgs::WP
  :super ros::object
  :slots (_x _y _type _num _range _arrive ))

(defmethod tricat_msgs::WP
  (:init
   (&key
    ((:x __x) (instance std_msgs::Float64 :init))
    ((:y __y) (instance std_msgs::Float64 :init))
    ((:type __type) (instance std_msgs::String :init))
    ((:num __num) (instance std_msgs::UInt16 :init))
    ((:range __range) (instance std_msgs::UInt16 :init))
    ((:arrive __arrive) (instance std_msgs::Bool :init))
    )
   (send-super :init)
   (setq _x __x)
   (setq _y __y)
   (setq _type __type)
   (setq _num __num)
   (setq _range __range)
   (setq _arrive __arrive)
   self)
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
  (:type
   (&rest __type)
   (if (keywordp (car __type))
       (send* _type __type)
     (progn
       (if __type (setq _type (car __type)))
       _type)))
  (:num
   (&rest __num)
   (if (keywordp (car __num))
       (send* _num __num)
     (progn
       (if __num (setq _num (car __num)))
       _num)))
  (:range
   (&rest __range)
   (if (keywordp (car __range))
       (send* _range __range)
     (progn
       (if __range (setq _range (car __range)))
       _range)))
  (:arrive
   (&rest __arrive)
   (if (keywordp (car __arrive))
       (send* _arrive __arrive)
     (progn
       (if __arrive (setq _arrive (car __arrive)))
       _arrive)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float64 _x
    (send _x :serialization-length)
    ;; std_msgs/Float64 _y
    (send _y :serialization-length)
    ;; std_msgs/String _type
    (send _type :serialization-length)
    ;; std_msgs/UInt16 _num
    (send _num :serialization-length)
    ;; std_msgs/UInt16 _range
    (send _range :serialization-length)
    ;; std_msgs/Bool _arrive
    (send _arrive :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float64 _x
       (send _x :serialize s)
     ;; std_msgs/Float64 _y
       (send _y :serialize s)
     ;; std_msgs/String _type
       (send _type :serialize s)
     ;; std_msgs/UInt16 _num
       (send _num :serialize s)
     ;; std_msgs/UInt16 _range
       (send _range :serialize s)
     ;; std_msgs/Bool _arrive
       (send _arrive :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float64 _x
     (send _x :deserialize buf ptr-) (incf ptr- (send _x :serialization-length))
   ;; std_msgs/Float64 _y
     (send _y :deserialize buf ptr-) (incf ptr- (send _y :serialization-length))
   ;; std_msgs/String _type
     (send _type :deserialize buf ptr-) (incf ptr- (send _type :serialization-length))
   ;; std_msgs/UInt16 _num
     (send _num :deserialize buf ptr-) (incf ptr- (send _num :serialization-length))
   ;; std_msgs/UInt16 _range
     (send _range :deserialize buf ptr-) (incf ptr- (send _range :serialization-length))
   ;; std_msgs/Bool _arrive
     (send _arrive :deserialize buf ptr-) (incf ptr- (send _arrive :serialization-length))
   ;;
   self)
  )

(setf (get tricat_msgs::WP :md5sum-) "e86d166212e6ad49f8865c7a6bb74d08")
(setf (get tricat_msgs::WP :datatype-) "tricat_msgs/WP")
(setf (get tricat_msgs::WP :definition-)
      "std_msgs/Float64 x
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
bool data
")



(provide :tricat_msgs/WP "e86d166212e6ad49f8865c7a6bb74d08")


