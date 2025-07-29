
(cl:in-package :asdf)

(defsystem "tricat_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Control" :depends-on ("_package_Control"))
    (:file "_package_Control" :depends-on ("_package"))
    (:file "Obstacle" :depends-on ("_package_Obstacle"))
    (:file "_package_Obstacle" :depends-on ("_package"))
    (:file "ObstacleList" :depends-on ("_package_ObstacleList"))
    (:file "_package_ObstacleList" :depends-on ("_package"))
    (:file "Pose" :depends-on ("_package_Pose"))
    (:file "_package_Pose" :depends-on ("_package"))
    (:file "Sensor_total" :depends-on ("_package_Sensor_total"))
    (:file "_package_Sensor_total" :depends-on ("_package"))
    (:file "WP" :depends-on ("_package_WP"))
    (:file "_package_WP" :depends-on ("_package"))
    (:file "WPList" :depends-on ("_package_WPList"))
    (:file "_package_WPList" :depends-on ("_package"))
  ))