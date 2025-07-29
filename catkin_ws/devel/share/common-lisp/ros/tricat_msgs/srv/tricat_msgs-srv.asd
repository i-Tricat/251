
(cl:in-package :asdf)

(defsystem "tricat_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :tricat_msgs-msg
)
  :components ((:file "_package")
    (:file "WaypointService" :depends-on ("_package_WaypointService"))
    (:file "_package_WaypointService" :depends-on ("_package"))
  ))