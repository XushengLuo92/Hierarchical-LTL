
(cl:in-package :asdf)

(defsystem "moveit_fanuc1_config-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "robot_action" :depends-on ("_package_robot_action"))
    (:file "_package_robot_action" :depends-on ("_package"))
  ))