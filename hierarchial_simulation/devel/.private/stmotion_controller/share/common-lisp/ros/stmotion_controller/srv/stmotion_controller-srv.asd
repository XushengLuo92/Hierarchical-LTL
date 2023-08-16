
(cl:in-package :asdf)

(defsystem "stmotion_controller-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "lego_pickup" :depends-on ("_package_lego_pickup"))
    (:file "_package_lego_pickup" :depends-on ("_package"))
    (:file "robot_action" :depends-on ("_package_robot_action"))
    (:file "_package_robot_action" :depends-on ("_package"))
  ))