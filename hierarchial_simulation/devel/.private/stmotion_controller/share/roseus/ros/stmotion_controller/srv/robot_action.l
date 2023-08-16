;; Auto-generated. Do not edit!


(when (boundp 'stmotion_controller::robot_action)
  (if (not (find-package "STMOTION_CONTROLLER"))
    (make-package "STMOTION_CONTROLLER"))
  (shadow 'robot_action (find-package "STMOTION_CONTROLLER")))
(unless (find-package "STMOTION_CONTROLLER::ROBOT_ACTION")
  (make-package "STMOTION_CONTROLLER::ROBOT_ACTION"))
(unless (find-package "STMOTION_CONTROLLER::ROBOT_ACTIONREQUEST")
  (make-package "STMOTION_CONTROLLER::ROBOT_ACTIONREQUEST"))
(unless (find-package "STMOTION_CONTROLLER::ROBOT_ACTIONRESPONSE")
  (make-package "STMOTION_CONTROLLER::ROBOT_ACTIONRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass stmotion_controller::robot_actionRequest
  :super ros::object
  :slots (_des _src ))

(defmethod stmotion_controller::robot_actionRequest
  (:init
   (&key
    ((:des __des) (instance geometry_msgs::PoseStamped :init))
    ((:src __src) (instance geometry_msgs::PoseStamped :init))
    )
   (send-super :init)
   (setq _des __des)
   (setq _src __src)
   self)
  (:des
   (&rest __des)
   (if (keywordp (car __des))
       (send* _des __des)
     (progn
       (if __des (setq _des (car __des)))
       _des)))
  (:src
   (&rest __src)
   (if (keywordp (car __src))
       (send* _src __src)
     (progn
       (if __src (setq _src (car __src)))
       _src)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PoseStamped _des
    (send _des :serialization-length)
    ;; geometry_msgs/PoseStamped _src
    (send _src :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PoseStamped _des
       (send _des :serialize s)
     ;; geometry_msgs/PoseStamped _src
       (send _src :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PoseStamped _des
     (send _des :deserialize buf ptr-) (incf ptr- (send _des :serialization-length))
   ;; geometry_msgs/PoseStamped _src
     (send _src :deserialize buf ptr-) (incf ptr- (send _src :serialization-length))
   ;;
   self)
  )

(defclass stmotion_controller::robot_actionResponse
  :super ros::object
  :slots (_finished ))

(defmethod stmotion_controller::robot_actionResponse
  (:init
   (&key
    ((:finished __finished) nil)
    )
   (send-super :init)
   (setq _finished __finished)
   self)
  (:finished
   (&optional (__finished :null))
   (if (not (eq __finished :null)) (setq _finished __finished)) _finished)
  (:serialization-length
   ()
   (+
    ;; bool _finished
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _finished
       (if _finished (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _finished
     (setq _finished (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass stmotion_controller::robot_action
  :super ros::object
  :slots ())

(setf (get stmotion_controller::robot_action :md5sum-) "19a560c38819713863fdccb541e95a18")
(setf (get stmotion_controller::robot_action :datatype-) "stmotion_controller/robot_action")
(setf (get stmotion_controller::robot_action :request) stmotion_controller::robot_actionRequest)
(setf (get stmotion_controller::robot_action :response) stmotion_controller::robot_actionResponse)

(defmethod stmotion_controller::robot_actionRequest
  (:response () (instance stmotion_controller::robot_actionResponse :init)))

(setf (get stmotion_controller::robot_actionRequest :md5sum-) "19a560c38819713863fdccb541e95a18")
(setf (get stmotion_controller::robot_actionRequest :datatype-) "stmotion_controller/robot_actionRequest")
(setf (get stmotion_controller::robot_actionRequest :definition-)
      "geometry_msgs/PoseStamped des
geometry_msgs/PoseStamped src

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
bool finished
")

(setf (get stmotion_controller::robot_actionResponse :md5sum-) "19a560c38819713863fdccb541e95a18")
(setf (get stmotion_controller::robot_actionResponse :datatype-) "stmotion_controller/robot_actionResponse")
(setf (get stmotion_controller::robot_actionResponse :definition-)
      "geometry_msgs/PoseStamped des
geometry_msgs/PoseStamped src

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
bool finished
")



(provide :stmotion_controller/robot_action "19a560c38819713863fdccb541e95a18")


