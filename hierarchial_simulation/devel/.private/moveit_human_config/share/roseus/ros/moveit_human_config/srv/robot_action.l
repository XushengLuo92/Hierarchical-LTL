;; Auto-generated. Do not edit!


(when (boundp 'moveit_human_config::robot_action)
  (if (not (find-package "MOVEIT_HUMAN_CONFIG"))
    (make-package "MOVEIT_HUMAN_CONFIG"))
  (shadow 'robot_action (find-package "MOVEIT_HUMAN_CONFIG")))
(unless (find-package "MOVEIT_HUMAN_CONFIG::ROBOT_ACTION")
  (make-package "MOVEIT_HUMAN_CONFIG::ROBOT_ACTION"))
(unless (find-package "MOVEIT_HUMAN_CONFIG::ROBOT_ACTIONREQUEST")
  (make-package "MOVEIT_HUMAN_CONFIG::ROBOT_ACTIONREQUEST"))
(unless (find-package "MOVEIT_HUMAN_CONFIG::ROBOT_ACTIONRESPONSE")
  (make-package "MOVEIT_HUMAN_CONFIG::ROBOT_ACTIONRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass moveit_human_config::robot_actionRequest
  :super ros::object
  :slots (_des _src ))

(defmethod moveit_human_config::robot_actionRequest
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

(defclass moveit_human_config::robot_actionResponse
  :super ros::object
  :slots (_finished ))

(defmethod moveit_human_config::robot_actionResponse
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

(defclass moveit_human_config::robot_action
  :super ros::object
  :slots ())

(setf (get moveit_human_config::robot_action :md5sum-) "19a560c38819713863fdccb541e95a18")
(setf (get moveit_human_config::robot_action :datatype-) "moveit_human_config/robot_action")
(setf (get moveit_human_config::robot_action :request) moveit_human_config::robot_actionRequest)
(setf (get moveit_human_config::robot_action :response) moveit_human_config::robot_actionResponse)

(defmethod moveit_human_config::robot_actionRequest
  (:response () (instance moveit_human_config::robot_actionResponse :init)))

(setf (get moveit_human_config::robot_actionRequest :md5sum-) "19a560c38819713863fdccb541e95a18")
(setf (get moveit_human_config::robot_actionRequest :datatype-) "moveit_human_config/robot_actionRequest")
(setf (get moveit_human_config::robot_actionRequest :definition-)
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

(setf (get moveit_human_config::robot_actionResponse :md5sum-) "19a560c38819713863fdccb541e95a18")
(setf (get moveit_human_config::robot_actionResponse :datatype-) "moveit_human_config/robot_actionResponse")
(setf (get moveit_human_config::robot_actionResponse :definition-)
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



(provide :moveit_human_config/robot_action "19a560c38819713863fdccb541e95a18")


