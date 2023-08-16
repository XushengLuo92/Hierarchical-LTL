; Auto-generated. Do not edit!


(cl:in-package moveit_human_config-srv)


;//! \htmlinclude robot_action-request.msg.html

(cl:defclass <robot_action-request> (roslisp-msg-protocol:ros-message)
  ((des
    :reader des
    :initarg :des
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (src
    :reader src
    :initarg :src
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
)

(cl:defclass robot_action-request (<robot_action-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <robot_action-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'robot_action-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moveit_human_config-srv:<robot_action-request> is deprecated: use moveit_human_config-srv:robot_action-request instead.")))

(cl:ensure-generic-function 'des-val :lambda-list '(m))
(cl:defmethod des-val ((m <robot_action-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moveit_human_config-srv:des-val is deprecated.  Use moveit_human_config-srv:des instead.")
  (des m))

(cl:ensure-generic-function 'src-val :lambda-list '(m))
(cl:defmethod src-val ((m <robot_action-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moveit_human_config-srv:src-val is deprecated.  Use moveit_human_config-srv:src instead.")
  (src m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <robot_action-request>) ostream)
  "Serializes a message object of type '<robot_action-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'des) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'src) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <robot_action-request>) istream)
  "Deserializes a message object of type '<robot_action-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'des) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'src) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<robot_action-request>)))
  "Returns string type for a service object of type '<robot_action-request>"
  "moveit_human_config/robot_actionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_action-request)))
  "Returns string type for a service object of type 'robot_action-request"
  "moveit_human_config/robot_actionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<robot_action-request>)))
  "Returns md5sum for a message object of type '<robot_action-request>"
  "19a560c38819713863fdccb541e95a18")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'robot_action-request)))
  "Returns md5sum for a message object of type 'robot_action-request"
  "19a560c38819713863fdccb541e95a18")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<robot_action-request>)))
  "Returns full string definition for message of type '<robot_action-request>"
  (cl:format cl:nil "geometry_msgs/PoseStamped des~%geometry_msgs/PoseStamped src~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'robot_action-request)))
  "Returns full string definition for message of type 'robot_action-request"
  (cl:format cl:nil "geometry_msgs/PoseStamped des~%geometry_msgs/PoseStamped src~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <robot_action-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'des))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'src))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <robot_action-request>))
  "Converts a ROS message object to a list"
  (cl:list 'robot_action-request
    (cl:cons ':des (des msg))
    (cl:cons ':src (src msg))
))
;//! \htmlinclude robot_action-response.msg.html

(cl:defclass <robot_action-response> (roslisp-msg-protocol:ros-message)
  ((finished
    :reader finished
    :initarg :finished
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass robot_action-response (<robot_action-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <robot_action-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'robot_action-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name moveit_human_config-srv:<robot_action-response> is deprecated: use moveit_human_config-srv:robot_action-response instead.")))

(cl:ensure-generic-function 'finished-val :lambda-list '(m))
(cl:defmethod finished-val ((m <robot_action-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader moveit_human_config-srv:finished-val is deprecated.  Use moveit_human_config-srv:finished instead.")
  (finished m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <robot_action-response>) ostream)
  "Serializes a message object of type '<robot_action-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'finished) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <robot_action-response>) istream)
  "Deserializes a message object of type '<robot_action-response>"
    (cl:setf (cl:slot-value msg 'finished) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<robot_action-response>)))
  "Returns string type for a service object of type '<robot_action-response>"
  "moveit_human_config/robot_actionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_action-response)))
  "Returns string type for a service object of type 'robot_action-response"
  "moveit_human_config/robot_actionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<robot_action-response>)))
  "Returns md5sum for a message object of type '<robot_action-response>"
  "19a560c38819713863fdccb541e95a18")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'robot_action-response)))
  "Returns md5sum for a message object of type 'robot_action-response"
  "19a560c38819713863fdccb541e95a18")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<robot_action-response>)))
  "Returns full string definition for message of type '<robot_action-response>"
  (cl:format cl:nil "bool finished~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'robot_action-response)))
  "Returns full string definition for message of type 'robot_action-response"
  (cl:format cl:nil "bool finished~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <robot_action-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <robot_action-response>))
  "Converts a ROS message object to a list"
  (cl:list 'robot_action-response
    (cl:cons ':finished (finished msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'robot_action)))
  'robot_action-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'robot_action)))
  'robot_action-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_action)))
  "Returns string type for a service object of type '<robot_action>"
  "moveit_human_config/robot_action")