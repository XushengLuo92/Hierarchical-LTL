; Auto-generated. Do not edit!


(cl:in-package hierarchial_simulation-srv)


;//! \htmlinclude lego_pickup-request.msg.html

(cl:defclass <lego_pickup-request> (roslisp-msg-protocol:ros-message)
  ((reference_frame
    :reader reference_frame
    :initarg :reference_frame
    :type cl:string
    :initform "")
   (is_pick
    :reader is_pick
    :initarg :is_pick
    :type cl:string
    :initform "")
   (pick_lego_name
    :reader pick_lego_name
    :initarg :pick_lego_name
    :type cl:string
    :initform ""))
)

(cl:defclass lego_pickup-request (<lego_pickup-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <lego_pickup-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'lego_pickup-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hierarchial_simulation-srv:<lego_pickup-request> is deprecated: use hierarchial_simulation-srv:lego_pickup-request instead.")))

(cl:ensure-generic-function 'reference_frame-val :lambda-list '(m))
(cl:defmethod reference_frame-val ((m <lego_pickup-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hierarchial_simulation-srv:reference_frame-val is deprecated.  Use hierarchial_simulation-srv:reference_frame instead.")
  (reference_frame m))

(cl:ensure-generic-function 'is_pick-val :lambda-list '(m))
(cl:defmethod is_pick-val ((m <lego_pickup-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hierarchial_simulation-srv:is_pick-val is deprecated.  Use hierarchial_simulation-srv:is_pick instead.")
  (is_pick m))

(cl:ensure-generic-function 'pick_lego_name-val :lambda-list '(m))
(cl:defmethod pick_lego_name-val ((m <lego_pickup-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hierarchial_simulation-srv:pick_lego_name-val is deprecated.  Use hierarchial_simulation-srv:pick_lego_name instead.")
  (pick_lego_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <lego_pickup-request>) ostream)
  "Serializes a message object of type '<lego_pickup-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reference_frame))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reference_frame))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'is_pick))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'is_pick))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'pick_lego_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'pick_lego_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <lego_pickup-request>) istream)
  "Deserializes a message object of type '<lego_pickup-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'reference_frame) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'reference_frame) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'is_pick) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'is_pick) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pick_lego_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'pick_lego_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<lego_pickup-request>)))
  "Returns string type for a service object of type '<lego_pickup-request>"
  "hierarchial_simulation/lego_pickupRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lego_pickup-request)))
  "Returns string type for a service object of type 'lego_pickup-request"
  "hierarchial_simulation/lego_pickupRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<lego_pickup-request>)))
  "Returns md5sum for a message object of type '<lego_pickup-request>"
  "054657831fad2fea203e93682e8ffae0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'lego_pickup-request)))
  "Returns md5sum for a message object of type 'lego_pickup-request"
  "054657831fad2fea203e93682e8ffae0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<lego_pickup-request>)))
  "Returns full string definition for message of type '<lego_pickup-request>"
  (cl:format cl:nil "string reference_frame~%string is_pick~%string pick_lego_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'lego_pickup-request)))
  "Returns full string definition for message of type 'lego_pickup-request"
  (cl:format cl:nil "string reference_frame~%string is_pick~%string pick_lego_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <lego_pickup-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'reference_frame))
     4 (cl:length (cl:slot-value msg 'is_pick))
     4 (cl:length (cl:slot-value msg 'pick_lego_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <lego_pickup-request>))
  "Converts a ROS message object to a list"
  (cl:list 'lego_pickup-request
    (cl:cons ':reference_frame (reference_frame msg))
    (cl:cons ':is_pick (is_pick msg))
    (cl:cons ':pick_lego_name (pick_lego_name msg))
))
;//! \htmlinclude lego_pickup-response.msg.html

(cl:defclass <lego_pickup-response> (roslisp-msg-protocol:ros-message)
  ((finished
    :reader finished
    :initarg :finished
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass lego_pickup-response (<lego_pickup-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <lego_pickup-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'lego_pickup-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hierarchial_simulation-srv:<lego_pickup-response> is deprecated: use hierarchial_simulation-srv:lego_pickup-response instead.")))

(cl:ensure-generic-function 'finished-val :lambda-list '(m))
(cl:defmethod finished-val ((m <lego_pickup-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hierarchial_simulation-srv:finished-val is deprecated.  Use hierarchial_simulation-srv:finished instead.")
  (finished m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <lego_pickup-response>) ostream)
  "Serializes a message object of type '<lego_pickup-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'finished) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <lego_pickup-response>) istream)
  "Deserializes a message object of type '<lego_pickup-response>"
    (cl:setf (cl:slot-value msg 'finished) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<lego_pickup-response>)))
  "Returns string type for a service object of type '<lego_pickup-response>"
  "hierarchial_simulation/lego_pickupResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lego_pickup-response)))
  "Returns string type for a service object of type 'lego_pickup-response"
  "hierarchial_simulation/lego_pickupResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<lego_pickup-response>)))
  "Returns md5sum for a message object of type '<lego_pickup-response>"
  "054657831fad2fea203e93682e8ffae0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'lego_pickup-response)))
  "Returns md5sum for a message object of type 'lego_pickup-response"
  "054657831fad2fea203e93682e8ffae0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<lego_pickup-response>)))
  "Returns full string definition for message of type '<lego_pickup-response>"
  (cl:format cl:nil "bool finished~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'lego_pickup-response)))
  "Returns full string definition for message of type 'lego_pickup-response"
  (cl:format cl:nil "bool finished~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <lego_pickup-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <lego_pickup-response>))
  "Converts a ROS message object to a list"
  (cl:list 'lego_pickup-response
    (cl:cons ':finished (finished msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'lego_pickup)))
  'lego_pickup-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'lego_pickup)))
  'lego_pickup-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lego_pickup)))
  "Returns string type for a service object of type '<lego_pickup>"
  "hierarchial_simulation/lego_pickup")