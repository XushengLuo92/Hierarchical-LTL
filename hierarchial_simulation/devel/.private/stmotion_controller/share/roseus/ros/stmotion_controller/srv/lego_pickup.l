;; Auto-generated. Do not edit!


(when (boundp 'stmotion_controller::lego_pickup)
  (if (not (find-package "STMOTION_CONTROLLER"))
    (make-package "STMOTION_CONTROLLER"))
  (shadow 'lego_pickup (find-package "STMOTION_CONTROLLER")))
(unless (find-package "STMOTION_CONTROLLER::LEGO_PICKUP")
  (make-package "STMOTION_CONTROLLER::LEGO_PICKUP"))
(unless (find-package "STMOTION_CONTROLLER::LEGO_PICKUPREQUEST")
  (make-package "STMOTION_CONTROLLER::LEGO_PICKUPREQUEST"))
(unless (find-package "STMOTION_CONTROLLER::LEGO_PICKUPRESPONSE")
  (make-package "STMOTION_CONTROLLER::LEGO_PICKUPRESPONSE"))

(in-package "ROS")





(defclass stmotion_controller::lego_pickupRequest
  :super ros::object
  :slots (_reference_frame _is_pick _pick_lego_name ))

(defmethod stmotion_controller::lego_pickupRequest
  (:init
   (&key
    ((:reference_frame __reference_frame) "")
    ((:is_pick __is_pick) "")
    ((:pick_lego_name __pick_lego_name) "")
    )
   (send-super :init)
   (setq _reference_frame (string __reference_frame))
   (setq _is_pick (string __is_pick))
   (setq _pick_lego_name (string __pick_lego_name))
   self)
  (:reference_frame
   (&optional __reference_frame)
   (if __reference_frame (setq _reference_frame __reference_frame)) _reference_frame)
  (:is_pick
   (&optional __is_pick)
   (if __is_pick (setq _is_pick __is_pick)) _is_pick)
  (:pick_lego_name
   (&optional __pick_lego_name)
   (if __pick_lego_name (setq _pick_lego_name __pick_lego_name)) _pick_lego_name)
  (:serialization-length
   ()
   (+
    ;; string _reference_frame
    4 (length _reference_frame)
    ;; string _is_pick
    4 (length _is_pick)
    ;; string _pick_lego_name
    4 (length _pick_lego_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _reference_frame
       (write-long (length _reference_frame) s) (princ _reference_frame s)
     ;; string _is_pick
       (write-long (length _is_pick) s) (princ _is_pick s)
     ;; string _pick_lego_name
       (write-long (length _pick_lego_name) s) (princ _pick_lego_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _reference_frame
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _reference_frame (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _is_pick
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _is_pick (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _pick_lego_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _pick_lego_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass stmotion_controller::lego_pickupResponse
  :super ros::object
  :slots (_finished ))

(defmethod stmotion_controller::lego_pickupResponse
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

(defclass stmotion_controller::lego_pickup
  :super ros::object
  :slots ())

(setf (get stmotion_controller::lego_pickup :md5sum-) "054657831fad2fea203e93682e8ffae0")
(setf (get stmotion_controller::lego_pickup :datatype-) "stmotion_controller/lego_pickup")
(setf (get stmotion_controller::lego_pickup :request) stmotion_controller::lego_pickupRequest)
(setf (get stmotion_controller::lego_pickup :response) stmotion_controller::lego_pickupResponse)

(defmethod stmotion_controller::lego_pickupRequest
  (:response () (instance stmotion_controller::lego_pickupResponse :init)))

(setf (get stmotion_controller::lego_pickupRequest :md5sum-) "054657831fad2fea203e93682e8ffae0")
(setf (get stmotion_controller::lego_pickupRequest :datatype-) "stmotion_controller/lego_pickupRequest")
(setf (get stmotion_controller::lego_pickupRequest :definition-)
      "string reference_frame
string is_pick
string pick_lego_name
---
bool finished
")

(setf (get stmotion_controller::lego_pickupResponse :md5sum-) "054657831fad2fea203e93682e8ffae0")
(setf (get stmotion_controller::lego_pickupResponse :datatype-) "stmotion_controller/lego_pickupResponse")
(setf (get stmotion_controller::lego_pickupResponse :definition-)
      "string reference_frame
string is_pick
string pick_lego_name
---
bool finished
")



(provide :stmotion_controller/lego_pickup "054657831fad2fea203e93682e8ffae0")


