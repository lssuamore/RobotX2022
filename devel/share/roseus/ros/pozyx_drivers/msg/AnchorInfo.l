;; Auto-generated. Do not edit!


(when (boundp 'pozyx_drivers::AnchorInfo)
  (if (not (find-package "POZYX_DRIVERS"))
    (make-package "POZYX_DRIVERS"))
  (shadow 'AnchorInfo (find-package "POZYX_DRIVERS")))
(unless (find-package "POZYX_DRIVERS::ANCHORINFO")
  (make-package "POZYX_DRIVERS::ANCHORINFO"))

(in-package "ROS")
;;//! \htmlinclude AnchorInfo.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pozyx_drivers::AnchorInfo
  :super ros::object
  :slots (_header _child_frame_id _id _status _position _position_cov _distance _distance_cov _RSS ))

(defmethod pozyx_drivers::AnchorInfo
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:child_frame_id __child_frame_id) "")
    ((:id __id) "")
    ((:status __status) nil)
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:position_cov __position_cov) (make-array 9 :initial-element 0.0 :element-type :float))
    ((:distance __distance) 0.0)
    ((:distance_cov __distance_cov) 0.0)
    ((:RSS __RSS) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _child_frame_id (string __child_frame_id))
   (setq _id (string __id))
   (setq _status __status)
   (setq _position __position)
   (setq _position_cov __position_cov)
   (setq _distance (float __distance))
   (setq _distance_cov (float __distance_cov))
   (setq _RSS (round __RSS))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:child_frame_id
   (&optional __child_frame_id)
   (if __child_frame_id (setq _child_frame_id __child_frame_id)) _child_frame_id)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:position_cov
   (&optional __position_cov)
   (if __position_cov (setq _position_cov __position_cov)) _position_cov)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:distance_cov
   (&optional __distance_cov)
   (if __distance_cov (setq _distance_cov __distance_cov)) _distance_cov)
  (:RSS
   (&optional __RSS)
   (if __RSS (setq _RSS __RSS)) _RSS)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _child_frame_id
    4 (length _child_frame_id)
    ;; string _id
    4 (length _id)
    ;; bool _status
    1
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; float64[9] _position_cov
    (* 8    9)
    ;; float64 _distance
    8
    ;; float64 _distance_cov
    8
    ;; int16 _RSS
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _child_frame_id
       (write-long (length _child_frame_id) s) (princ _child_frame_id s)
     ;; string _id
       (write-long (length _id) s) (princ _id s)
     ;; bool _status
       (if _status (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; float64[9] _position_cov
     (dotimes (i 9)
       (sys::poke (elt _position_cov i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _distance_cov
       (sys::poke _distance_cov (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int16 _RSS
       (write-word _RSS s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _child_frame_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _child_frame_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _status
     (setq _status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; float64[9] _position_cov
   (dotimes (i (length _position_cov))
     (setf (elt _position_cov i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64 _distance
     (setq _distance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _distance_cov
     (setq _distance_cov (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int16 _RSS
     (setq _RSS (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get pozyx_drivers::AnchorInfo :md5sum-) "325b1e2a6e1b43f05d9487e491169f3b")
(setf (get pozyx_drivers::AnchorInfo :datatype-) "pozyx_drivers/AnchorInfo")
(setf (get pozyx_drivers::AnchorInfo :definition-)
      "Header header
string child_frame_id
string id
bool status
geometry_msgs/Point position
float64[9] position_cov
float64 distance
float64 distance_cov
int16 RSS
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
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :pozyx_drivers/AnchorInfo "325b1e2a6e1b43f05d9487e491169f3b")


