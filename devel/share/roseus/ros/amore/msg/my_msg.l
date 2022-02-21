;; Auto-generated. Do not edit!


(when (boundp 'amore::my_msg)
  (if (not (find-package "AMORE"))
    (make-package "AMORE"))
  (shadow 'my_msg (find-package "AMORE")))
(unless (find-package "AMORE::MY_MSG")
  (make-package "AMORE::MY_MSG"))

(in-package "ROS")
;;//! \htmlinclude my_msg.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass amore::my_msg
  :super ros::object
  :slots (_points _psi ))

(defmethod amore::my_msg
  (:init
   (&key
    ((:points __points) ())
    ((:psi __psi) 0.0)
    )
   (send-super :init)
   (setq _points __points)
   (setq _psi (float __psi))
   self)
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:psi
   (&optional __psi)
   (if __psi (setq _psi __psi)) _psi)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ;; float64 _psi
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;; float64 _psi
       (sys::poke _psi (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float64 _psi
     (setq _psi (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get amore::my_msg :md5sum-) "64555233587211f62e3a55ce9242df37")
(setf (get amore::my_msg :datatype-) "amore/my_msg")
(setf (get amore::my_msg :definition-)
      "geometry_msgs/Point[] points
float64 psi

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :amore/my_msg "64555233587211f62e3a55ce9242df37")


