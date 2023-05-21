;; Auto-generated. Do not edit!


(when (boundp 'jetson::task_info)
  (if (not (find-package "JETSON"))
    (make-package "JETSON"))
  (shadow 'task_info (find-package "JETSON")))
(unless (find-package "JETSON::TASK_INFO")
  (make-package "JETSON::TASK_INFO"))

(in-package "ROS")
;;//! \htmlinclude task_info.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jetson::task_info
  :super ros::object
  :slots (_name _state ))

(defmethod jetson::task_info
  (:init
   (&key
    ((:name __name) (instance std_msgs::Int64 :init))
    ((:state __state) "")
    )
   (send-super :init)
   (setq _name __name)
   (setq _state (string __state))
   self)
  (:name
   (&rest __name)
   (if (keywordp (car __name))
       (send* _name __name)
     (progn
       (if __name (setq _name (car __name)))
       _name)))
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Int64 _name
    (send _name :serialization-length)
    ;; string _state
    4 (length _state)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Int64 _name
       (send _name :serialize s)
     ;; string _state
       (write-long (length _state) s) (princ _state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Int64 _name
     (send _name :deserialize buf ptr-) (incf ptr- (send _name :serialization-length))
   ;; string _state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get jetson::task_info :md5sum-) "8604b4549acbc2637d40681d3885d9ee")
(setf (get jetson::task_info :datatype-) "jetson/task_info")
(setf (get jetson::task_info :definition-)
      "std_msgs/Int64 name
string state


================================================================================
MSG: std_msgs/Int64
int64 data
")



(provide :jetson/task_info "8604b4549acbc2637d40681d3885d9ee")


