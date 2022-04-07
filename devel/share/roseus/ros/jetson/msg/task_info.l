;; Auto-generated. Do not edit!


(when (boundp 'jetson::task_info)
  (if (not (find-package "JETSON"))
    (make-package "JETSON"))
  (shadow 'task_info (find-package "JETSON")))
(unless (find-package "JETSON::TASK_INFO")
  (make-package "JETSON::TASK_INFO"))

(in-package "ROS")
;;//! \htmlinclude task_info.msg.html


(defclass jetson::task_info
  :super ros::object
  :slots (_name _state ))

(defmethod jetson::task_info
  (:init
   (&key
    ((:name __name) "")
    ((:state __state) "")
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _state (string __state))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; string _state
    4 (length _state)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _state
       (write-long (length _state) s) (princ _state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get jetson::task_info :md5sum-) "181acc0441b0be709bbbbfe6dba51bd0")
(setf (get jetson::task_info :datatype-) "jetson/task_info")
(setf (get jetson::task_info :definition-)
      "string name
string state


")



(provide :jetson/task_info "181acc0441b0be709bbbbfe6dba51bd0")


