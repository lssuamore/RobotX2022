; Auto-generated. Do not edit!


(cl:in-package jetson-msg)


;//! \htmlinclude task_info.msg.html

(cl:defclass <task_info> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (state
    :reader state
    :initarg :state
    :type cl:string
    :initform ""))
)

(cl:defclass task_info (<task_info>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <task_info>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'task_info)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jetson-msg:<task_info> is deprecated: use jetson-msg:task_info instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <task_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jetson-msg:name-val is deprecated.  Use jetson-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <task_info>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jetson-msg:state-val is deprecated.  Use jetson-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <task_info>) ostream)
  "Serializes a message object of type '<task_info>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'state))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <task_info>) istream)
  "Deserializes a message object of type '<task_info>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<task_info>)))
  "Returns string type for a message object of type '<task_info>"
  "jetson/task_info")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'task_info)))
  "Returns string type for a message object of type 'task_info"
  "jetson/task_info")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<task_info>)))
  "Returns md5sum for a message object of type '<task_info>"
  "181acc0441b0be709bbbbfe6dba51bd0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'task_info)))
  "Returns md5sum for a message object of type 'task_info"
  "181acc0441b0be709bbbbfe6dba51bd0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<task_info>)))
  "Returns full string definition for message of type '<task_info>"
  (cl:format cl:nil "string name~%string state~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'task_info)))
  "Returns full string definition for message of type 'task_info"
  (cl:format cl:nil "string name~%string state~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <task_info>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <task_info>))
  "Converts a ROS message object to a list"
  (cl:list 'task_info
    (cl:cons ':name (name msg))
    (cl:cons ':state (state msg))
))
