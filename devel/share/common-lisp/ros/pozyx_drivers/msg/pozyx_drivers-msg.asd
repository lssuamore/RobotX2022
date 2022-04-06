
(cl:in-package :asdf)

(defsystem "pozyx_drivers-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AnchorInfo" :depends-on ("_package_AnchorInfo"))
    (:file "_package_AnchorInfo" :depends-on ("_package"))
  ))