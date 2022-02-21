
(cl:in-package :asdf)

(defsystem "amore-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "NED_waypoints" :depends-on ("_package_NED_waypoints"))
    (:file "_package_NED_waypoints" :depends-on ("_package"))
  ))