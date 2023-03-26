
(cl:in-package :asdf)

(defsystem "jetson-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "NED_acoustic" :depends-on ("_package_NED_acoustic"))
    (:file "_package_NED_acoustic" :depends-on ("_package"))
    (:file "NED_objects" :depends-on ("_package_NED_objects"))
    (:file "_package_NED_objects" :depends-on ("_package"))
    (:file "NED_poses" :depends-on ("_package_NED_poses"))
    (:file "_package_NED_poses" :depends-on ("_package"))
    (:file "NED_waypoints" :depends-on ("_package_NED_waypoints"))
    (:file "_package_NED_waypoints" :depends-on ("_package"))
    (:file "Task" :depends-on ("_package_Task"))
    (:file "_package_Task" :depends-on ("_package"))
    (:file "control_efforts" :depends-on ("_package_control_efforts"))
    (:file "_package_control_efforts" :depends-on ("_package"))
    (:file "motorStatus_" :depends-on ("_package_motorStatus_"))
    (:file "_package_motorStatus_" :depends-on ("_package"))
    (:file "propulsion_system" :depends-on ("_package_propulsion_system"))
    (:file "_package_propulsion_system" :depends-on ("_package"))
    (:file "state" :depends-on ("_package_state"))
    (:file "_package_state" :depends-on ("_package"))
  ))