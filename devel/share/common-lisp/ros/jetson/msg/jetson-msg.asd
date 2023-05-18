
(cl:in-package :asdf)

(defsystem "jetson-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AMS_state" :depends-on ("_package_AMS_state"))
    (:file "_package_AMS_state" :depends-on ("_package"))
    (:file "Acoustics_msg" :depends-on ("_package_Acoustics_msg"))
    (:file "_package_Acoustics_msg" :depends-on ("_package"))
    (:file "Detect_Dock_Fling" :depends-on ("_package_Detect_Dock_Fling"))
    (:file "_package_Detect_Dock_Fling" :depends-on ("_package"))
    (:file "NED_objects" :depends-on ("_package_NED_objects"))
    (:file "_package_NED_objects" :depends-on ("_package"))
    (:file "NED_waypoints" :depends-on ("_package_NED_waypoints"))
    (:file "_package_NED_waypoints" :depends-on ("_package"))
    (:file "control_efforts" :depends-on ("_package_control_efforts"))
    (:file "_package_control_efforts" :depends-on ("_package"))
    (:file "state_msg" :depends-on ("_package_state_msg"))
    (:file "_package_state_msg" :depends-on ("_package"))
    (:file "task_info" :depends-on ("_package_task_info"))
    (:file "_package_task_info" :depends-on ("_package"))
    (:file "usv_pose_msg" :depends-on ("_package_usv_pose_msg"))
    (:file "_package_usv_pose_msg" :depends-on ("_package"))
    (:file "zed2i_msg" :depends-on ("_package_zed2i_msg"))
    (:file "_package_zed2i_msg" :depends-on ("_package"))
  ))