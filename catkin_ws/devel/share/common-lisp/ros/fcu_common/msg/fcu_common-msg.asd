
(cl:in-package :asdf)

(defsystem "fcu_common-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "MR_Controller_Commands" :depends-on ("_package_MR_Controller_Commands"))
    (:file "_package_MR_Controller_Commands" :depends-on ("_package"))
    (:file "FW_State" :depends-on ("_package_FW_State"))
    (:file "_package_FW_State" :depends-on ("_package"))
    (:file "FW_Controller_Commands" :depends-on ("_package_FW_Controller_Commands"))
    (:file "_package_FW_Controller_Commands" :depends-on ("_package"))
    (:file "FW_Current_Path" :depends-on ("_package_FW_Current_Path"))
    (:file "_package_FW_Current_Path" :depends-on ("_package"))
    (:file "Command" :depends-on ("_package_Command"))
    (:file "_package_Command" :depends-on ("_package"))
    (:file "GPS" :depends-on ("_package_GPS"))
    (:file "_package_GPS" :depends-on ("_package"))
    (:file "FW_Waypoint" :depends-on ("_package_FW_Waypoint"))
    (:file "_package_FW_Waypoint" :depends-on ("_package"))
  ))