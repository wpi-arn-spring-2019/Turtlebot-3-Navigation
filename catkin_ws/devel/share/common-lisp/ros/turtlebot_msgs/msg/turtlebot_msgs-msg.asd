
(cl:in-package :asdf)

(defsystem "turtlebot_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "GoalPose" :depends-on ("_package_GoalPose"))
    (:file "_package_GoalPose" :depends-on ("_package"))
    (:file "Trajectory" :depends-on ("_package_Trajectory"))
    (:file "_package_Trajectory" :depends-on ("_package"))
  ))