;; Auto-generated. Do not edit!


(when (boundp 'turtlebot_msgs::GoalPose)
  (if (not (find-package "TURTLEBOT_MSGS"))
    (make-package "TURTLEBOT_MSGS"))
  (shadow 'GoalPose (find-package "TURTLEBOT_MSGS")))
(unless (find-package "TURTLEBOT_MSGS::GOALPOSE")
  (make-package "TURTLEBOT_MSGS::GOALPOSE"))

(in-package "ROS")
;;//! \htmlinclude GoalPose.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass turtlebot_msgs::GoalPose
  :super ros::object
  :slots (_header _x _y _speed _heading _max_speed _max_accel ))

(defmethod turtlebot_msgs::GoalPose
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:speed __speed) 0.0)
    ((:heading __heading) 0.0)
    ((:max_speed __max_speed) 0.0)
    ((:max_accel __max_accel) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _speed (float __speed))
   (setq _heading (float __heading))
   (setq _max_speed (float __max_speed))
   (setq _max_accel (float __max_accel))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:heading
   (&optional __heading)
   (if __heading (setq _heading __heading)) _heading)
  (:max_speed
   (&optional __max_speed)
   (if __max_speed (setq _max_speed __max_speed)) _max_speed)
  (:max_accel
   (&optional __max_accel)
   (if __max_accel (setq _max_accel __max_accel)) _max_accel)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _speed
    8
    ;; float64 _heading
    8
    ;; float64 _max_speed
    8
    ;; float64 _max_accel
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _heading
       (sys::poke _heading (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _max_speed
       (sys::poke _max_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _max_accel
       (sys::poke _max_accel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _speed
     (setq _speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _heading
     (setq _heading (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _max_speed
     (setq _max_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _max_accel
     (setq _max_accel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get turtlebot_msgs::GoalPose :md5sum-) "b41ed743e7940944d509142a63f050aa")
(setf (get turtlebot_msgs::GoalPose :datatype-) "turtlebot_msgs/GoalPose")
(setf (get turtlebot_msgs::GoalPose :definition-)
      "Header header

float64 x
float64 y
float64 speed
float64 heading
float64 max_speed
float64 max_accel

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :turtlebot_msgs/GoalPose "b41ed743e7940944d509142a63f050aa")


