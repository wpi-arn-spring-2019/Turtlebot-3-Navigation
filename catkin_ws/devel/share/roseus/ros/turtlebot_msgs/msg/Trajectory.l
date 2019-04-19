;; Auto-generated. Do not edit!


(when (boundp 'turtlebot_msgs::Trajectory)
  (if (not (find-package "TURTLEBOT_MSGS"))
    (make-package "TURTLEBOT_MSGS"))
  (shadow 'Trajectory (find-package "TURTLEBOT_MSGS")))
(unless (find-package "TURTLEBOT_MSGS::TRAJECTORY")
  (make-package "TURTLEBOT_MSGS::TRAJECTORY"))

(in-package "ROS")
;;//! \htmlinclude Trajectory.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass turtlebot_msgs::Trajectory
  :super ros::object
  :slots (_header _x_values _y_values _headings _yaw_rates _durations _speeds _max_speed _max_accel ))

(defmethod turtlebot_msgs::Trajectory
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:x_values __x_values) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:y_values __y_values) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:headings __headings) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:yaw_rates __yaw_rates) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:durations __durations) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:speeds __speeds) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:max_speed __max_speed) 0.0)
    ((:max_accel __max_accel) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _x_values __x_values)
   (setq _y_values __y_values)
   (setq _headings __headings)
   (setq _yaw_rates __yaw_rates)
   (setq _durations __durations)
   (setq _speeds __speeds)
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
  (:x_values
   (&optional __x_values)
   (if __x_values (setq _x_values __x_values)) _x_values)
  (:y_values
   (&optional __y_values)
   (if __y_values (setq _y_values __y_values)) _y_values)
  (:headings
   (&optional __headings)
   (if __headings (setq _headings __headings)) _headings)
  (:yaw_rates
   (&optional __yaw_rates)
   (if __yaw_rates (setq _yaw_rates __yaw_rates)) _yaw_rates)
  (:durations
   (&optional __durations)
   (if __durations (setq _durations __durations)) _durations)
  (:speeds
   (&optional __speeds)
   (if __speeds (setq _speeds __speeds)) _speeds)
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
    ;; float64[] _x_values
    (* 8    (length _x_values)) 4
    ;; float64[] _y_values
    (* 8    (length _y_values)) 4
    ;; float64[] _headings
    (* 8    (length _headings)) 4
    ;; float64[] _yaw_rates
    (* 8    (length _yaw_rates)) 4
    ;; float64[] _durations
    (* 8    (length _durations)) 4
    ;; float64[] _speeds
    (* 8    (length _speeds)) 4
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
     ;; float64[] _x_values
     (write-long (length _x_values) s)
     (dotimes (i (length _x_values))
       (sys::poke (elt _x_values i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _y_values
     (write-long (length _y_values) s)
     (dotimes (i (length _y_values))
       (sys::poke (elt _y_values i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _headings
     (write-long (length _headings) s)
     (dotimes (i (length _headings))
       (sys::poke (elt _headings i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _yaw_rates
     (write-long (length _yaw_rates) s)
     (dotimes (i (length _yaw_rates))
       (sys::poke (elt _yaw_rates i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _durations
     (write-long (length _durations) s)
     (dotimes (i (length _durations))
       (sys::poke (elt _durations i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _speeds
     (write-long (length _speeds) s)
     (dotimes (i (length _speeds))
       (sys::poke (elt _speeds i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
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
   ;; float64[] _x_values
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _x_values (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _x_values i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _y_values
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _y_values (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _y_values i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _headings
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _headings (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _headings i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _yaw_rates
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _yaw_rates (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _yaw_rates i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _durations
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _durations (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _durations i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _speeds
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _speeds (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _speeds i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64 _max_speed
     (setq _max_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _max_accel
     (setq _max_accel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get turtlebot_msgs::Trajectory :md5sum-) "110b624ce51252d0ba8b1a212a825dd2")
(setf (get turtlebot_msgs::Trajectory :datatype-) "turtlebot_msgs/Trajectory")
(setf (get turtlebot_msgs::Trajectory :definition-)
      "Header header

float64[] x_values
float64[] y_values
float64[] headings
float64[] yaw_rates
float64[] durations
float64[] speeds
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



(provide :turtlebot_msgs/Trajectory "110b624ce51252d0ba8b1a212a825dd2")


