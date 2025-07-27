;; Auto-generated. Do not edit!


(when (boundp 'joy_andro::JoyAndro)
  (if (not (find-package "JOY_ANDRO"))
    (make-package "JOY_ANDRO"))
  (shadow 'JoyAndro (find-package "JOY_ANDRO")))
(unless (find-package "JOY_ANDRO::JOYANDRO")
  (make-package "JOY_ANDRO::JOYANDRO"))

(in-package "ROS")
;;//! \htmlinclude JoyAndro.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass joy_andro::JoyAndro
  :super ros::object
  :slots (_header _x _y _button_x _button_circle _button_triangle _button_square ))

(defmethod joy_andro::JoyAndro
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:button_x __button_x) nil)
    ((:button_circle __button_circle) nil)
    ((:button_triangle __button_triangle) nil)
    ((:button_square __button_square) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _button_x __button_x)
   (setq _button_circle __button_circle)
   (setq _button_triangle __button_triangle)
   (setq _button_square __button_square)
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
  (:button_x
   (&optional (__button_x :null))
   (if (not (eq __button_x :null)) (setq _button_x __button_x)) _button_x)
  (:button_circle
   (&optional (__button_circle :null))
   (if (not (eq __button_circle :null)) (setq _button_circle __button_circle)) _button_circle)
  (:button_triangle
   (&optional (__button_triangle :null))
   (if (not (eq __button_triangle :null)) (setq _button_triangle __button_triangle)) _button_triangle)
  (:button_square
   (&optional (__button_square :null))
   (if (not (eq __button_square :null)) (setq _button_square __button_square)) _button_square)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; bool _button_x
    1
    ;; bool _button_circle
    1
    ;; bool _button_triangle
    1
    ;; bool _button_square
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _button_x
       (if _button_x (write-byte -1 s) (write-byte 0 s))
     ;; bool _button_circle
       (if _button_circle (write-byte -1 s) (write-byte 0 s))
     ;; bool _button_triangle
       (if _button_triangle (write-byte -1 s) (write-byte 0 s))
     ;; bool _button_square
       (if _button_square (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _button_x
     (setq _button_x (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button_circle
     (setq _button_circle (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button_triangle
     (setq _button_triangle (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _button_square
     (setq _button_square (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get joy_andro::JoyAndro :md5sum-) "4772fa7b3815bf94f88ced97e695f175")
(setf (get joy_andro::JoyAndro :datatype-) "joy_andro/JoyAndro")
(setf (get joy_andro::JoyAndro :definition-)
      "std_msgs/Header header
float32 x
float32 y
bool button_x
bool button_circle
bool button_triangle
bool button_square
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
string frame_id

")



(provide :joy_andro/JoyAndro "4772fa7b3815bf94f88ced97e695f175")


