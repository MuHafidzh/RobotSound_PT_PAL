; Auto-generated. Do not edit!


(cl:in-package joy_andro-msg)


;//! \htmlinclude JoyAndro.msg.html

(cl:defclass <JoyAndro> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (button_x
    :reader button_x
    :initarg :button_x
    :type cl:boolean
    :initform cl:nil)
   (button_circle
    :reader button_circle
    :initarg :button_circle
    :type cl:boolean
    :initform cl:nil)
   (button_triangle
    :reader button_triangle
    :initarg :button_triangle
    :type cl:boolean
    :initform cl:nil)
   (button_square
    :reader button_square
    :initarg :button_square
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass JoyAndro (<JoyAndro>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JoyAndro>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JoyAndro)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name joy_andro-msg:<JoyAndro> is deprecated: use joy_andro-msg:JoyAndro instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <JoyAndro>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joy_andro-msg:header-val is deprecated.  Use joy_andro-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <JoyAndro>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joy_andro-msg:x-val is deprecated.  Use joy_andro-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <JoyAndro>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joy_andro-msg:y-val is deprecated.  Use joy_andro-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'button_x-val :lambda-list '(m))
(cl:defmethod button_x-val ((m <JoyAndro>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joy_andro-msg:button_x-val is deprecated.  Use joy_andro-msg:button_x instead.")
  (button_x m))

(cl:ensure-generic-function 'button_circle-val :lambda-list '(m))
(cl:defmethod button_circle-val ((m <JoyAndro>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joy_andro-msg:button_circle-val is deprecated.  Use joy_andro-msg:button_circle instead.")
  (button_circle m))

(cl:ensure-generic-function 'button_triangle-val :lambda-list '(m))
(cl:defmethod button_triangle-val ((m <JoyAndro>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joy_andro-msg:button_triangle-val is deprecated.  Use joy_andro-msg:button_triangle instead.")
  (button_triangle m))

(cl:ensure-generic-function 'button_square-val :lambda-list '(m))
(cl:defmethod button_square-val ((m <JoyAndro>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joy_andro-msg:button_square-val is deprecated.  Use joy_andro-msg:button_square instead.")
  (button_square m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JoyAndro>) ostream)
  "Serializes a message object of type '<JoyAndro>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button_x) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button_circle) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button_triangle) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'button_square) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JoyAndro>) istream)
  "Deserializes a message object of type '<JoyAndro>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'button_x) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button_circle) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button_triangle) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'button_square) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JoyAndro>)))
  "Returns string type for a message object of type '<JoyAndro>"
  "joy_andro/JoyAndro")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JoyAndro)))
  "Returns string type for a message object of type 'JoyAndro"
  "joy_andro/JoyAndro")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JoyAndro>)))
  "Returns md5sum for a message object of type '<JoyAndro>"
  "4772fa7b3815bf94f88ced97e695f175")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JoyAndro)))
  "Returns md5sum for a message object of type 'JoyAndro"
  "4772fa7b3815bf94f88ced97e695f175")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JoyAndro>)))
  "Returns full string definition for message of type '<JoyAndro>"
  (cl:format cl:nil "std_msgs/Header header~%float32 x~%float32 y~%bool button_x~%bool button_circle~%bool button_triangle~%bool button_square~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JoyAndro)))
  "Returns full string definition for message of type 'JoyAndro"
  (cl:format cl:nil "std_msgs/Header header~%float32 x~%float32 y~%bool button_x~%bool button_circle~%bool button_triangle~%bool button_square~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JoyAndro>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JoyAndro>))
  "Converts a ROS message object to a list"
  (cl:list 'JoyAndro
    (cl:cons ':header (header msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':button_x (button_x msg))
    (cl:cons ':button_circle (button_circle msg))
    (cl:cons ':button_triangle (button_triangle msg))
    (cl:cons ':button_square (button_square msg))
))
