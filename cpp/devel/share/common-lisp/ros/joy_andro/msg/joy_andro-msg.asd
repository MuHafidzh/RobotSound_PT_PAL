
(cl:in-package :asdf)

(defsystem "joy_andro-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "JoyAndro" :depends-on ("_package_JoyAndro"))
    (:file "_package_JoyAndro" :depends-on ("_package"))
  ))