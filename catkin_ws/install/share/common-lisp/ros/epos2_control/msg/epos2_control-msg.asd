
(cl:in-package :asdf)

(defsystem "epos2_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "velocity" :depends-on ("_package_velocity"))
    (:file "_package_velocity" :depends-on ("_package"))
    (:file "set_error" :depends-on ("_package_set_error"))
    (:file "_package_set_error" :depends-on ("_package"))
  ))