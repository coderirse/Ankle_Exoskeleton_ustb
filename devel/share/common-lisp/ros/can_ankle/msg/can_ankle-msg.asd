
(cl:in-package :asdf)

(defsystem "can_ankle-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Torque" :depends-on ("_package_Torque"))
    (:file "_package_Torque" :depends-on ("_package"))
  ))