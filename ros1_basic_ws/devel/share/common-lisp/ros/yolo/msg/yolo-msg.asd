
(cl:in-package :asdf)

(defsystem "yolo-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "number" :depends-on ("_package_number"))
    (:file "_package_number" :depends-on ("_package"))
    (:file "yolo" :depends-on ("_package_yolo"))
    (:file "_package_yolo" :depends-on ("_package"))
  ))