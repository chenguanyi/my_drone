
(cl:in-package :asdf)

(defsystem "unencoder-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "decode" :depends-on ("_package_decode"))
    (:file "_package_decode" :depends-on ("_package"))
  ))