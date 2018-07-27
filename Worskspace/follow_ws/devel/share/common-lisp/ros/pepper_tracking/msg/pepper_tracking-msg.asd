
(cl:in-package :asdf)

(defsystem "pepper_tracking-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "image_request" :depends-on ("_package_image_request"))
    (:file "_package_image_request" :depends-on ("_package"))
    (:file "image_return" :depends-on ("_package_image_return"))
    (:file "_package_image_return" :depends-on ("_package"))
  ))