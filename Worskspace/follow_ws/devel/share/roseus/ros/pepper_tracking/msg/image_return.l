;; Auto-generated. Do not edit!


(when (boundp 'pepper_tracking::image_return)
  (if (not (find-package "PEPPER_TRACKING"))
    (make-package "PEPPER_TRACKING"))
  (shadow 'image_return (find-package "PEPPER_TRACKING")))
(unless (find-package "PEPPER_TRACKING::IMAGE_RETURN")
  (make-package "PEPPER_TRACKING::IMAGE_RETURN"))

(in-package "ROS")
;;//! \htmlinclude image_return.msg.html


(defclass pepper_tracking::image_return
  :super ros::object
  :slots (_found _image_name ))

(defmethod pepper_tracking::image_return
  (:init
   (&key
    ((:found __found) nil)
    ((:image_name __image_name) "")
    )
   (send-super :init)
   (setq _found __found)
   (setq _image_name (string __image_name))
   self)
  (:found
   (&optional __found)
   (if __found (setq _found __found)) _found)
  (:image_name
   (&optional __image_name)
   (if __image_name (setq _image_name __image_name)) _image_name)
  (:serialization-length
   ()
   (+
    ;; bool _found
    1
    ;; string _image_name
    4 (length _image_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _found
       (if _found (write-byte -1 s) (write-byte 0 s))
     ;; string _image_name
       (write-long (length _image_name) s) (princ _image_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _found
     (setq _found (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _image_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _image_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get pepper_tracking::image_return :md5sum-) "68cd02969b39e785e4cdf1cf76db231c")
(setf (get pepper_tracking::image_return :datatype-) "pepper_tracking/image_return")
(setf (get pepper_tracking::image_return :definition-)
      "bool found
string image_name

")



(provide :pepper_tracking/image_return "68cd02969b39e785e4cdf1cf76db231c")


