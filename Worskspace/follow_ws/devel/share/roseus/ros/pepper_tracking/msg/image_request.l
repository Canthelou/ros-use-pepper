;; Auto-generated. Do not edit!


(when (boundp 'pepper_tracking::image_request)
  (if (not (find-package "PEPPER_TRACKING"))
    (make-package "PEPPER_TRACKING"))
  (shadow 'image_request (find-package "PEPPER_TRACKING")))
(unless (find-package "PEPPER_TRACKING::IMAGE_REQUEST")
  (make-package "PEPPER_TRACKING::IMAGE_REQUEST"))

(in-package "ROS")
;;//! \htmlinclude image_request.msg.html


(defclass pepper_tracking::image_request
  :super ros::object
  :slots (_task _image_name _image_data ))

(defmethod pepper_tracking::image_request
  (:init
   (&key
    ((:task __task) "")
    ((:image_name __image_name) "")
    ((:image_data __image_data) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _task (string __task))
   (setq _image_name (string __image_name))
   (setq _image_data __image_data)
   self)
  (:task
   (&optional __task)
   (if __task (setq _task __task)) _task)
  (:image_name
   (&optional __image_name)
   (if __image_name (setq _image_name __image_name)) _image_name)
  (:image_data
   (&optional __image_data)
   (if __image_data (setq _image_data __image_data)) _image_data)
  (:serialization-length
   ()
   (+
    ;; string _task
    4 (length _task)
    ;; string _image_name
    4 (length _image_name)
    ;; uint8[] _image_data
    (* 1    (length _image_data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _task
       (write-long (length _task) s) (princ _task s)
     ;; string _image_name
       (write-long (length _image_name) s) (princ _image_name s)
     ;; uint8[] _image_data
     (write-long (length _image_data) s)
     (princ _image_data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _task
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _task (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _image_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _image_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8[] _image_data
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _image_data (make-array n :element-type :char))
     (replace _image_data buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get pepper_tracking::image_request :md5sum-) "b557edbbfe4bf3b00801f9e1895f014e")
(setf (get pepper_tracking::image_request :datatype-) "pepper_tracking/image_request")
(setf (get pepper_tracking::image_request :definition-)
      "string task
string image_name
uint8[] image_data

")



(provide :pepper_tracking/image_request "b557edbbfe4bf3b00801f9e1895f014e")


