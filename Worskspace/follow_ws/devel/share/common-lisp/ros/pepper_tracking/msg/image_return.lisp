; Auto-generated. Do not edit!


(cl:in-package pepper_tracking-msg)


;//! \htmlinclude image_return.msg.html

(cl:defclass <image_return> (roslisp-msg-protocol:ros-message)
  ((found
    :reader found
    :initarg :found
    :type cl:boolean
    :initform cl:nil)
   (image_name
    :reader image_name
    :initarg :image_name
    :type cl:string
    :initform ""))
)

(cl:defclass image_return (<image_return>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <image_return>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'image_return)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pepper_tracking-msg:<image_return> is deprecated: use pepper_tracking-msg:image_return instead.")))

(cl:ensure-generic-function 'found-val :lambda-list '(m))
(cl:defmethod found-val ((m <image_return>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pepper_tracking-msg:found-val is deprecated.  Use pepper_tracking-msg:found instead.")
  (found m))

(cl:ensure-generic-function 'image_name-val :lambda-list '(m))
(cl:defmethod image_name-val ((m <image_return>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pepper_tracking-msg:image_name-val is deprecated.  Use pepper_tracking-msg:image_name instead.")
  (image_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <image_return>) ostream)
  "Serializes a message object of type '<image_return>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'found) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'image_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'image_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <image_return>) istream)
  "Deserializes a message object of type '<image_return>"
    (cl:setf (cl:slot-value msg 'found) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'image_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'image_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<image_return>)))
  "Returns string type for a message object of type '<image_return>"
  "pepper_tracking/image_return")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'image_return)))
  "Returns string type for a message object of type 'image_return"
  "pepper_tracking/image_return")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<image_return>)))
  "Returns md5sum for a message object of type '<image_return>"
  "68cd02969b39e785e4cdf1cf76db231c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'image_return)))
  "Returns md5sum for a message object of type 'image_return"
  "68cd02969b39e785e4cdf1cf76db231c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<image_return>)))
  "Returns full string definition for message of type '<image_return>"
  (cl:format cl:nil "bool found~%string image_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'image_return)))
  "Returns full string definition for message of type 'image_return"
  (cl:format cl:nil "bool found~%string image_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <image_return>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'image_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <image_return>))
  "Converts a ROS message object to a list"
  (cl:list 'image_return
    (cl:cons ':found (found msg))
    (cl:cons ':image_name (image_name msg))
))
