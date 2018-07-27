; Auto-generated. Do not edit!


(cl:in-package pepper_tracking-msg)


;//! \htmlinclude image_request.msg.html

(cl:defclass <image_request> (roslisp-msg-protocol:ros-message)
  ((task
    :reader task
    :initarg :task
    :type cl:string
    :initform "")
   (image_name
    :reader image_name
    :initarg :image_name
    :type cl:string
    :initform "")
   (image_data
    :reader image_data
    :initarg :image_data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass image_request (<image_request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <image_request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'image_request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pepper_tracking-msg:<image_request> is deprecated: use pepper_tracking-msg:image_request instead.")))

(cl:ensure-generic-function 'task-val :lambda-list '(m))
(cl:defmethod task-val ((m <image_request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pepper_tracking-msg:task-val is deprecated.  Use pepper_tracking-msg:task instead.")
  (task m))

(cl:ensure-generic-function 'image_name-val :lambda-list '(m))
(cl:defmethod image_name-val ((m <image_request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pepper_tracking-msg:image_name-val is deprecated.  Use pepper_tracking-msg:image_name instead.")
  (image_name m))

(cl:ensure-generic-function 'image_data-val :lambda-list '(m))
(cl:defmethod image_data-val ((m <image_request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pepper_tracking-msg:image_data-val is deprecated.  Use pepper_tracking-msg:image_data instead.")
  (image_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <image_request>) ostream)
  "Serializes a message object of type '<image_request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'task))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'task))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'image_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'image_name))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'image_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'image_data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <image_request>) istream)
  "Deserializes a message object of type '<image_request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'task) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'task) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'image_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'image_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'image_data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'image_data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<image_request>)))
  "Returns string type for a message object of type '<image_request>"
  "pepper_tracking/image_request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'image_request)))
  "Returns string type for a message object of type 'image_request"
  "pepper_tracking/image_request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<image_request>)))
  "Returns md5sum for a message object of type '<image_request>"
  "b557edbbfe4bf3b00801f9e1895f014e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'image_request)))
  "Returns md5sum for a message object of type 'image_request"
  "b557edbbfe4bf3b00801f9e1895f014e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<image_request>)))
  "Returns full string definition for message of type '<image_request>"
  (cl:format cl:nil "string task~%string image_name~%uint8[] image_data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'image_request)))
  "Returns full string definition for message of type 'image_request"
  (cl:format cl:nil "string task~%string image_name~%uint8[] image_data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <image_request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'task))
     4 (cl:length (cl:slot-value msg 'image_name))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'image_data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <image_request>))
  "Converts a ROS message object to a list"
  (cl:list 'image_request
    (cl:cons ':task (task msg))
    (cl:cons ':image_name (image_name msg))
    (cl:cons ':image_data (image_data msg))
))
