; Auto-generated. Do not edit!


(cl:in-package yolo-msg)


;//! \htmlinclude yolo.msg.html

(cl:defclass <yolo> (roslisp-msg-protocol:ros-message)
  ((leixing
    :reader leixing
    :initarg :leixing
    :type cl:string
    :initform "")
   (x
    :reader x
    :initarg :x
    :type cl:fixnum
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:fixnum
    :initform 0)
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0))
)

(cl:defclass yolo (<yolo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <yolo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'yolo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yolo-msg:<yolo> is deprecated: use yolo-msg:yolo instead.")))

(cl:ensure-generic-function 'leixing-val :lambda-list '(m))
(cl:defmethod leixing-val ((m <yolo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolo-msg:leixing-val is deprecated.  Use yolo-msg:leixing instead.")
  (leixing m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <yolo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolo-msg:x-val is deprecated.  Use yolo-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <yolo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolo-msg:y-val is deprecated.  Use yolo-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <yolo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolo-msg:confidence-val is deprecated.  Use yolo-msg:confidence instead.")
  (confidence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <yolo>) ostream)
  "Serializes a message object of type '<yolo>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'leixing))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'leixing))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'y)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <yolo>) istream)
  "Deserializes a message object of type '<yolo>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'leixing) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'leixing) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'y)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<yolo>)))
  "Returns string type for a message object of type '<yolo>"
  "yolo/yolo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'yolo)))
  "Returns string type for a message object of type 'yolo"
  "yolo/yolo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<yolo>)))
  "Returns md5sum for a message object of type '<yolo>"
  "1740de2716a9dc4b45746fc186bca2f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'yolo)))
  "Returns md5sum for a message object of type 'yolo"
  "1740de2716a9dc4b45746fc186bca2f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<yolo>)))
  "Returns full string definition for message of type '<yolo>"
  (cl:format cl:nil "string leixing~%uint16 x~%uint16 y~%float32 confidence~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'yolo)))
  "Returns full string definition for message of type 'yolo"
  (cl:format cl:nil "string leixing~%uint16 x~%uint16 y~%float32 confidence~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <yolo>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'leixing))
     2
     2
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <yolo>))
  "Converts a ROS message object to a list"
  (cl:list 'yolo
    (cl:cons ':leixing (leixing msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':confidence (confidence msg))
))
