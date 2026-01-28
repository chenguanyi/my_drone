; Auto-generated. Do not edit!


(cl:in-package yolo-msg)


;//! \htmlinclude number.msg.html

(cl:defclass <number> (roslisp-msg-protocol:ros-message)
  ((number
    :reader number
    :initarg :number
    :type cl:fixnum
    :initform 0)
   (MaxH
    :reader MaxH
    :initarg :MaxH
    :type cl:float
    :initform 0.0)
   (MinH
    :reader MinH
    :initarg :MinH
    :type cl:float
    :initform 0.0)
   (CenterH
    :reader CenterH
    :initarg :CenterH
    :type cl:float
    :initform 0.0)
   (AverageH
    :reader AverageH
    :initarg :AverageH
    :type cl:float
    :initform 0.0)
   (x
    :reader x
    :initarg :x
    :type cl:fixnum
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:fixnum
    :initform 0))
)

(cl:defclass number (<number>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <number>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'number)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yolo-msg:<number> is deprecated: use yolo-msg:number instead.")))

(cl:ensure-generic-function 'number-val :lambda-list '(m))
(cl:defmethod number-val ((m <number>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolo-msg:number-val is deprecated.  Use yolo-msg:number instead.")
  (number m))

(cl:ensure-generic-function 'MaxH-val :lambda-list '(m))
(cl:defmethod MaxH-val ((m <number>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolo-msg:MaxH-val is deprecated.  Use yolo-msg:MaxH instead.")
  (MaxH m))

(cl:ensure-generic-function 'MinH-val :lambda-list '(m))
(cl:defmethod MinH-val ((m <number>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolo-msg:MinH-val is deprecated.  Use yolo-msg:MinH instead.")
  (MinH m))

(cl:ensure-generic-function 'CenterH-val :lambda-list '(m))
(cl:defmethod CenterH-val ((m <number>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolo-msg:CenterH-val is deprecated.  Use yolo-msg:CenterH instead.")
  (CenterH m))

(cl:ensure-generic-function 'AverageH-val :lambda-list '(m))
(cl:defmethod AverageH-val ((m <number>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolo-msg:AverageH-val is deprecated.  Use yolo-msg:AverageH instead.")
  (AverageH m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <number>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolo-msg:x-val is deprecated.  Use yolo-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <number>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolo-msg:y-val is deprecated.  Use yolo-msg:y instead.")
  (y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <number>) ostream)
  "Serializes a message object of type '<number>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'number)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'MaxH))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'MinH))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'CenterH))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'AverageH))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'y)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <number>) istream)
  "Deserializes a message object of type '<number>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'number)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'MaxH) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'MinH) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'CenterH) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'AverageH) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'y)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<number>)))
  "Returns string type for a message object of type '<number>"
  "yolo/number")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'number)))
  "Returns string type for a message object of type 'number"
  "yolo/number")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<number>)))
  "Returns md5sum for a message object of type '<number>"
  "8414b5a969236b037efc4bd669e714c5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'number)))
  "Returns md5sum for a message object of type 'number"
  "8414b5a969236b037efc4bd669e714c5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<number>)))
  "Returns full string definition for message of type '<number>"
  (cl:format cl:nil "uint8 number~%float32 MaxH~%float32 MinH~%float32 CenterH~%float32 AverageH~%uint8 x~%uint8 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'number)))
  "Returns full string definition for message of type 'number"
  (cl:format cl:nil "uint8 number~%float32 MaxH~%float32 MinH~%float32 CenterH~%float32 AverageH~%uint8 x~%uint8 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <number>))
  (cl:+ 0
     1
     4
     4
     4
     4
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <number>))
  "Converts a ROS message object to a list"
  (cl:list 'number
    (cl:cons ':number (number msg))
    (cl:cons ':MaxH (MaxH msg))
    (cl:cons ':MinH (MinH msg))
    (cl:cons ':CenterH (CenterH msg))
    (cl:cons ':AverageH (AverageH msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
))
