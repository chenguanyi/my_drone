; Auto-generated. Do not edit!


(cl:in-package unencoder-msg)


;//! \htmlinclude decode.msg.html

(cl:defclass <decode> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:fixnum
    :initform 0)
   (is_valid
    :reader is_valid
    :initarg :is_valid
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass decode (<decode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <decode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'decode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name unencoder-msg:<decode> is deprecated: use unencoder-msg:decode instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <decode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unencoder-msg:data-val is deprecated.  Use unencoder-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'is_valid-val :lambda-list '(m))
(cl:defmethod is_valid-val ((m <decode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader unencoder-msg:is_valid-val is deprecated.  Use unencoder-msg:is_valid instead.")
  (is_valid m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <decode>) ostream)
  "Serializes a message object of type '<decode>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'data)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_valid) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <decode>) istream)
  "Deserializes a message object of type '<decode>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'data)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'is_valid) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<decode>)))
  "Returns string type for a message object of type '<decode>"
  "unencoder/decode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'decode)))
  "Returns string type for a message object of type 'decode"
  "unencoder/decode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<decode>)))
  "Returns md5sum for a message object of type '<decode>"
  "a7470f513ee83a01fc610ee9182c0f80")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'decode)))
  "Returns md5sum for a message object of type 'decode"
  "a7470f513ee83a01fc610ee9182c0f80")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<decode>)))
  "Returns full string definition for message of type '<decode>"
  (cl:format cl:nil "uint8 data~%bool  is_valid~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'decode)))
  "Returns full string definition for message of type 'decode"
  (cl:format cl:nil "uint8 data~%bool  is_valid~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <decode>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <decode>))
  "Converts a ROS message object to a list"
  (cl:list 'decode
    (cl:cons ':data (data msg))
    (cl:cons ':is_valid (is_valid msg))
))
