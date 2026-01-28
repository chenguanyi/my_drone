;; Auto-generated. Do not edit!


(when (boundp 'unencoder::decode)
  (if (not (find-package "UNENCODER"))
    (make-package "UNENCODER"))
  (shadow 'decode (find-package "UNENCODER")))
(unless (find-package "UNENCODER::DECODE")
  (make-package "UNENCODER::DECODE"))

(in-package "ROS")
;;//! \htmlinclude decode.msg.html


(defclass unencoder::decode
  :super ros::object
  :slots (_data _is_valid ))

(defmethod unencoder::decode
  (:init
   (&key
    ((:data __data) 0)
    ((:is_valid __is_valid) nil)
    )
   (send-super :init)
   (setq _data (round __data))
   (setq _is_valid __is_valid)
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:is_valid
   (&optional (__is_valid :null))
   (if (not (eq __is_valid :null)) (setq _is_valid __is_valid)) _is_valid)
  (:serialization-length
   ()
   (+
    ;; uint8 _data
    1
    ;; bool _is_valid
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _data
       (write-byte _data s)
     ;; bool _is_valid
       (if _is_valid (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _data
     (setq _data (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _is_valid
     (setq _is_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get unencoder::decode :md5sum-) "a7470f513ee83a01fc610ee9182c0f80")
(setf (get unencoder::decode :datatype-) "unencoder/decode")
(setf (get unencoder::decode :definition-)
      "uint8 data
bool  is_valid
")



(provide :unencoder/decode "a7470f513ee83a01fc610ee9182c0f80")


