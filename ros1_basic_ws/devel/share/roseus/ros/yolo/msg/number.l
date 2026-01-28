;; Auto-generated. Do not edit!


(when (boundp 'yolo::number)
  (if (not (find-package "YOLO"))
    (make-package "YOLO"))
  (shadow 'number (find-package "YOLO")))
(unless (find-package "YOLO::NUMBER")
  (make-package "YOLO::NUMBER"))

(in-package "ROS")
;;//! \htmlinclude number.msg.html


(defclass yolo::number
  :super ros::object
  :slots (_number _MaxH _MinH _CenterH _AverageH _x _y ))

(defmethod yolo::number
  (:init
   (&key
    ((:number __number) 0)
    ((:MaxH __MaxH) 0.0)
    ((:MinH __MinH) 0.0)
    ((:CenterH __CenterH) 0.0)
    ((:AverageH __AverageH) 0.0)
    ((:x __x) 0)
    ((:y __y) 0)
    )
   (send-super :init)
   (setq _number (round __number))
   (setq _MaxH (float __MaxH))
   (setq _MinH (float __MinH))
   (setq _CenterH (float __CenterH))
   (setq _AverageH (float __AverageH))
   (setq _x (round __x))
   (setq _y (round __y))
   self)
  (:number
   (&optional __number)
   (if __number (setq _number __number)) _number)
  (:MaxH
   (&optional __MaxH)
   (if __MaxH (setq _MaxH __MaxH)) _MaxH)
  (:MinH
   (&optional __MinH)
   (if __MinH (setq _MinH __MinH)) _MinH)
  (:CenterH
   (&optional __CenterH)
   (if __CenterH (setq _CenterH __CenterH)) _CenterH)
  (:AverageH
   (&optional __AverageH)
   (if __AverageH (setq _AverageH __AverageH)) _AverageH)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; uint8 _number
    1
    ;; float32 _MaxH
    4
    ;; float32 _MinH
    4
    ;; float32 _CenterH
    4
    ;; float32 _AverageH
    4
    ;; uint8 _x
    1
    ;; uint8 _y
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _number
       (write-byte _number s)
     ;; float32 _MaxH
       (sys::poke _MaxH (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _MinH
       (sys::poke _MinH (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _CenterH
       (sys::poke _CenterH (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _AverageH
       (sys::poke _AverageH (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _x
       (write-byte _x s)
     ;; uint8 _y
       (write-byte _y s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _number
     (setq _number (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _MaxH
     (setq _MaxH (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _MinH
     (setq _MinH (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _CenterH
     (setq _CenterH (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _AverageH
     (setq _AverageH (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _x
     (setq _x (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _y
     (setq _y (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get yolo::number :md5sum-) "8414b5a969236b037efc4bd669e714c5")
(setf (get yolo::number :datatype-) "yolo/number")
(setf (get yolo::number :definition-)
      "uint8 number
float32 MaxH
float32 MinH
float32 CenterH
float32 AverageH
uint8 x
uint8 y

")



(provide :yolo/number "8414b5a969236b037efc4bd669e714c5")


