;; Auto-generated. Do not edit!


(when (boundp 'yolo::yolo)
  (if (not (find-package "YOLO"))
    (make-package "YOLO"))
  (shadow 'yolo (find-package "YOLO")))
(unless (find-package "YOLO::YOLO")
  (make-package "YOLO::YOLO"))

(in-package "ROS")
;;//! \htmlinclude yolo.msg.html


(defclass yolo::yolo
  :super ros::object
  :slots (_leixing _x _y _confidence ))

(defmethod yolo::yolo
  (:init
   (&key
    ((:leixing __leixing) "")
    ((:x __x) 0)
    ((:y __y) 0)
    ((:confidence __confidence) 0.0)
    )
   (send-super :init)
   (setq _leixing (string __leixing))
   (setq _x (round __x))
   (setq _y (round __y))
   (setq _confidence (float __confidence))
   self)
  (:leixing
   (&optional __leixing)
   (if __leixing (setq _leixing __leixing)) _leixing)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:serialization-length
   ()
   (+
    ;; string _leixing
    4 (length _leixing)
    ;; uint16 _x
    2
    ;; uint16 _y
    2
    ;; float32 _confidence
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _leixing
       (write-long (length _leixing) s) (princ _leixing s)
     ;; uint16 _x
       (write-word _x s)
     ;; uint16 _y
       (write-word _y s)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _leixing
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _leixing (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint16 _x
     (setq _x (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _y
     (setq _y (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get yolo::yolo :md5sum-) "1740de2716a9dc4b45746fc186bca2f5")
(setf (get yolo::yolo :datatype-) "yolo/yolo")
(setf (get yolo::yolo :definition-)
      "string leixing
uint16 x
uint16 y
float32 confidence


")



(provide :yolo/yolo "1740de2716a9dc4b45746fc186bca2f5")


