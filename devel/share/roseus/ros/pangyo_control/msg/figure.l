;; Auto-generated. Do not edit!


(when (boundp 'pangyo_control::figure)
  (if (not (find-package "PANGYO_CONTROL"))
    (make-package "PANGYO_CONTROL"))
  (shadow 'figure (find-package "PANGYO_CONTROL")))
(unless (find-package "PANGYO_CONTROL::FIGURE")
  (make-package "PANGYO_CONTROL::FIGURE"))

(in-package "ROS")
;;//! \htmlinclude figure.msg.html


(defclass pangyo_control::figure
  :super ros::object
  :slots (_figure ))

(defmethod pangyo_control::figure
  (:init
   (&key
    ((:figure __figure) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _figure __figure)
   self)
  (:figure
   (&optional __figure)
   (if __figure (setq _figure __figure)) _figure)
  (:serialization-length
   ()
   (+
    ;; int32[] _figure
    (* 4    (length _figure)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _figure
     (write-long (length _figure) s)
     (dotimes (i (length _figure))
       (write-long (elt _figure i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _figure
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _figure (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _figure i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get pangyo_control::figure :md5sum-) "5aeb03f16c36d602af577d480cad4dd3")
(setf (get pangyo_control::figure :datatype-) "pangyo_control/figure")
(setf (get pangyo_control::figure :definition-)
      "int32[] figure

")



(provide :pangyo_control/figure "5aeb03f16c36d602af577d480cad4dd3")


