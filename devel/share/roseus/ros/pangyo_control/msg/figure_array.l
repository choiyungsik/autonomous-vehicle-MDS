;; Auto-generated. Do not edit!


(when (boundp 'pangyo_control::figure_array)
  (if (not (find-package "PANGYO_CONTROL"))
    (make-package "PANGYO_CONTROL"))
  (shadow 'figure_array (find-package "PANGYO_CONTROL")))
(unless (find-package "PANGYO_CONTROL::FIGURE_ARRAY")
  (make-package "PANGYO_CONTROL::FIGURE_ARRAY"))

(in-package "ROS")
;;//! \htmlinclude figure_array.msg.html


(defclass pangyo_control::figure_array
  :super ros::object
  :slots (_figure_array ))

(defmethod pangyo_control::figure_array
  (:init
   (&key
    ((:figure_array __figure_array) (let (r) (dotimes (i 0) (push (instance pangyo_control::figure :init) r)) r))
    )
   (send-super :init)
   (setq _figure_array __figure_array)
   self)
  (:figure_array
   (&rest __figure_array)
   (if (keywordp (car __figure_array))
       (send* _figure_array __figure_array)
     (progn
       (if __figure_array (setq _figure_array (car __figure_array)))
       _figure_array)))
  (:serialization-length
   ()
   (+
    ;; pangyo_control/figure[] _figure_array
    (apply #'+ (send-all _figure_array :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; pangyo_control/figure[] _figure_array
     (write-long (length _figure_array) s)
     (dolist (elem _figure_array)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; pangyo_control/figure[] _figure_array
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _figure_array (let (r) (dotimes (i n) (push (instance pangyo_control::figure :init) r)) r))
     (dolist (elem- _figure_array)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get pangyo_control::figure_array :md5sum-) "51ba417e9d022c3e94d411dca3690368")
(setf (get pangyo_control::figure_array :datatype-) "pangyo_control/figure_array")
(setf (get pangyo_control::figure_array :definition-)
      "pangyo_control/figure[] figure_array

================================================================================
MSG: pangyo_control/figure
int32[] figure

")



(provide :pangyo_control/figure_array "51ba417e9d022c3e94d411dca3690368")


