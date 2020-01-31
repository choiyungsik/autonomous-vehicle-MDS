;; Auto-generated. Do not edit!


(when (boundp 'pangyo_control::steer_step)
  (if (not (find-package "PANGYO_CONTROL"))
    (make-package "PANGYO_CONTROL"))
  (shadow 'steer_step (find-package "PANGYO_CONTROL")))
(unless (find-package "PANGYO_CONTROL::STEER_STEP")
  (make-package "PANGYO_CONTROL::STEER_STEP"))

(in-package "ROS")
;;//! \htmlinclude steer_step.msg.html


(defclass pangyo_control::steer_step
  :super ros::object
  :slots (_steer _step ))

(defmethod pangyo_control::steer_step
  (:init
   (&key
    ((:steer __steer) 0.0)
    ((:step __step) 0)
    )
   (send-super :init)
   (setq _steer (float __steer))
   (setq _step (round __step))
   self)
  (:steer
   (&optional __steer)
   (if __steer (setq _steer __steer)) _steer)
  (:step
   (&optional __step)
   (if __step (setq _step __step)) _step)
  (:serialization-length
   ()
   (+
    ;; float32 _steer
    4
    ;; uint16 _step
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _steer
       (sys::poke _steer (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint16 _step
       (write-word _step s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _steer
     (setq _steer (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint16 _step
     (setq _step (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get pangyo_control::steer_step :md5sum-) "d57c21a8b6c47635ef292bfcd70e3721")
(setf (get pangyo_control::steer_step :datatype-) "pangyo_control/steer_step")
(setf (get pangyo_control::steer_step :definition-)
      "float32 steer
uint16 step

")



(provide :pangyo_control/steer_step "d57c21a8b6c47635ef292bfcd70e3721")


