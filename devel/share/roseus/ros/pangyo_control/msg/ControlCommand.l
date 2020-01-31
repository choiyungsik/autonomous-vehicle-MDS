;; Auto-generated. Do not edit!


(when (boundp 'pangyo_control::ControlCommand)
  (if (not (find-package "PANGYO_CONTROL"))
    (make-package "PANGYO_CONTROL"))
  (shadow 'ControlCommand (find-package "PANGYO_CONTROL")))
(unless (find-package "PANGYO_CONTROL::CONTROLCOMMAND")
  (make-package "PANGYO_CONTROL::CONTROLCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude ControlCommand.msg.html


(defclass pangyo_control::ControlCommand
  :super ros::object
  :slots (_gear _speed _steer _brake ))

(defmethod pangyo_control::ControlCommand
  (:init
   (&key
    ((:gear __gear) 0)
    ((:speed __speed) 0)
    ((:steer __steer) 0.0)
    ((:brake __brake) 0)
    )
   (send-super :init)
   (setq _gear (round __gear))
   (setq _speed (round __speed))
   (setq _steer (float __steer))
   (setq _brake (round __brake))
   self)
  (:gear
   (&optional __gear)
   (if __gear (setq _gear __gear)) _gear)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:steer
   (&optional __steer)
   (if __steer (setq _steer __steer)) _steer)
  (:brake
   (&optional __brake)
   (if __brake (setq _brake __brake)) _brake)
  (:serialization-length
   ()
   (+
    ;; uint16 _gear
    2
    ;; uint16 _speed
    2
    ;; float32 _steer
    4
    ;; uint16 _brake
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _gear
       (write-word _gear s)
     ;; uint16 _speed
       (write-word _speed s)
     ;; float32 _steer
       (sys::poke _steer (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint16 _brake
       (write-word _brake s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _gear
     (setq _gear (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _speed
     (setq _speed (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _steer
     (setq _steer (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint16 _brake
     (setq _brake (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get pangyo_control::ControlCommand :md5sum-) "37feea1cd0d627db0c18584be77b9973")
(setf (get pangyo_control::ControlCommand :datatype-) "pangyo_control/ControlCommand")
(setf (get pangyo_control::ControlCommand :definition-)
      "uint16 gear
uint16 speed
float32 steer
uint16 brake

")



(provide :pangyo_control/ControlCommand "37feea1cd0d627db0c18584be77b9973")


