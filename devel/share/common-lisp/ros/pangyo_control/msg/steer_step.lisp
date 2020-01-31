; Auto-generated. Do not edit!


(cl:in-package pangyo_control-msg)


;//! \htmlinclude steer_step.msg.html

(cl:defclass <steer_step> (roslisp-msg-protocol:ros-message)
  ((steer
    :reader steer
    :initarg :steer
    :type cl:float
    :initform 0.0)
   (step
    :reader step
    :initarg :step
    :type cl:fixnum
    :initform 0))
)

(cl:defclass steer_step (<steer_step>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <steer_step>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'steer_step)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pangyo_control-msg:<steer_step> is deprecated: use pangyo_control-msg:steer_step instead.")))

(cl:ensure-generic-function 'steer-val :lambda-list '(m))
(cl:defmethod steer-val ((m <steer_step>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pangyo_control-msg:steer-val is deprecated.  Use pangyo_control-msg:steer instead.")
  (steer m))

(cl:ensure-generic-function 'step-val :lambda-list '(m))
(cl:defmethod step-val ((m <steer_step>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pangyo_control-msg:step-val is deprecated.  Use pangyo_control-msg:step instead.")
  (step m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <steer_step>) ostream)
  "Serializes a message object of type '<steer_step>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'steer))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'step)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'step)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <steer_step>) istream)
  "Deserializes a message object of type '<steer_step>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steer) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'step)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'step)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<steer_step>)))
  "Returns string type for a message object of type '<steer_step>"
  "pangyo_control/steer_step")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'steer_step)))
  "Returns string type for a message object of type 'steer_step"
  "pangyo_control/steer_step")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<steer_step>)))
  "Returns md5sum for a message object of type '<steer_step>"
  "d57c21a8b6c47635ef292bfcd70e3721")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'steer_step)))
  "Returns md5sum for a message object of type 'steer_step"
  "d57c21a8b6c47635ef292bfcd70e3721")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<steer_step>)))
  "Returns full string definition for message of type '<steer_step>"
  (cl:format cl:nil "float32 steer~%uint16 step~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'steer_step)))
  "Returns full string definition for message of type 'steer_step"
  (cl:format cl:nil "float32 steer~%uint16 step~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <steer_step>))
  (cl:+ 0
     4
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <steer_step>))
  "Converts a ROS message object to a list"
  (cl:list 'steer_step
    (cl:cons ':steer (steer msg))
    (cl:cons ':step (step msg))
))
