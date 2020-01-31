; Auto-generated. Do not edit!


(cl:in-package pangyo_control-msg)


;//! \htmlinclude ControlCommand.msg.html

(cl:defclass <ControlCommand> (roslisp-msg-protocol:ros-message)
  ((gear
    :reader gear
    :initarg :gear
    :type cl:fixnum
    :initform 0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:fixnum
    :initform 0)
   (steer
    :reader steer
    :initarg :steer
    :type cl:float
    :initform 0.0)
   (brake
    :reader brake
    :initarg :brake
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ControlCommand (<ControlCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pangyo_control-msg:<ControlCommand> is deprecated: use pangyo_control-msg:ControlCommand instead.")))

(cl:ensure-generic-function 'gear-val :lambda-list '(m))
(cl:defmethod gear-val ((m <ControlCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pangyo_control-msg:gear-val is deprecated.  Use pangyo_control-msg:gear instead.")
  (gear m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <ControlCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pangyo_control-msg:speed-val is deprecated.  Use pangyo_control-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'steer-val :lambda-list '(m))
(cl:defmethod steer-val ((m <ControlCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pangyo_control-msg:steer-val is deprecated.  Use pangyo_control-msg:steer instead.")
  (steer m))

(cl:ensure-generic-function 'brake-val :lambda-list '(m))
(cl:defmethod brake-val ((m <ControlCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pangyo_control-msg:brake-val is deprecated.  Use pangyo_control-msg:brake instead.")
  (brake m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlCommand>) ostream)
  "Serializes a message object of type '<ControlCommand>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gear)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gear)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'speed)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'speed)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'steer))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'brake)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'brake)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlCommand>) istream)
  "Deserializes a message object of type '<ControlCommand>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gear)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gear)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'speed)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'speed)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steer) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'brake)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'brake)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlCommand>)))
  "Returns string type for a message object of type '<ControlCommand>"
  "pangyo_control/ControlCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlCommand)))
  "Returns string type for a message object of type 'ControlCommand"
  "pangyo_control/ControlCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlCommand>)))
  "Returns md5sum for a message object of type '<ControlCommand>"
  "37feea1cd0d627db0c18584be77b9973")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlCommand)))
  "Returns md5sum for a message object of type 'ControlCommand"
  "37feea1cd0d627db0c18584be77b9973")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlCommand>)))
  "Returns full string definition for message of type '<ControlCommand>"
  (cl:format cl:nil "uint16 gear~%uint16 speed~%float32 steer~%uint16 brake~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlCommand)))
  "Returns full string definition for message of type 'ControlCommand"
  (cl:format cl:nil "uint16 gear~%uint16 speed~%float32 steer~%uint16 brake~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlCommand>))
  (cl:+ 0
     2
     2
     4
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlCommand
    (cl:cons ':gear (gear msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':steer (steer msg))
    (cl:cons ':brake (brake msg))
))
