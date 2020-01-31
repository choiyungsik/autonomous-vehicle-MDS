; Auto-generated. Do not edit!


(cl:in-package pangyo_control-msg)


;//! \htmlinclude figure_array.msg.html

(cl:defclass <figure_array> (roslisp-msg-protocol:ros-message)
  ((figure_array
    :reader figure_array
    :initarg :figure_array
    :type (cl:vector pangyo_control-msg:figure)
   :initform (cl:make-array 0 :element-type 'pangyo_control-msg:figure :initial-element (cl:make-instance 'pangyo_control-msg:figure))))
)

(cl:defclass figure_array (<figure_array>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <figure_array>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'figure_array)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pangyo_control-msg:<figure_array> is deprecated: use pangyo_control-msg:figure_array instead.")))

(cl:ensure-generic-function 'figure_array-val :lambda-list '(m))
(cl:defmethod figure_array-val ((m <figure_array>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pangyo_control-msg:figure_array-val is deprecated.  Use pangyo_control-msg:figure_array instead.")
  (figure_array m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <figure_array>) ostream)
  "Serializes a message object of type '<figure_array>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'figure_array))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'figure_array))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <figure_array>) istream)
  "Deserializes a message object of type '<figure_array>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'figure_array) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'figure_array)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'pangyo_control-msg:figure))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<figure_array>)))
  "Returns string type for a message object of type '<figure_array>"
  "pangyo_control/figure_array")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'figure_array)))
  "Returns string type for a message object of type 'figure_array"
  "pangyo_control/figure_array")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<figure_array>)))
  "Returns md5sum for a message object of type '<figure_array>"
  "51ba417e9d022c3e94d411dca3690368")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'figure_array)))
  "Returns md5sum for a message object of type 'figure_array"
  "51ba417e9d022c3e94d411dca3690368")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<figure_array>)))
  "Returns full string definition for message of type '<figure_array>"
  (cl:format cl:nil "pangyo_control/figure[] figure_array~%~%================================================================================~%MSG: pangyo_control/figure~%int32[] figure~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'figure_array)))
  "Returns full string definition for message of type 'figure_array"
  (cl:format cl:nil "pangyo_control/figure[] figure_array~%~%================================================================================~%MSG: pangyo_control/figure~%int32[] figure~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <figure_array>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'figure_array) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <figure_array>))
  "Converts a ROS message object to a list"
  (cl:list 'figure_array
    (cl:cons ':figure_array (figure_array msg))
))
