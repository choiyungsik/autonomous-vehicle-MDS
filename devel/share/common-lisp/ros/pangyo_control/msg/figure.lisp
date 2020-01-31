; Auto-generated. Do not edit!


(cl:in-package pangyo_control-msg)


;//! \htmlinclude figure.msg.html

(cl:defclass <figure> (roslisp-msg-protocol:ros-message)
  ((figure
    :reader figure
    :initarg :figure
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass figure (<figure>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <figure>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'figure)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pangyo_control-msg:<figure> is deprecated: use pangyo_control-msg:figure instead.")))

(cl:ensure-generic-function 'figure-val :lambda-list '(m))
(cl:defmethod figure-val ((m <figure>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pangyo_control-msg:figure-val is deprecated.  Use pangyo_control-msg:figure instead.")
  (figure m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <figure>) ostream)
  "Serializes a message object of type '<figure>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'figure))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'figure))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <figure>) istream)
  "Deserializes a message object of type '<figure>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'figure) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'figure)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<figure>)))
  "Returns string type for a message object of type '<figure>"
  "pangyo_control/figure")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'figure)))
  "Returns string type for a message object of type 'figure"
  "pangyo_control/figure")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<figure>)))
  "Returns md5sum for a message object of type '<figure>"
  "5aeb03f16c36d602af577d480cad4dd3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'figure)))
  "Returns md5sum for a message object of type 'figure"
  "5aeb03f16c36d602af577d480cad4dd3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<figure>)))
  "Returns full string definition for message of type '<figure>"
  (cl:format cl:nil "int32[] figure~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'figure)))
  "Returns full string definition for message of type 'figure"
  (cl:format cl:nil "int32[] figure~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <figure>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'figure) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <figure>))
  "Converts a ROS message object to a list"
  (cl:list 'figure
    (cl:cons ':figure (figure msg))
))
