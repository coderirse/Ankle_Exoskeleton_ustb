; Auto-generated. Do not edit!


(cl:in-package can_ankle-msg)


;//! \htmlinclude ForceSensor.msg.html

(cl:defclass <ForceSensor> (roslisp-msg-protocol:ros-message)
  ((ForceValue
    :reader ForceValue
    :initarg :ForceValue
    :type cl:float
    :initform 0.0))
)

(cl:defclass ForceSensor (<ForceSensor>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ForceSensor>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ForceSensor)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name can_ankle-msg:<ForceSensor> is deprecated: use can_ankle-msg:ForceSensor instead.")))

(cl:ensure-generic-function 'ForceValue-val :lambda-list '(m))
(cl:defmethod ForceValue-val ((m <ForceSensor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader can_ankle-msg:ForceValue-val is deprecated.  Use can_ankle-msg:ForceValue instead.")
  (ForceValue m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ForceSensor>) ostream)
  "Serializes a message object of type '<ForceSensor>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ForceValue))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ForceSensor>) istream)
  "Deserializes a message object of type '<ForceSensor>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ForceValue) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ForceSensor>)))
  "Returns string type for a message object of type '<ForceSensor>"
  "can_ankle/ForceSensor")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ForceSensor)))
  "Returns string type for a message object of type 'ForceSensor"
  "can_ankle/ForceSensor")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ForceSensor>)))
  "Returns md5sum for a message object of type '<ForceSensor>"
  "0610f446d964ed45a89ead06523db642")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ForceSensor)))
  "Returns md5sum for a message object of type 'ForceSensor"
  "0610f446d964ed45a89ead06523db642")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ForceSensor>)))
  "Returns full string definition for message of type '<ForceSensor>"
  (cl:format cl:nil "float32 ForceValue~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ForceSensor)))
  "Returns full string definition for message of type 'ForceSensor"
  (cl:format cl:nil "float32 ForceValue~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ForceSensor>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ForceSensor>))
  "Converts a ROS message object to a list"
  (cl:list 'ForceSensor
    (cl:cons ':ForceValue (ForceValue msg))
))
