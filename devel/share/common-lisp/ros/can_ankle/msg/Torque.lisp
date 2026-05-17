; Auto-generated. Do not edit!


(cl:in-package can_ankle-msg)


;//! \htmlinclude Torque.msg.html

(cl:defclass <Torque> (roslisp-msg-protocol:ros-message)
  ((TorqueValue
    :reader TorqueValue
    :initarg :TorqueValue
    :type cl:float
    :initform 0.0)
   (ReturnTorqueValue
    :reader ReturnTorqueValue
    :initarg :ReturnTorqueValue
    :type cl:float
    :initform 0.0)
   (VelocityValue
    :reader VelocityValue
    :initarg :VelocityValue
    :type cl:float
    :initform 0.0)
   (ReturnVelocity
    :reader ReturnVelocity
    :initarg :ReturnVelocity
    :type cl:float
    :initform 0.0)
   (PDvelocity
    :reader PDvelocity
    :initarg :PDvelocity
    :type cl:float
    :initform 0.0)
   (ForceSensortorque
    :reader ForceSensortorque
    :initarg :ForceSensortorque
    :type cl:float
    :initform 0.0))
)

(cl:defclass Torque (<Torque>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Torque>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Torque)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name can_ankle-msg:<Torque> is deprecated: use can_ankle-msg:Torque instead.")))

(cl:ensure-generic-function 'TorqueValue-val :lambda-list '(m))
(cl:defmethod TorqueValue-val ((m <Torque>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader can_ankle-msg:TorqueValue-val is deprecated.  Use can_ankle-msg:TorqueValue instead.")
  (TorqueValue m))

(cl:ensure-generic-function 'ReturnTorqueValue-val :lambda-list '(m))
(cl:defmethod ReturnTorqueValue-val ((m <Torque>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader can_ankle-msg:ReturnTorqueValue-val is deprecated.  Use can_ankle-msg:ReturnTorqueValue instead.")
  (ReturnTorqueValue m))

(cl:ensure-generic-function 'VelocityValue-val :lambda-list '(m))
(cl:defmethod VelocityValue-val ((m <Torque>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader can_ankle-msg:VelocityValue-val is deprecated.  Use can_ankle-msg:VelocityValue instead.")
  (VelocityValue m))

(cl:ensure-generic-function 'ReturnVelocity-val :lambda-list '(m))
(cl:defmethod ReturnVelocity-val ((m <Torque>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader can_ankle-msg:ReturnVelocity-val is deprecated.  Use can_ankle-msg:ReturnVelocity instead.")
  (ReturnVelocity m))

(cl:ensure-generic-function 'PDvelocity-val :lambda-list '(m))
(cl:defmethod PDvelocity-val ((m <Torque>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader can_ankle-msg:PDvelocity-val is deprecated.  Use can_ankle-msg:PDvelocity instead.")
  (PDvelocity m))

(cl:ensure-generic-function 'ForceSensortorque-val :lambda-list '(m))
(cl:defmethod ForceSensortorque-val ((m <Torque>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader can_ankle-msg:ForceSensortorque-val is deprecated.  Use can_ankle-msg:ForceSensortorque instead.")
  (ForceSensortorque m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Torque>) ostream)
  "Serializes a message object of type '<Torque>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'TorqueValue))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ReturnTorqueValue))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'VelocityValue))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ReturnVelocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'PDvelocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ForceSensortorque))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Torque>) istream)
  "Deserializes a message object of type '<Torque>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'TorqueValue) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ReturnTorqueValue) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'VelocityValue) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ReturnVelocity) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'PDvelocity) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ForceSensortorque) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Torque>)))
  "Returns string type for a message object of type '<Torque>"
  "can_ankle/Torque")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Torque)))
  "Returns string type for a message object of type 'Torque"
  "can_ankle/Torque")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Torque>)))
  "Returns md5sum for a message object of type '<Torque>"
  "f1c893b945ede6496888af3eab4a52a8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Torque)))
  "Returns md5sum for a message object of type 'Torque"
  "f1c893b945ede6496888af3eab4a52a8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Torque>)))
  "Returns full string definition for message of type '<Torque>"
  (cl:format cl:nil "float64 TorqueValue~%float64 ReturnTorqueValue~%float64 VelocityValue~%float64 ReturnVelocity~%float64 PDvelocity~%float32 ForceSensortorque~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Torque)))
  "Returns full string definition for message of type 'Torque"
  (cl:format cl:nil "float64 TorqueValue~%float64 ReturnTorqueValue~%float64 VelocityValue~%float64 ReturnVelocity~%float64 PDvelocity~%float32 ForceSensortorque~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Torque>))
  (cl:+ 0
     8
     8
     8
     8
     8
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Torque>))
  "Converts a ROS message object to a list"
  (cl:list 'Torque
    (cl:cons ':TorqueValue (TorqueValue msg))
    (cl:cons ':ReturnTorqueValue (ReturnTorqueValue msg))
    (cl:cons ':VelocityValue (VelocityValue msg))
    (cl:cons ':ReturnVelocity (ReturnVelocity msg))
    (cl:cons ':PDvelocity (PDvelocity msg))
    (cl:cons ':ForceSensortorque (ForceSensortorque msg))
))
