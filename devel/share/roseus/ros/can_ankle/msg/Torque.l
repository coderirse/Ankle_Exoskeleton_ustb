;; Auto-generated. Do not edit!


(when (boundp 'can_ankle::Torque)
  (if (not (find-package "CAN_ANKLE"))
    (make-package "CAN_ANKLE"))
  (shadow 'Torque (find-package "CAN_ANKLE")))
(unless (find-package "CAN_ANKLE::TORQUE")
  (make-package "CAN_ANKLE::TORQUE"))

(in-package "ROS")
;;//! \htmlinclude Torque.msg.html


(defclass can_ankle::Torque
  :super ros::object
  :slots (_TorqueValue _ReturnTorqueValue _VelocityValue _ReturnVelocity _PDvelocity _ForceSensortorque ))

(defmethod can_ankle::Torque
  (:init
   (&key
    ((:TorqueValue __TorqueValue) 0.0)
    ((:ReturnTorqueValue __ReturnTorqueValue) 0.0)
    ((:VelocityValue __VelocityValue) 0.0)
    ((:ReturnVelocity __ReturnVelocity) 0.0)
    ((:PDvelocity __PDvelocity) 0.0)
    ((:ForceSensortorque __ForceSensortorque) 0.0)
    )
   (send-super :init)
   (setq _TorqueValue (float __TorqueValue))
   (setq _ReturnTorqueValue (float __ReturnTorqueValue))
   (setq _VelocityValue (float __VelocityValue))
   (setq _ReturnVelocity (float __ReturnVelocity))
   (setq _PDvelocity (float __PDvelocity))
   (setq _ForceSensortorque (float __ForceSensortorque))
   self)
  (:TorqueValue
   (&optional __TorqueValue)
   (if __TorqueValue (setq _TorqueValue __TorqueValue)) _TorqueValue)
  (:ReturnTorqueValue
   (&optional __ReturnTorqueValue)
   (if __ReturnTorqueValue (setq _ReturnTorqueValue __ReturnTorqueValue)) _ReturnTorqueValue)
  (:VelocityValue
   (&optional __VelocityValue)
   (if __VelocityValue (setq _VelocityValue __VelocityValue)) _VelocityValue)
  (:ReturnVelocity
   (&optional __ReturnVelocity)
   (if __ReturnVelocity (setq _ReturnVelocity __ReturnVelocity)) _ReturnVelocity)
  (:PDvelocity
   (&optional __PDvelocity)
   (if __PDvelocity (setq _PDvelocity __PDvelocity)) _PDvelocity)
  (:ForceSensortorque
   (&optional __ForceSensortorque)
   (if __ForceSensortorque (setq _ForceSensortorque __ForceSensortorque)) _ForceSensortorque)
  (:serialization-length
   ()
   (+
    ;; float64 _TorqueValue
    8
    ;; float64 _ReturnTorqueValue
    8
    ;; float64 _VelocityValue
    8
    ;; float64 _ReturnVelocity
    8
    ;; float64 _PDvelocity
    8
    ;; float32 _ForceSensortorque
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _TorqueValue
       (sys::poke _TorqueValue (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ReturnTorqueValue
       (sys::poke _ReturnTorqueValue (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _VelocityValue
       (sys::poke _VelocityValue (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ReturnVelocity
       (sys::poke _ReturnVelocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _PDvelocity
       (sys::poke _PDvelocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _ForceSensortorque
       (sys::poke _ForceSensortorque (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _TorqueValue
     (setq _TorqueValue (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ReturnTorqueValue
     (setq _ReturnTorqueValue (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _VelocityValue
     (setq _VelocityValue (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ReturnVelocity
     (setq _ReturnVelocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _PDvelocity
     (setq _PDvelocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _ForceSensortorque
     (setq _ForceSensortorque (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get can_ankle::Torque :md5sum-) "f1c893b945ede6496888af3eab4a52a8")
(setf (get can_ankle::Torque :datatype-) "can_ankle/Torque")
(setf (get can_ankle::Torque :definition-)
      "float64 TorqueValue
float64 ReturnTorqueValue
float64 VelocityValue
float64 ReturnVelocity
float64 PDvelocity
float32 ForceSensortorque


")



(provide :can_ankle/Torque "f1c893b945ede6496888af3eab4a52a8")


