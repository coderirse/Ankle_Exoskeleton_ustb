;; Auto-generated. Do not edit!


(when (boundp 'can_ankle::ForceSensor)
  (if (not (find-package "CAN_ANKLE"))
    (make-package "CAN_ANKLE"))
  (shadow 'ForceSensor (find-package "CAN_ANKLE")))
(unless (find-package "CAN_ANKLE::FORCESENSOR")
  (make-package "CAN_ANKLE::FORCESENSOR"))

(in-package "ROS")
;;//! \htmlinclude ForceSensor.msg.html


(defclass can_ankle::ForceSensor
  :super ros::object
  :slots (_ForceValue ))

(defmethod can_ankle::ForceSensor
  (:init
   (&key
    ((:ForceValue __ForceValue) 0.0)
    )
   (send-super :init)
   (setq _ForceValue (float __ForceValue))
   self)
  (:ForceValue
   (&optional __ForceValue)
   (if __ForceValue (setq _ForceValue __ForceValue)) _ForceValue)
  (:serialization-length
   ()
   (+
    ;; float32 _ForceValue
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _ForceValue
       (sys::poke _ForceValue (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _ForceValue
     (setq _ForceValue (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get can_ankle::ForceSensor :md5sum-) "0610f446d964ed45a89ead06523db642")
(setf (get can_ankle::ForceSensor :datatype-) "can_ankle/ForceSensor")
(setf (get can_ankle::ForceSensor :definition-)
      "float32 ForceValue


")



(provide :can_ankle/ForceSensor "0610f446d964ed45a89ead06523db642")


