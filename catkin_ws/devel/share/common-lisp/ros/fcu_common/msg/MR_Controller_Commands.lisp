; Auto-generated. Do not edit!


(cl:in-package fcu_common-msg)


;//! \htmlinclude MR_Controller_Commands.msg.html

(cl:defclass <MR_Controller_Commands> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (acceleration
    :reader acceleration
    :initarg :acceleration
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (velocity
    :reader velocity
    :initarg :velocity
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (alpha
    :reader alpha
    :initarg :alpha
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (omega
    :reader omega
    :initarg :omega
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (theta
    :reader theta
    :initarg :theta
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (aux
    :reader aux
    :initarg :aux
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (position_valid
    :reader position_valid
    :initarg :position_valid
    :type cl:boolean
    :initform cl:nil)
   (velocity_valid
    :reader velocity_valid
    :initarg :velocity_valid
    :type cl:boolean
    :initform cl:nil)
   (acceleration_valid
    :reader acceleration_valid
    :initarg :acceleration_valid
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass MR_Controller_Commands (<MR_Controller_Commands>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MR_Controller_Commands>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MR_Controller_Commands)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fcu_common-msg:<MR_Controller_Commands> is deprecated: use fcu_common-msg:MR_Controller_Commands instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MR_Controller_Commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:header-val is deprecated.  Use fcu_common-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'acceleration-val :lambda-list '(m))
(cl:defmethod acceleration-val ((m <MR_Controller_Commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:acceleration-val is deprecated.  Use fcu_common-msg:acceleration instead.")
  (acceleration m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <MR_Controller_Commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:velocity-val is deprecated.  Use fcu_common-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <MR_Controller_Commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:position-val is deprecated.  Use fcu_common-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'alpha-val :lambda-list '(m))
(cl:defmethod alpha-val ((m <MR_Controller_Commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:alpha-val is deprecated.  Use fcu_common-msg:alpha instead.")
  (alpha m))

(cl:ensure-generic-function 'omega-val :lambda-list '(m))
(cl:defmethod omega-val ((m <MR_Controller_Commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:omega-val is deprecated.  Use fcu_common-msg:omega instead.")
  (omega m))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <MR_Controller_Commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:theta-val is deprecated.  Use fcu_common-msg:theta instead.")
  (theta m))

(cl:ensure-generic-function 'aux-val :lambda-list '(m))
(cl:defmethod aux-val ((m <MR_Controller_Commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:aux-val is deprecated.  Use fcu_common-msg:aux instead.")
  (aux m))

(cl:ensure-generic-function 'position_valid-val :lambda-list '(m))
(cl:defmethod position_valid-val ((m <MR_Controller_Commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:position_valid-val is deprecated.  Use fcu_common-msg:position_valid instead.")
  (position_valid m))

(cl:ensure-generic-function 'velocity_valid-val :lambda-list '(m))
(cl:defmethod velocity_valid-val ((m <MR_Controller_Commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:velocity_valid-val is deprecated.  Use fcu_common-msg:velocity_valid instead.")
  (velocity_valid m))

(cl:ensure-generic-function 'acceleration_valid-val :lambda-list '(m))
(cl:defmethod acceleration_valid-val ((m <MR_Controller_Commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:acceleration_valid-val is deprecated.  Use fcu_common-msg:acceleration_valid instead.")
  (acceleration_valid m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MR_Controller_Commands>) ostream)
  "Serializes a message object of type '<MR_Controller_Commands>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'acceleration) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'alpha) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'omega) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'theta) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'aux))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'position_valid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'velocity_valid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'acceleration_valid) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MR_Controller_Commands>) istream)
  "Deserializes a message object of type '<MR_Controller_Commands>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'acceleration) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'alpha) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'omega) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'theta) istream)
  (cl:setf (cl:slot-value msg 'aux) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'aux)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
    (cl:setf (cl:slot-value msg 'position_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'velocity_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'acceleration_valid) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MR_Controller_Commands>)))
  "Returns string type for a message object of type '<MR_Controller_Commands>"
  "fcu_common/MR_Controller_Commands")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MR_Controller_Commands)))
  "Returns string type for a message object of type 'MR_Controller_Commands"
  "fcu_common/MR_Controller_Commands")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MR_Controller_Commands>)))
  "Returns md5sum for a message object of type '<MR_Controller_Commands>"
  "11c19eeab1f78e012f469cf56504e52c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MR_Controller_Commands)))
  "Returns md5sum for a message object of type 'MR_Controller_Commands"
  "11c19eeab1f78e012f469cf56504e52c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MR_Controller_Commands>)))
  "Returns full string definition for message of type '<MR_Controller_Commands>"
  (cl:format cl:nil "# Controller commands to the controller of a multirotor~%~%Header header~%geometry_msgs/Vector3 acceleration # commanded acceleration~%geometry_msgs/Vector3 velocity # commanded velocity~%geometry_msgs/Vector3 position # commanded position~%geometry_msgs/Vector3 alpha # commanded angular acceleration~%geometry_msgs/Vector3 omega # commanded angular velocity~%geometry_msgs/Vector3 theta # commanded angle~%float64[4] aux		# Optional auxiliary commands~%~%bool position_valid~%bool velocity_valid~%bool acceleration_valid~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MR_Controller_Commands)))
  "Returns full string definition for message of type 'MR_Controller_Commands"
  (cl:format cl:nil "# Controller commands to the controller of a multirotor~%~%Header header~%geometry_msgs/Vector3 acceleration # commanded acceleration~%geometry_msgs/Vector3 velocity # commanded velocity~%geometry_msgs/Vector3 position # commanded position~%geometry_msgs/Vector3 alpha # commanded angular acceleration~%geometry_msgs/Vector3 omega # commanded angular velocity~%geometry_msgs/Vector3 theta # commanded angle~%float64[4] aux		# Optional auxiliary commands~%~%bool position_valid~%bool velocity_valid~%bool acceleration_valid~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MR_Controller_Commands>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'acceleration))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'alpha))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'omega))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'theta))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'aux) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MR_Controller_Commands>))
  "Converts a ROS message object to a list"
  (cl:list 'MR_Controller_Commands
    (cl:cons ':header (header msg))
    (cl:cons ':acceleration (acceleration msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':position (position msg))
    (cl:cons ':alpha (alpha msg))
    (cl:cons ':omega (omega msg))
    (cl:cons ':theta (theta msg))
    (cl:cons ':aux (aux msg))
    (cl:cons ':position_valid (position_valid msg))
    (cl:cons ':velocity_valid (velocity_valid msg))
    (cl:cons ':acceleration_valid (acceleration_valid msg))
))
