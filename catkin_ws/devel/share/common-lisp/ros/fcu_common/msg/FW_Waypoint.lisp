; Auto-generated. Do not edit!


(cl:in-package fcu_common-msg)


;//! \htmlinclude FW_Waypoint.msg.html

(cl:defclass <FW_Waypoint> (roslisp-msg-protocol:ros-message)
  ((w
    :reader w
    :initarg :w
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (chi_d
    :reader chi_d
    :initarg :chi_d
    :type cl:float
    :initform 0.0)
   (chi_valid
    :reader chi_valid
    :initarg :chi_valid
    :type cl:boolean
    :initform cl:nil)
   (Va_d
    :reader Va_d
    :initarg :Va_d
    :type cl:float
    :initform 0.0)
   (set_current
    :reader set_current
    :initarg :set_current
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass FW_Waypoint (<FW_Waypoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FW_Waypoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FW_Waypoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fcu_common-msg:<FW_Waypoint> is deprecated: use fcu_common-msg:FW_Waypoint instead.")))

(cl:ensure-generic-function 'w-val :lambda-list '(m))
(cl:defmethod w-val ((m <FW_Waypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:w-val is deprecated.  Use fcu_common-msg:w instead.")
  (w m))

(cl:ensure-generic-function 'chi_d-val :lambda-list '(m))
(cl:defmethod chi_d-val ((m <FW_Waypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:chi_d-val is deprecated.  Use fcu_common-msg:chi_d instead.")
  (chi_d m))

(cl:ensure-generic-function 'chi_valid-val :lambda-list '(m))
(cl:defmethod chi_valid-val ((m <FW_Waypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:chi_valid-val is deprecated.  Use fcu_common-msg:chi_valid instead.")
  (chi_valid m))

(cl:ensure-generic-function 'Va_d-val :lambda-list '(m))
(cl:defmethod Va_d-val ((m <FW_Waypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:Va_d-val is deprecated.  Use fcu_common-msg:Va_d instead.")
  (Va_d m))

(cl:ensure-generic-function 'set_current-val :lambda-list '(m))
(cl:defmethod set_current-val ((m <FW_Waypoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:set_current-val is deprecated.  Use fcu_common-msg:set_current instead.")
  (set_current m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FW_Waypoint>) ostream)
  "Serializes a message object of type '<FW_Waypoint>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'w))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'chi_d))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'chi_valid) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Va_d))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'set_current) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FW_Waypoint>) istream)
  "Deserializes a message object of type '<FW_Waypoint>"
  (cl:setf (cl:slot-value msg 'w) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'w)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'chi_d) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'chi_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Va_d) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'set_current) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FW_Waypoint>)))
  "Returns string type for a message object of type '<FW_Waypoint>"
  "fcu_common/FW_Waypoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FW_Waypoint)))
  "Returns string type for a message object of type 'FW_Waypoint"
  "fcu_common/FW_Waypoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FW_Waypoint>)))
  "Returns md5sum for a message object of type '<FW_Waypoint>"
  "7fa894c36e1adfc4bdc28e11056871e8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FW_Waypoint)))
  "Returns md5sum for a message object of type 'FW_Waypoint"
  "7fa894c36e1adfc4bdc28e11056871e8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FW_Waypoint>)))
  "Returns full string definition for message of type '<FW_Waypoint>"
  (cl:format cl:nil "# New waypoint, input to path manager~%~%# @warning w and set_current always have to be valid; the chi_d is optional.~%float32[3] w		# Waypoint in local NED (m)~%float32 chi_d		# Desired course at this waypoint (rad)~%bool chi_valid		# Desired course valid~%float32 Va_d		# Desired airspeed (m/s)~%bool set_current	# Sets this waypoint to be executed now!~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FW_Waypoint)))
  "Returns full string definition for message of type 'FW_Waypoint"
  (cl:format cl:nil "# New waypoint, input to path manager~%~%# @warning w and set_current always have to be valid; the chi_d is optional.~%float32[3] w		# Waypoint in local NED (m)~%float32 chi_d		# Desired course at this waypoint (rad)~%bool chi_valid		# Desired course valid~%float32 Va_d		# Desired airspeed (m/s)~%bool set_current	# Sets this waypoint to be executed now!~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FW_Waypoint>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'w) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     1
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FW_Waypoint>))
  "Converts a ROS message object to a list"
  (cl:list 'FW_Waypoint
    (cl:cons ':w (w msg))
    (cl:cons ':chi_d (chi_d msg))
    (cl:cons ':chi_valid (chi_valid msg))
    (cl:cons ':Va_d (Va_d msg))
    (cl:cons ':set_current (set_current msg))
))
