; Auto-generated. Do not edit!


(cl:in-package fcu_common-msg)


;//! \htmlinclude FW_Controller_Commands.msg.html

(cl:defclass <FW_Controller_Commands> (roslisp-msg-protocol:ros-message)
  ((Va_c
    :reader Va_c
    :initarg :Va_c
    :type cl:float
    :initform 0.0)
   (h_c
    :reader h_c
    :initarg :h_c
    :type cl:float
    :initform 0.0)
   (chi_c
    :reader chi_c
    :initarg :chi_c
    :type cl:float
    :initform 0.0)
   (aux
    :reader aux
    :initarg :aux
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (aux_valid
    :reader aux_valid
    :initarg :aux_valid
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass FW_Controller_Commands (<FW_Controller_Commands>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FW_Controller_Commands>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FW_Controller_Commands)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fcu_common-msg:<FW_Controller_Commands> is deprecated: use fcu_common-msg:FW_Controller_Commands instead.")))

(cl:ensure-generic-function 'Va_c-val :lambda-list '(m))
(cl:defmethod Va_c-val ((m <FW_Controller_Commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:Va_c-val is deprecated.  Use fcu_common-msg:Va_c instead.")
  (Va_c m))

(cl:ensure-generic-function 'h_c-val :lambda-list '(m))
(cl:defmethod h_c-val ((m <FW_Controller_Commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:h_c-val is deprecated.  Use fcu_common-msg:h_c instead.")
  (h_c m))

(cl:ensure-generic-function 'chi_c-val :lambda-list '(m))
(cl:defmethod chi_c-val ((m <FW_Controller_Commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:chi_c-val is deprecated.  Use fcu_common-msg:chi_c instead.")
  (chi_c m))

(cl:ensure-generic-function 'aux-val :lambda-list '(m))
(cl:defmethod aux-val ((m <FW_Controller_Commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:aux-val is deprecated.  Use fcu_common-msg:aux instead.")
  (aux m))

(cl:ensure-generic-function 'aux_valid-val :lambda-list '(m))
(cl:defmethod aux_valid-val ((m <FW_Controller_Commands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:aux_valid-val is deprecated.  Use fcu_common-msg:aux_valid instead.")
  (aux_valid m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FW_Controller_Commands>) ostream)
  "Serializes a message object of type '<FW_Controller_Commands>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Va_c))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'h_c))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'chi_c))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'aux))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'aux_valid) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FW_Controller_Commands>) istream)
  "Deserializes a message object of type '<FW_Controller_Commands>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Va_c) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'h_c) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'chi_c) (roslisp-utils:decode-single-float-bits bits)))
  (cl:setf (cl:slot-value msg 'aux) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'aux)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:setf (cl:slot-value msg 'aux_valid) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FW_Controller_Commands>)))
  "Returns string type for a message object of type '<FW_Controller_Commands>"
  "fcu_common/FW_Controller_Commands")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FW_Controller_Commands)))
  "Returns string type for a message object of type 'FW_Controller_Commands"
  "fcu_common/FW_Controller_Commands")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FW_Controller_Commands>)))
  "Returns md5sum for a message object of type '<FW_Controller_Commands>"
  "866a52e318b720616a6342f870a0e13f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FW_Controller_Commands)))
  "Returns md5sum for a message object of type 'FW_Controller_Commands"
  "866a52e318b720616a6342f870a0e13f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FW_Controller_Commands>)))
  "Returns full string definition for message of type '<FW_Controller_Commands>"
  (cl:format cl:nil "# Controller commands output from the path follower, input to autopilot controller~%~%# @warning Va_c, h_c and chi_c have always to be valid, the aux array is optional~%float32 Va_c		# Commanded airspeed (m/s)~%float32 h_c		# Commanded altitude (m)~%float32 chi_c		# Commanded course (rad)~%float32[4] aux		# Optional auxiliary commands~%bool aux_valid		# Auxiliary commands valid~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FW_Controller_Commands)))
  "Returns full string definition for message of type 'FW_Controller_Commands"
  (cl:format cl:nil "# Controller commands output from the path follower, input to autopilot controller~%~%# @warning Va_c, h_c and chi_c have always to be valid, the aux array is optional~%float32 Va_c		# Commanded airspeed (m/s)~%float32 h_c		# Commanded altitude (m)~%float32 chi_c		# Commanded course (rad)~%float32[4] aux		# Optional auxiliary commands~%bool aux_valid		# Auxiliary commands valid~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FW_Controller_Commands>))
  (cl:+ 0
     4
     4
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'aux) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FW_Controller_Commands>))
  "Converts a ROS message object to a list"
  (cl:list 'FW_Controller_Commands
    (cl:cons ':Va_c (Va_c msg))
    (cl:cons ':h_c (h_c msg))
    (cl:cons ':chi_c (chi_c msg))
    (cl:cons ':aux (aux msg))
    (cl:cons ':aux_valid (aux_valid msg))
))
