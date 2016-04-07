; Auto-generated. Do not edit!


(cl:in-package fcu_common-msg)


;//! \htmlinclude FW_State.msg.html

(cl:defclass <FW_State> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (Va
    :reader Va
    :initarg :Va
    :type cl:float
    :initform 0.0)
   (alpha
    :reader alpha
    :initarg :alpha
    :type cl:float
    :initform 0.0)
   (beta
    :reader beta
    :initarg :beta
    :type cl:float
    :initform 0.0)
   (phi
    :reader phi
    :initarg :phi
    :type cl:float
    :initform 0.0)
   (theta
    :reader theta
    :initarg :theta
    :type cl:float
    :initform 0.0)
   (psi
    :reader psi
    :initarg :psi
    :type cl:float
    :initform 0.0)
   (chi
    :reader chi
    :initarg :chi
    :type cl:float
    :initform 0.0)
   (p
    :reader p
    :initarg :p
    :type cl:float
    :initform 0.0)
   (q
    :reader q
    :initarg :q
    :type cl:float
    :initform 0.0)
   (r
    :reader r
    :initarg :r
    :type cl:float
    :initform 0.0)
   (Vg
    :reader Vg
    :initarg :Vg
    :type cl:float
    :initform 0.0)
   (wn
    :reader wn
    :initarg :wn
    :type cl:float
    :initform 0.0)
   (we
    :reader we
    :initarg :we
    :type cl:float
    :initform 0.0)
   (quat
    :reader quat
    :initarg :quat
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (quat_valid
    :reader quat_valid
    :initarg :quat_valid
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass FW_State (<FW_State>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FW_State>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FW_State)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fcu_common-msg:<FW_State> is deprecated: use fcu_common-msg:FW_State instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <FW_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:position-val is deprecated.  Use fcu_common-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'Va-val :lambda-list '(m))
(cl:defmethod Va-val ((m <FW_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:Va-val is deprecated.  Use fcu_common-msg:Va instead.")
  (Va m))

(cl:ensure-generic-function 'alpha-val :lambda-list '(m))
(cl:defmethod alpha-val ((m <FW_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:alpha-val is deprecated.  Use fcu_common-msg:alpha instead.")
  (alpha m))

(cl:ensure-generic-function 'beta-val :lambda-list '(m))
(cl:defmethod beta-val ((m <FW_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:beta-val is deprecated.  Use fcu_common-msg:beta instead.")
  (beta m))

(cl:ensure-generic-function 'phi-val :lambda-list '(m))
(cl:defmethod phi-val ((m <FW_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:phi-val is deprecated.  Use fcu_common-msg:phi instead.")
  (phi m))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <FW_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:theta-val is deprecated.  Use fcu_common-msg:theta instead.")
  (theta m))

(cl:ensure-generic-function 'psi-val :lambda-list '(m))
(cl:defmethod psi-val ((m <FW_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:psi-val is deprecated.  Use fcu_common-msg:psi instead.")
  (psi m))

(cl:ensure-generic-function 'chi-val :lambda-list '(m))
(cl:defmethod chi-val ((m <FW_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:chi-val is deprecated.  Use fcu_common-msg:chi instead.")
  (chi m))

(cl:ensure-generic-function 'p-val :lambda-list '(m))
(cl:defmethod p-val ((m <FW_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:p-val is deprecated.  Use fcu_common-msg:p instead.")
  (p m))

(cl:ensure-generic-function 'q-val :lambda-list '(m))
(cl:defmethod q-val ((m <FW_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:q-val is deprecated.  Use fcu_common-msg:q instead.")
  (q m))

(cl:ensure-generic-function 'r-val :lambda-list '(m))
(cl:defmethod r-val ((m <FW_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:r-val is deprecated.  Use fcu_common-msg:r instead.")
  (r m))

(cl:ensure-generic-function 'Vg-val :lambda-list '(m))
(cl:defmethod Vg-val ((m <FW_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:Vg-val is deprecated.  Use fcu_common-msg:Vg instead.")
  (Vg m))

(cl:ensure-generic-function 'wn-val :lambda-list '(m))
(cl:defmethod wn-val ((m <FW_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:wn-val is deprecated.  Use fcu_common-msg:wn instead.")
  (wn m))

(cl:ensure-generic-function 'we-val :lambda-list '(m))
(cl:defmethod we-val ((m <FW_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:we-val is deprecated.  Use fcu_common-msg:we instead.")
  (we m))

(cl:ensure-generic-function 'quat-val :lambda-list '(m))
(cl:defmethod quat-val ((m <FW_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:quat-val is deprecated.  Use fcu_common-msg:quat instead.")
  (quat m))

(cl:ensure-generic-function 'quat_valid-val :lambda-list '(m))
(cl:defmethod quat_valid-val ((m <FW_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fcu_common-msg:quat_valid-val is deprecated.  Use fcu_common-msg:quat_valid instead.")
  (quat_valid m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FW_State>) ostream)
  "Serializes a message object of type '<FW_State>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'position))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Va))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'alpha))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'beta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'phi))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'psi))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'chi))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'p))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'q))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'r))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Vg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'wn))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'we))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'quat))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'quat_valid) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FW_State>) istream)
  "Deserializes a message object of type '<FW_State>"
  (cl:setf (cl:slot-value msg 'position) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'position)))
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
    (cl:setf (cl:slot-value msg 'Va) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'alpha) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'beta) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'phi) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'psi) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'chi) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'p) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'q) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'r) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Vg) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wn) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'we) (roslisp-utils:decode-single-float-bits bits)))
  (cl:setf (cl:slot-value msg 'quat) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'quat)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:setf (cl:slot-value msg 'quat_valid) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FW_State>)))
  "Returns string type for a message object of type '<FW_State>"
  "fcu_common/FW_State")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FW_State)))
  "Returns string type for a message object of type 'FW_State"
  "fcu_common/FW_State")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FW_State>)))
  "Returns md5sum for a message object of type '<FW_State>"
  "68d0c7e8c59772a7dff6c1724b7b01b6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FW_State)))
  "Returns md5sum for a message object of type 'FW_State"
  "68d0c7e8c59772a7dff6c1724b7b01b6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FW_State>)))
  "Returns full string definition for message of type '<FW_State>"
  (cl:format cl:nil "# Vehicle state 'x_hat' output from the estimator or from simulator ~%~%# @warning roll, pitch and yaw have always to be valid, the quaternion is optional~%float32[3] position	# north, east, down (m)~%float32 Va		# Airspeed (m/s)~%float32 alpha		# Angle of attack (rad)~%float32 beta		# Slide slip angle (rad)~%float32 phi		# Roll angle (rad)~%float32 theta		# Pitch angle (rad)~%float32 psi		# Yaw angle (rad)~%float32 chi		# Course angle (rad)~%float32 p		# Body frame rollrate (rad/s)~%float32 q		# Body frame pitchrate (rad/s)~%float32 r		# Body frame yawrate (rad/s)~%float32 Vg		# Groundspeed (m/s)~%float32 wn		# Windspeed north component (m/s)~%float32 we		# Windspeed east component (m/s)~%float32[4] quat		# Quaternion (wxyz, NED)~%bool quat_valid		# Quaternion valid~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FW_State)))
  "Returns full string definition for message of type 'FW_State"
  (cl:format cl:nil "# Vehicle state 'x_hat' output from the estimator or from simulator ~%~%# @warning roll, pitch and yaw have always to be valid, the quaternion is optional~%float32[3] position	# north, east, down (m)~%float32 Va		# Airspeed (m/s)~%float32 alpha		# Angle of attack (rad)~%float32 beta		# Slide slip angle (rad)~%float32 phi		# Roll angle (rad)~%float32 theta		# Pitch angle (rad)~%float32 psi		# Yaw angle (rad)~%float32 chi		# Course angle (rad)~%float32 p		# Body frame rollrate (rad/s)~%float32 q		# Body frame pitchrate (rad/s)~%float32 r		# Body frame yawrate (rad/s)~%float32 Vg		# Groundspeed (m/s)~%float32 wn		# Windspeed north component (m/s)~%float32 we		# Windspeed east component (m/s)~%float32[4] quat		# Quaternion (wxyz, NED)~%bool quat_valid		# Quaternion valid~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FW_State>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'position) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'quat) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FW_State>))
  "Converts a ROS message object to a list"
  (cl:list 'FW_State
    (cl:cons ':position (position msg))
    (cl:cons ':Va (Va msg))
    (cl:cons ':alpha (alpha msg))
    (cl:cons ':beta (beta msg))
    (cl:cons ':phi (phi msg))
    (cl:cons ':theta (theta msg))
    (cl:cons ':psi (psi msg))
    (cl:cons ':chi (chi msg))
    (cl:cons ':p (p msg))
    (cl:cons ':q (q msg))
    (cl:cons ':r (r msg))
    (cl:cons ':Vg (Vg msg))
    (cl:cons ':wn (wn msg))
    (cl:cons ':we (we msg))
    (cl:cons ':quat (quat msg))
    (cl:cons ':quat_valid (quat_valid msg))
))
