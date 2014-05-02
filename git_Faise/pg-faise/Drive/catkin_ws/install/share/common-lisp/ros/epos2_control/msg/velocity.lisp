; Auto-generated. Do not edit!


(cl:in-package epos2_control-msg)


;//! \htmlinclude velocity.msg.html

(cl:defclass <velocity> (roslisp-msg-protocol:ros-message)
  ((left
    :reader left
    :initarg :left
    :type cl:float
    :initform 0.0)
   (right
    :reader right
    :initarg :right
    :type cl:float
    :initform 0.0))
)

(cl:defclass velocity (<velocity>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <velocity>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'velocity)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name epos2_control-msg:<velocity> is deprecated: use epos2_control-msg:velocity instead.")))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <velocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader epos2_control-msg:left-val is deprecated.  Use epos2_control-msg:left instead.")
  (left m))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <velocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader epos2_control-msg:right-val is deprecated.  Use epos2_control-msg:right instead.")
  (right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <velocity>) ostream)
  "Serializes a message object of type '<velocity>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <velocity>) istream)
  "Deserializes a message object of type '<velocity>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<velocity>)))
  "Returns string type for a message object of type '<velocity>"
  "epos2_control/velocity")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'velocity)))
  "Returns string type for a message object of type 'velocity"
  "epos2_control/velocity")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<velocity>)))
  "Returns md5sum for a message object of type '<velocity>"
  "3a927990ab5d5c3d628e2d52b8533e52")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'velocity)))
  "Returns md5sum for a message object of type 'velocity"
  "3a927990ab5d5c3d628e2d52b8533e52")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<velocity>)))
  "Returns full string definition for message of type '<velocity>"
  (cl:format cl:nil "float32 left~%float32 right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'velocity)))
  "Returns full string definition for message of type 'velocity"
  (cl:format cl:nil "float32 left~%float32 right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <velocity>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <velocity>))
  "Converts a ROS message object to a list"
  (cl:list 'velocity
    (cl:cons ':left (left msg))
    (cl:cons ':right (right msg))
))
