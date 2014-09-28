; Auto-generated. Do not edit!


(cl:in-package epos2_control-msg)


;//! \htmlinclude HubFlow.msg.html

(cl:defclass <HubFlow> (roslisp-msg-protocol:ros-message)
  ((hub
    :reader hub
    :initarg :hub
    :type cl:float
    :initform 0.0)
   (flow
    :reader flow
    :initarg :flow
    :type cl:float
    :initform 0.0))
)

(cl:defclass HubFlow (<HubFlow>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HubFlow>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HubFlow)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name epos2_control-msg:<HubFlow> is deprecated: use epos2_control-msg:HubFlow instead.")))

(cl:ensure-generic-function 'hub-val :lambda-list '(m))
(cl:defmethod hub-val ((m <HubFlow>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader epos2_control-msg:hub-val is deprecated.  Use epos2_control-msg:hub instead.")
  (hub m))

(cl:ensure-generic-function 'flow-val :lambda-list '(m))
(cl:defmethod flow-val ((m <HubFlow>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader epos2_control-msg:flow-val is deprecated.  Use epos2_control-msg:flow instead.")
  (flow m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HubFlow>) ostream)
  "Serializes a message object of type '<HubFlow>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'hub))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'flow))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HubFlow>) istream)
  "Deserializes a message object of type '<HubFlow>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'hub) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'flow) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HubFlow>)))
  "Returns string type for a message object of type '<HubFlow>"
  "epos2_control/HubFlow")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HubFlow)))
  "Returns string type for a message object of type 'HubFlow"
  "epos2_control/HubFlow")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HubFlow>)))
  "Returns md5sum for a message object of type '<HubFlow>"
  "0937cbb35470b309d903626d01eaffea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HubFlow)))
  "Returns md5sum for a message object of type 'HubFlow"
  "0937cbb35470b309d903626d01eaffea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HubFlow>)))
  "Returns full string definition for message of type '<HubFlow>"
  (cl:format cl:nil "float32 hub~%float32 flow~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HubFlow)))
  "Returns full string definition for message of type 'HubFlow"
  (cl:format cl:nil "float32 hub~%float32 flow~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HubFlow>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HubFlow>))
  "Converts a ROS message object to a list"
  (cl:list 'HubFlow
    (cl:cons ':hub (hub msg))
    (cl:cons ':flow (flow msg))
))
