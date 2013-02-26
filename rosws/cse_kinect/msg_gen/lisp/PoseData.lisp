; Auto-generated. Do not edit!


(cl:in-package cse_kinect-msg)


;//! \htmlinclude PoseData.msg.html

(cl:defclass <PoseData> (roslisp-msg-protocol:ros-message)
  ((mStop
    :reader mStop
    :initarg :mStop
    :type cl:boolean
    :initform cl:nil)
   (mGo
    :reader mGo
    :initarg :mGo
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass PoseData (<PoseData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PoseData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PoseData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cse_kinect-msg:<PoseData> is deprecated: use cse_kinect-msg:PoseData instead.")))

(cl:ensure-generic-function 'mStop-val :lambda-list '(m))
(cl:defmethod mStop-val ((m <PoseData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cse_kinect-msg:mStop-val is deprecated.  Use cse_kinect-msg:mStop instead.")
  (mStop m))

(cl:ensure-generic-function 'mGo-val :lambda-list '(m))
(cl:defmethod mGo-val ((m <PoseData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cse_kinect-msg:mGo-val is deprecated.  Use cse_kinect-msg:mGo instead.")
  (mGo m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PoseData>) ostream)
  "Serializes a message object of type '<PoseData>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'mStop) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'mGo) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PoseData>) istream)
  "Deserializes a message object of type '<PoseData>"
    (cl:setf (cl:slot-value msg 'mStop) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'mGo) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PoseData>)))
  "Returns string type for a message object of type '<PoseData>"
  "cse_kinect/PoseData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PoseData)))
  "Returns string type for a message object of type 'PoseData"
  "cse_kinect/PoseData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PoseData>)))
  "Returns md5sum for a message object of type '<PoseData>"
  "652a0a9116dab48c1aaaa7a7a1f63840")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PoseData)))
  "Returns md5sum for a message object of type 'PoseData"
  "652a0a9116dab48c1aaaa7a7a1f63840")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PoseData>)))
  "Returns full string definition for message of type '<PoseData>"
  (cl:format cl:nil "bool mStop~%bool mGo~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PoseData)))
  "Returns full string definition for message of type 'PoseData"
  (cl:format cl:nil "bool mStop~%bool mGo~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PoseData>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PoseData>))
  "Converts a ROS message object to a list"
  (cl:list 'PoseData
    (cl:cons ':mStop (mStop msg))
    (cl:cons ':mGo (mGo msg))
))
