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
    :initform cl:nil)
   (mTooClose
    :reader mTooClose
    :initarg :mTooClose
    :type cl:boolean
    :initform cl:nil)
   (mTooFar
    :reader mTooFar
    :initarg :mTooFar
    :type cl:boolean
    :initform cl:nil)
   (mTooClockwise
    :reader mTooClockwise
    :initarg :mTooClockwise
    :type cl:boolean
    :initform cl:nil)
   (mTooCClockwise
    :reader mTooCClockwise
    :initarg :mTooCClockwise
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

(cl:ensure-generic-function 'mTooClose-val :lambda-list '(m))
(cl:defmethod mTooClose-val ((m <PoseData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cse_kinect-msg:mTooClose-val is deprecated.  Use cse_kinect-msg:mTooClose instead.")
  (mTooClose m))

(cl:ensure-generic-function 'mTooFar-val :lambda-list '(m))
(cl:defmethod mTooFar-val ((m <PoseData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cse_kinect-msg:mTooFar-val is deprecated.  Use cse_kinect-msg:mTooFar instead.")
  (mTooFar m))

(cl:ensure-generic-function 'mTooClockwise-val :lambda-list '(m))
(cl:defmethod mTooClockwise-val ((m <PoseData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cse_kinect-msg:mTooClockwise-val is deprecated.  Use cse_kinect-msg:mTooClockwise instead.")
  (mTooClockwise m))

(cl:ensure-generic-function 'mTooCClockwise-val :lambda-list '(m))
(cl:defmethod mTooCClockwise-val ((m <PoseData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cse_kinect-msg:mTooCClockwise-val is deprecated.  Use cse_kinect-msg:mTooCClockwise instead.")
  (mTooCClockwise m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PoseData>) ostream)
  "Serializes a message object of type '<PoseData>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'mStop) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'mGo) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'mTooClose) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'mTooFar) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'mTooClockwise) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'mTooCClockwise) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PoseData>) istream)
  "Deserializes a message object of type '<PoseData>"
    (cl:setf (cl:slot-value msg 'mStop) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'mGo) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'mTooClose) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'mTooFar) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'mTooClockwise) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'mTooCClockwise) (cl:not (cl:zerop (cl:read-byte istream))))
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
  "b0bc174e6016dc1293495107c6428485")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PoseData)))
  "Returns md5sum for a message object of type 'PoseData"
  "b0bc174e6016dc1293495107c6428485")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PoseData>)))
  "Returns full string definition for message of type '<PoseData>"
  (cl:format cl:nil "bool mStop~%bool mGo~%bool mTooClose~%bool mTooFar~%bool mTooClockwise~%bool mTooCClockwise~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PoseData)))
  "Returns full string definition for message of type 'PoseData"
  (cl:format cl:nil "bool mStop~%bool mGo~%bool mTooClose~%bool mTooFar~%bool mTooClockwise~%bool mTooCClockwise~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PoseData>))
  (cl:+ 0
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PoseData>))
  "Converts a ROS message object to a list"
  (cl:list 'PoseData
    (cl:cons ':mStop (mStop msg))
    (cl:cons ':mGo (mGo msg))
    (cl:cons ':mTooClose (mTooClose msg))
    (cl:cons ':mTooFar (mTooFar msg))
    (cl:cons ':mTooClockwise (mTooClockwise msg))
    (cl:cons ':mTooCClockwise (mTooCClockwise msg))
))
