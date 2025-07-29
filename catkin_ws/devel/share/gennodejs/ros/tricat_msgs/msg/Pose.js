// Auto-generated. Do not edit!

// (in-package tricat_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Pose {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.x = null;
      this.y = null;
      this.psi = null;
      this.u = null;
      this.v = null;
      this.r = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('psi')) {
        this.psi = initObj.psi
      }
      else {
        this.psi = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('u')) {
        this.u = initObj.u
      }
      else {
        this.u = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('v')) {
        this.v = initObj.v
      }
      else {
        this.v = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('r')) {
        this.r = initObj.r
      }
      else {
        this.r = new std_msgs.msg.Float64();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Pose
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.y, buffer, bufferOffset);
    // Serialize message field [psi]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.psi, buffer, bufferOffset);
    // Serialize message field [u]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.u, buffer, bufferOffset);
    // Serialize message field [v]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.v, buffer, bufferOffset);
    // Serialize message field [r]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.r, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Pose
    let len;
    let data = new Pose(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [psi]
    data.psi = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [u]
    data.u = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [v]
    data.v = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [r]
    data.r = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tricat_msgs/Pose';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7faf8df64baddc9fd0c262f0be990408';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    std_msgs/Float64 x
    std_msgs/Float64 y
    std_msgs/Float64 psi
    std_msgs/Float64 u
    std_msgs/Float64 v
    std_msgs/Float64 r
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: std_msgs/Float64
    float64 data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Pose(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.x !== undefined) {
      resolved.x = std_msgs.msg.Float64.Resolve(msg.x)
    }
    else {
      resolved.x = new std_msgs.msg.Float64()
    }

    if (msg.y !== undefined) {
      resolved.y = std_msgs.msg.Float64.Resolve(msg.y)
    }
    else {
      resolved.y = new std_msgs.msg.Float64()
    }

    if (msg.psi !== undefined) {
      resolved.psi = std_msgs.msg.Float64.Resolve(msg.psi)
    }
    else {
      resolved.psi = new std_msgs.msg.Float64()
    }

    if (msg.u !== undefined) {
      resolved.u = std_msgs.msg.Float64.Resolve(msg.u)
    }
    else {
      resolved.u = new std_msgs.msg.Float64()
    }

    if (msg.v !== undefined) {
      resolved.v = std_msgs.msg.Float64.Resolve(msg.v)
    }
    else {
      resolved.v = new std_msgs.msg.Float64()
    }

    if (msg.r !== undefined) {
      resolved.r = std_msgs.msg.Float64.Resolve(msg.r)
    }
    else {
      resolved.r = new std_msgs.msg.Float64()
    }

    return resolved;
    }
};

module.exports = Pose;
