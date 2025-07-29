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

class Control {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.servo_s = null;
      this.servo_p = null;
      this.thruster_p = null;
      this.thruster_s = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('servo_s')) {
        this.servo_s = initObj.servo_s
      }
      else {
        this.servo_s = new std_msgs.msg.UInt16();
      }
      if (initObj.hasOwnProperty('servo_p')) {
        this.servo_p = initObj.servo_p
      }
      else {
        this.servo_p = new std_msgs.msg.UInt16();
      }
      if (initObj.hasOwnProperty('thruster_p')) {
        this.thruster_p = initObj.thruster_p
      }
      else {
        this.thruster_p = new std_msgs.msg.UInt16();
      }
      if (initObj.hasOwnProperty('thruster_s')) {
        this.thruster_s = initObj.thruster_s
      }
      else {
        this.thruster_s = new std_msgs.msg.UInt16();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Control
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [servo_s]
    bufferOffset = std_msgs.msg.UInt16.serialize(obj.servo_s, buffer, bufferOffset);
    // Serialize message field [servo_p]
    bufferOffset = std_msgs.msg.UInt16.serialize(obj.servo_p, buffer, bufferOffset);
    // Serialize message field [thruster_p]
    bufferOffset = std_msgs.msg.UInt16.serialize(obj.thruster_p, buffer, bufferOffset);
    // Serialize message field [thruster_s]
    bufferOffset = std_msgs.msg.UInt16.serialize(obj.thruster_s, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Control
    let len;
    let data = new Control(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [servo_s]
    data.servo_s = std_msgs.msg.UInt16.deserialize(buffer, bufferOffset);
    // Deserialize message field [servo_p]
    data.servo_p = std_msgs.msg.UInt16.deserialize(buffer, bufferOffset);
    // Deserialize message field [thruster_p]
    data.thruster_p = std_msgs.msg.UInt16.deserialize(buffer, bufferOffset);
    // Deserialize message field [thruster_s]
    data.thruster_s = std_msgs.msg.UInt16.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tricat_msgs/Control';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b0e90ce225824dce6ea0018e8f0cbd3f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    std_msgs/UInt16 servo_s
    std_msgs/UInt16 servo_p
    std_msgs/UInt16 thruster_p
    std_msgs/UInt16 thruster_s
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
    MSG: std_msgs/UInt16
    uint16 data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Control(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.servo_s !== undefined) {
      resolved.servo_s = std_msgs.msg.UInt16.Resolve(msg.servo_s)
    }
    else {
      resolved.servo_s = new std_msgs.msg.UInt16()
    }

    if (msg.servo_p !== undefined) {
      resolved.servo_p = std_msgs.msg.UInt16.Resolve(msg.servo_p)
    }
    else {
      resolved.servo_p = new std_msgs.msg.UInt16()
    }

    if (msg.thruster_p !== undefined) {
      resolved.thruster_p = std_msgs.msg.UInt16.Resolve(msg.thruster_p)
    }
    else {
      resolved.thruster_p = new std_msgs.msg.UInt16()
    }

    if (msg.thruster_s !== undefined) {
      resolved.thruster_s = std_msgs.msg.UInt16.Resolve(msg.thruster_s)
    }
    else {
      resolved.thruster_s = new std_msgs.msg.UInt16()
    }

    return resolved;
    }
};

module.exports = Control;
