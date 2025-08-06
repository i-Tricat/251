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

class WP {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x = null;
      this.y = null;
      this.type = null;
      this.num = null;
      this.range = null;
      this.arrive = null;
    }
    else {
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
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = new std_msgs.msg.String();
      }
      if (initObj.hasOwnProperty('num')) {
        this.num = initObj.num
      }
      else {
        this.num = new std_msgs.msg.UInt16();
      }
      if (initObj.hasOwnProperty('range')) {
        this.range = initObj.range
      }
      else {
        this.range = new std_msgs.msg.UInt16();
      }
      if (initObj.hasOwnProperty('arrive')) {
        this.arrive = initObj.arrive
      }
      else {
        this.arrive = new std_msgs.msg.Bool();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WP
    // Serialize message field [x]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.y, buffer, bufferOffset);
    // Serialize message field [type]
    bufferOffset = std_msgs.msg.String.serialize(obj.type, buffer, bufferOffset);
    // Serialize message field [num]
    bufferOffset = std_msgs.msg.UInt16.serialize(obj.num, buffer, bufferOffset);
    // Serialize message field [range]
    bufferOffset = std_msgs.msg.UInt16.serialize(obj.range, buffer, bufferOffset);
    // Serialize message field [arrive]
    bufferOffset = std_msgs.msg.Bool.serialize(obj.arrive, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WP
    let len;
    let data = new WP(null);
    // Deserialize message field [x]
    data.x = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [type]
    data.type = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    // Deserialize message field [num]
    data.num = std_msgs.msg.UInt16.deserialize(buffer, bufferOffset);
    // Deserialize message field [range]
    data.range = std_msgs.msg.UInt16.deserialize(buffer, bufferOffset);
    // Deserialize message field [arrive]
    data.arrive = std_msgs.msg.Bool.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.String.getMessageSize(object.type);
    return length + 21;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tricat_msgs/WP';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e86d166212e6ad49f8865c7a6bb74d08';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Float64 x
    std_msgs/Float64 y
    std_msgs/String type
    std_msgs/UInt16 num
    std_msgs/UInt16 range
    std_msgs/Bool arrive
    ================================================================================
    MSG: std_msgs/Float64
    float64 data
    ================================================================================
    MSG: std_msgs/String
    string data
    
    ================================================================================
    MSG: std_msgs/UInt16
    uint16 data
    
    ================================================================================
    MSG: std_msgs/Bool
    bool data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WP(null);
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

    if (msg.type !== undefined) {
      resolved.type = std_msgs.msg.String.Resolve(msg.type)
    }
    else {
      resolved.type = new std_msgs.msg.String()
    }

    if (msg.num !== undefined) {
      resolved.num = std_msgs.msg.UInt16.Resolve(msg.num)
    }
    else {
      resolved.num = new std_msgs.msg.UInt16()
    }

    if (msg.range !== undefined) {
      resolved.range = std_msgs.msg.UInt16.Resolve(msg.range)
    }
    else {
      resolved.range = new std_msgs.msg.UInt16()
    }

    if (msg.arrive !== undefined) {
      resolved.arrive = std_msgs.msg.Bool.Resolve(msg.arrive)
    }
    else {
      resolved.arrive = new std_msgs.msg.Bool()
    }

    return resolved;
    }
};

module.exports = WP;
