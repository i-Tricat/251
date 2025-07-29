// Auto-generated. Do not edit!

// (in-package tricat_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Obstacle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.begin = null;
      this.end = null;
    }
    else {
      if (initObj.hasOwnProperty('begin')) {
        this.begin = initObj.begin
      }
      else {
        this.begin = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('end')) {
        this.end = initObj.end
      }
      else {
        this.end = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Obstacle
    // Serialize message field [begin]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.begin, buffer, bufferOffset);
    // Serialize message field [end]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.end, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Obstacle
    let len;
    let data = new Obstacle(null);
    // Deserialize message field [begin]
    data.begin = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [end]
    data.end = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tricat_msgs/Obstacle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '41e958a799f3b38d26f0715b99f1cec2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point begin
    geometry_msgs/Point end
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Obstacle(null);
    if (msg.begin !== undefined) {
      resolved.begin = geometry_msgs.msg.Point.Resolve(msg.begin)
    }
    else {
      resolved.begin = new geometry_msgs.msg.Point()
    }

    if (msg.end !== undefined) {
      resolved.end = geometry_msgs.msg.Point.Resolve(msg.end)
    }
    else {
      resolved.end = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

module.exports = Obstacle;
