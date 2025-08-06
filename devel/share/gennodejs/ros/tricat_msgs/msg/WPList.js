// Auto-generated. Do not edit!

// (in-package tricat_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let WP = require('./WP.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class WPList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.WP_data = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('WP_data')) {
        this.WP_data = initObj.WP_data
      }
      else {
        this.WP_data = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WPList
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [WP_data]
    // Serialize the length for message field [WP_data]
    bufferOffset = _serializer.uint32(obj.WP_data.length, buffer, bufferOffset);
    obj.WP_data.forEach((val) => {
      bufferOffset = WP.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WPList
    let len;
    let data = new WPList(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [WP_data]
    // Deserialize array length for message field [WP_data]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.WP_data = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.WP_data[i] = WP.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.WP_data.forEach((val) => {
      length += WP.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tricat_msgs/WPList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f6d33a8e2e98aba038773acbcf874e13';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    tricat_msgs/WP[] WP_data
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
    MSG: tricat_msgs/WP
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
    const resolved = new WPList(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.WP_data !== undefined) {
      resolved.WP_data = new Array(msg.WP_data.length);
      for (let i = 0; i < resolved.WP_data.length; ++i) {
        resolved.WP_data[i] = WP.Resolve(msg.WP_data[i]);
      }
    }
    else {
      resolved.WP_data = []
    }

    return resolved;
    }
};

module.exports = WPList;
