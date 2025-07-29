// Auto-generated. Do not edit!

// (in-package tricat_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Obstacle = require('./Obstacle.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ObstacleList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.lidar_header = null;
      this.obstacle = null;
    }
    else {
      if (initObj.hasOwnProperty('lidar_header')) {
        this.lidar_header = initObj.lidar_header
      }
      else {
        this.lidar_header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('obstacle')) {
        this.obstacle = initObj.obstacle
      }
      else {
        this.obstacle = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObstacleList
    // Serialize message field [lidar_header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.lidar_header, buffer, bufferOffset);
    // Serialize message field [obstacle]
    // Serialize the length for message field [obstacle]
    bufferOffset = _serializer.uint32(obj.obstacle.length, buffer, bufferOffset);
    obj.obstacle.forEach((val) => {
      bufferOffset = Obstacle.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObstacleList
    let len;
    let data = new ObstacleList(null);
    // Deserialize message field [lidar_header]
    data.lidar_header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [obstacle]
    // Deserialize array length for message field [obstacle]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.obstacle = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.obstacle[i] = Obstacle.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.lidar_header);
    length += 48 * object.obstacle.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tricat_msgs/ObstacleList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e58b9c6e530a428f4aad1beec1ae1b08';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header lidar_header
    tricat_msgs/Obstacle[] obstacle
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
    MSG: tricat_msgs/Obstacle
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
    const resolved = new ObstacleList(null);
    if (msg.lidar_header !== undefined) {
      resolved.lidar_header = std_msgs.msg.Header.Resolve(msg.lidar_header)
    }
    else {
      resolved.lidar_header = new std_msgs.msg.Header()
    }

    if (msg.obstacle !== undefined) {
      resolved.obstacle = new Array(msg.obstacle.length);
      for (let i = 0; i < resolved.obstacle.length; ++i) {
        resolved.obstacle[i] = Obstacle.Resolve(msg.obstacle[i]);
      }
    }
    else {
      resolved.obstacle = []
    }

    return resolved;
    }
};

module.exports = ObstacleList;
