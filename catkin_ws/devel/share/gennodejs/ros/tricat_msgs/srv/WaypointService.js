// Auto-generated. Do not edit!

// (in-package tricat_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let WPList = require('../msg/WPList.js');

//-----------------------------------------------------------

class WaypointServiceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WaypointServiceRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WaypointServiceRequest
    let len;
    let data = new WaypointServiceRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tricat_msgs/WaypointServiceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Request
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WaypointServiceRequest(null);
    return resolved;
    }
};

class WaypointServiceResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.waypoint_list = null;
    }
    else {
      if (initObj.hasOwnProperty('waypoint_list')) {
        this.waypoint_list = initObj.waypoint_list
      }
      else {
        this.waypoint_list = new WPList();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WaypointServiceResponse
    // Serialize message field [waypoint_list]
    bufferOffset = WPList.serialize(obj.waypoint_list, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WaypointServiceResponse
    let len;
    let data = new WaypointServiceResponse(null);
    // Deserialize message field [waypoint_list]
    data.waypoint_list = WPList.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += WPList.getMessageSize(object.waypoint_list);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tricat_msgs/WaypointServiceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bbca728dc1a971940e618ed2a281f3e2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Response
    WPList waypoint_list
    
    ================================================================================
    MSG: tricat_msgs/WPList
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
    const resolved = new WaypointServiceResponse(null);
    if (msg.waypoint_list !== undefined) {
      resolved.waypoint_list = WPList.Resolve(msg.waypoint_list)
    }
    else {
      resolved.waypoint_list = new WPList()
    }

    return resolved;
    }
};

module.exports = {
  Request: WaypointServiceRequest,
  Response: WaypointServiceResponse,
  md5sum() { return 'bbca728dc1a971940e618ed2a281f3e2'; },
  datatype() { return 'tricat_msgs/WaypointService'; }
};
