// Auto-generated. Do not edit!

// (in-package turtlebot_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Trajectory {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.x_values = null;
      this.y_values = null;
      this.headings = null;
      this.yaw_rates = null;
      this.durations = null;
      this.speeds = null;
      this.max_speed = null;
      this.max_accel = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('x_values')) {
        this.x_values = initObj.x_values
      }
      else {
        this.x_values = [];
      }
      if (initObj.hasOwnProperty('y_values')) {
        this.y_values = initObj.y_values
      }
      else {
        this.y_values = [];
      }
      if (initObj.hasOwnProperty('headings')) {
        this.headings = initObj.headings
      }
      else {
        this.headings = [];
      }
      if (initObj.hasOwnProperty('yaw_rates')) {
        this.yaw_rates = initObj.yaw_rates
      }
      else {
        this.yaw_rates = [];
      }
      if (initObj.hasOwnProperty('durations')) {
        this.durations = initObj.durations
      }
      else {
        this.durations = [];
      }
      if (initObj.hasOwnProperty('speeds')) {
        this.speeds = initObj.speeds
      }
      else {
        this.speeds = [];
      }
      if (initObj.hasOwnProperty('max_speed')) {
        this.max_speed = initObj.max_speed
      }
      else {
        this.max_speed = 0.0;
      }
      if (initObj.hasOwnProperty('max_accel')) {
        this.max_accel = initObj.max_accel
      }
      else {
        this.max_accel = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Trajectory
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [x_values]
    bufferOffset = _arraySerializer.float64(obj.x_values, buffer, bufferOffset, null);
    // Serialize message field [y_values]
    bufferOffset = _arraySerializer.float64(obj.y_values, buffer, bufferOffset, null);
    // Serialize message field [headings]
    bufferOffset = _arraySerializer.float64(obj.headings, buffer, bufferOffset, null);
    // Serialize message field [yaw_rates]
    bufferOffset = _arraySerializer.float64(obj.yaw_rates, buffer, bufferOffset, null);
    // Serialize message field [durations]
    bufferOffset = _arraySerializer.float64(obj.durations, buffer, bufferOffset, null);
    // Serialize message field [speeds]
    bufferOffset = _arraySerializer.float64(obj.speeds, buffer, bufferOffset, null);
    // Serialize message field [max_speed]
    bufferOffset = _serializer.float64(obj.max_speed, buffer, bufferOffset);
    // Serialize message field [max_accel]
    bufferOffset = _serializer.float64(obj.max_accel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Trajectory
    let len;
    let data = new Trajectory(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [x_values]
    data.x_values = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [y_values]
    data.y_values = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [headings]
    data.headings = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [yaw_rates]
    data.yaw_rates = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [durations]
    data.durations = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [speeds]
    data.speeds = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [max_speed]
    data.max_speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [max_accel]
    data.max_accel = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 8 * object.x_values.length;
    length += 8 * object.y_values.length;
    length += 8 * object.headings.length;
    length += 8 * object.yaw_rates.length;
    length += 8 * object.durations.length;
    length += 8 * object.speeds.length;
    return length + 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'turtlebot_msgs/Trajectory';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '110b624ce51252d0ba8b1a212a825dd2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    float64[] x_values
    float64[] y_values
    float64[] headings
    float64[] yaw_rates
    float64[] durations
    float64[] speeds
    float64 max_speed
    float64 max_accel
    
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
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Trajectory(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.x_values !== undefined) {
      resolved.x_values = msg.x_values;
    }
    else {
      resolved.x_values = []
    }

    if (msg.y_values !== undefined) {
      resolved.y_values = msg.y_values;
    }
    else {
      resolved.y_values = []
    }

    if (msg.headings !== undefined) {
      resolved.headings = msg.headings;
    }
    else {
      resolved.headings = []
    }

    if (msg.yaw_rates !== undefined) {
      resolved.yaw_rates = msg.yaw_rates;
    }
    else {
      resolved.yaw_rates = []
    }

    if (msg.durations !== undefined) {
      resolved.durations = msg.durations;
    }
    else {
      resolved.durations = []
    }

    if (msg.speeds !== undefined) {
      resolved.speeds = msg.speeds;
    }
    else {
      resolved.speeds = []
    }

    if (msg.max_speed !== undefined) {
      resolved.max_speed = msg.max_speed;
    }
    else {
      resolved.max_speed = 0.0
    }

    if (msg.max_accel !== undefined) {
      resolved.max_accel = msg.max_accel;
    }
    else {
      resolved.max_accel = 0.0
    }

    return resolved;
    }
};

module.exports = Trajectory;
