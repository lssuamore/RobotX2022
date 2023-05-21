// Auto-generated. Do not edit!

// (in-package jetson.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class task_info {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.state = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = new std_msgs.msg.Int64();
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type task_info
    // Serialize message field [name]
    bufferOffset = std_msgs.msg.Int64.serialize(obj.name, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.string(obj.state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type task_info
    let len;
    let data = new task_info(null);
    // Deserialize message field [name]
    data.name = std_msgs.msg.Int64.deserialize(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.state);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jetson/task_info';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8604b4549acbc2637d40681d3885d9ee';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Int64 name
    string state
    
    
    ================================================================================
    MSG: std_msgs/Int64
    int64 data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new task_info(null);
    if (msg.name !== undefined) {
      resolved.name = std_msgs.msg.Int64.Resolve(msg.name)
    }
    else {
      resolved.name = new std_msgs.msg.Int64()
    }

    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = ''
    }

    return resolved;
    }
};

module.exports = task_info;
