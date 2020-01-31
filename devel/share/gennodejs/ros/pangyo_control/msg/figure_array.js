// Auto-generated. Do not edit!

// (in-package pangyo_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let figure = require('./figure.js');

//-----------------------------------------------------------

class figure_array {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.figure_array = null;
    }
    else {
      if (initObj.hasOwnProperty('figure_array')) {
        this.figure_array = initObj.figure_array
      }
      else {
        this.figure_array = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type figure_array
    // Serialize message field [figure_array]
    // Serialize the length for message field [figure_array]
    bufferOffset = _serializer.uint32(obj.figure_array.length, buffer, bufferOffset);
    obj.figure_array.forEach((val) => {
      bufferOffset = figure.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type figure_array
    let len;
    let data = new figure_array(null);
    // Deserialize message field [figure_array]
    // Deserialize array length for message field [figure_array]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.figure_array = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.figure_array[i] = figure.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.figure_array.forEach((val) => {
      length += figure.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pangyo_control/figure_array';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '51ba417e9d022c3e94d411dca3690368';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    pangyo_control/figure[] figure_array
    
    ================================================================================
    MSG: pangyo_control/figure
    int32[] figure
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new figure_array(null);
    if (msg.figure_array !== undefined) {
      resolved.figure_array = new Array(msg.figure_array.length);
      for (let i = 0; i < resolved.figure_array.length; ++i) {
        resolved.figure_array[i] = figure.Resolve(msg.figure_array[i]);
      }
    }
    else {
      resolved.figure_array = []
    }

    return resolved;
    }
};

module.exports = figure_array;
