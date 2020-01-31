// Auto-generated. Do not edit!

// (in-package pangyo_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class figure {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.figure = null;
    }
    else {
      if (initObj.hasOwnProperty('figure')) {
        this.figure = initObj.figure
      }
      else {
        this.figure = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type figure
    // Serialize message field [figure]
    bufferOffset = _arraySerializer.int32(obj.figure, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type figure
    let len;
    let data = new figure(null);
    // Deserialize message field [figure]
    data.figure = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.figure.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pangyo_control/figure';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5aeb03f16c36d602af577d480cad4dd3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32[] figure
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new figure(null);
    if (msg.figure !== undefined) {
      resolved.figure = msg.figure;
    }
    else {
      resolved.figure = []
    }

    return resolved;
    }
};

module.exports = figure;
