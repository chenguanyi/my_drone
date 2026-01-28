// Auto-generated. Do not edit!

// (in-package yolo.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class number {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.number = null;
      this.MaxH = null;
      this.MinH = null;
      this.CenterH = null;
      this.AverageH = null;
      this.x = null;
      this.y = null;
    }
    else {
      if (initObj.hasOwnProperty('number')) {
        this.number = initObj.number
      }
      else {
        this.number = 0;
      }
      if (initObj.hasOwnProperty('MaxH')) {
        this.MaxH = initObj.MaxH
      }
      else {
        this.MaxH = 0.0;
      }
      if (initObj.hasOwnProperty('MinH')) {
        this.MinH = initObj.MinH
      }
      else {
        this.MinH = 0.0;
      }
      if (initObj.hasOwnProperty('CenterH')) {
        this.CenterH = initObj.CenterH
      }
      else {
        this.CenterH = 0.0;
      }
      if (initObj.hasOwnProperty('AverageH')) {
        this.AverageH = initObj.AverageH
      }
      else {
        this.AverageH = 0.0;
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type number
    // Serialize message field [number]
    bufferOffset = _serializer.uint8(obj.number, buffer, bufferOffset);
    // Serialize message field [MaxH]
    bufferOffset = _serializer.float32(obj.MaxH, buffer, bufferOffset);
    // Serialize message field [MinH]
    bufferOffset = _serializer.float32(obj.MinH, buffer, bufferOffset);
    // Serialize message field [CenterH]
    bufferOffset = _serializer.float32(obj.CenterH, buffer, bufferOffset);
    // Serialize message field [AverageH]
    bufferOffset = _serializer.float32(obj.AverageH, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.uint8(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.uint8(obj.y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type number
    let len;
    let data = new number(null);
    // Deserialize message field [number]
    data.number = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [MaxH]
    data.MaxH = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [MinH]
    data.MinH = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [CenterH]
    data.CenterH = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [AverageH]
    data.AverageH = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 19;
  }

  static datatype() {
    // Returns string type for a message object
    return 'yolo/number';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8414b5a969236b037efc4bd669e714c5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 number
    float32 MaxH
    float32 MinH
    float32 CenterH
    float32 AverageH
    uint8 x
    uint8 y
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new number(null);
    if (msg.number !== undefined) {
      resolved.number = msg.number;
    }
    else {
      resolved.number = 0
    }

    if (msg.MaxH !== undefined) {
      resolved.MaxH = msg.MaxH;
    }
    else {
      resolved.MaxH = 0.0
    }

    if (msg.MinH !== undefined) {
      resolved.MinH = msg.MinH;
    }
    else {
      resolved.MinH = 0.0
    }

    if (msg.CenterH !== undefined) {
      resolved.CenterH = msg.CenterH;
    }
    else {
      resolved.CenterH = 0.0
    }

    if (msg.AverageH !== undefined) {
      resolved.AverageH = msg.AverageH;
    }
    else {
      resolved.AverageH = 0.0
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0
    }

    return resolved;
    }
};

module.exports = number;
