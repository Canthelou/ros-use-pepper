// Auto-generated. Do not edit!

// (in-package pepper_tracking.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class image_return {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.found = null;
      this.image_name = null;
    }
    else {
      if (initObj.hasOwnProperty('found')) {
        this.found = initObj.found
      }
      else {
        this.found = false;
      }
      if (initObj.hasOwnProperty('image_name')) {
        this.image_name = initObj.image_name
      }
      else {
        this.image_name = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type image_return
    // Serialize message field [found]
    bufferOffset = _serializer.bool(obj.found, buffer, bufferOffset);
    // Serialize message field [image_name]
    bufferOffset = _serializer.string(obj.image_name, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type image_return
    let len;
    let data = new image_return(null);
    // Deserialize message field [found]
    data.found = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [image_name]
    data.image_name = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.image_name.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pepper_tracking/image_return';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '68cd02969b39e785e4cdf1cf76db231c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool found
    string image_name
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new image_return(null);
    if (msg.found !== undefined) {
      resolved.found = msg.found;
    }
    else {
      resolved.found = false
    }

    if (msg.image_name !== undefined) {
      resolved.image_name = msg.image_name;
    }
    else {
      resolved.image_name = ''
    }

    return resolved;
    }
};

module.exports = image_return;
