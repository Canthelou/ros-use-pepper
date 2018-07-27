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

class image_request {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.task = null;
      this.image_name = null;
      this.image_data = null;
    }
    else {
      if (initObj.hasOwnProperty('task')) {
        this.task = initObj.task
      }
      else {
        this.task = '';
      }
      if (initObj.hasOwnProperty('image_name')) {
        this.image_name = initObj.image_name
      }
      else {
        this.image_name = '';
      }
      if (initObj.hasOwnProperty('image_data')) {
        this.image_data = initObj.image_data
      }
      else {
        this.image_data = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type image_request
    // Serialize message field [task]
    bufferOffset = _serializer.string(obj.task, buffer, bufferOffset);
    // Serialize message field [image_name]
    bufferOffset = _serializer.string(obj.image_name, buffer, bufferOffset);
    // Serialize message field [image_data]
    bufferOffset = _arraySerializer.uint8(obj.image_data, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type image_request
    let len;
    let data = new image_request(null);
    // Deserialize message field [task]
    data.task = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [image_name]
    data.image_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [image_data]
    data.image_data = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.task.length;
    length += object.image_name.length;
    length += object.image_data.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pepper_tracking/image_request';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b557edbbfe4bf3b00801f9e1895f014e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string task
    string image_name
    uint8[] image_data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new image_request(null);
    if (msg.task !== undefined) {
      resolved.task = msg.task;
    }
    else {
      resolved.task = ''
    }

    if (msg.image_name !== undefined) {
      resolved.image_name = msg.image_name;
    }
    else {
      resolved.image_name = ''
    }

    if (msg.image_data !== undefined) {
      resolved.image_data = msg.image_data;
    }
    else {
      resolved.image_data = []
    }

    return resolved;
    }
};

module.exports = image_request;
