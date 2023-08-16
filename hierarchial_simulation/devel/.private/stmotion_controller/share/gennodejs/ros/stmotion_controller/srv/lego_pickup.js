// Auto-generated. Do not edit!

// (in-package stmotion_controller.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class lego_pickupRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.reference_frame = null;
      this.is_pick = null;
      this.pick_lego_name = null;
    }
    else {
      if (initObj.hasOwnProperty('reference_frame')) {
        this.reference_frame = initObj.reference_frame
      }
      else {
        this.reference_frame = '';
      }
      if (initObj.hasOwnProperty('is_pick')) {
        this.is_pick = initObj.is_pick
      }
      else {
        this.is_pick = '';
      }
      if (initObj.hasOwnProperty('pick_lego_name')) {
        this.pick_lego_name = initObj.pick_lego_name
      }
      else {
        this.pick_lego_name = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type lego_pickupRequest
    // Serialize message field [reference_frame]
    bufferOffset = _serializer.string(obj.reference_frame, buffer, bufferOffset);
    // Serialize message field [is_pick]
    bufferOffset = _serializer.string(obj.is_pick, buffer, bufferOffset);
    // Serialize message field [pick_lego_name]
    bufferOffset = _serializer.string(obj.pick_lego_name, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type lego_pickupRequest
    let len;
    let data = new lego_pickupRequest(null);
    // Deserialize message field [reference_frame]
    data.reference_frame = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [is_pick]
    data.is_pick = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [pick_lego_name]
    data.pick_lego_name = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.reference_frame);
    length += _getByteLength(object.is_pick);
    length += _getByteLength(object.pick_lego_name);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'stmotion_controller/lego_pickupRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3932247b94dffeeb281fcf442c51f24e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string reference_frame
    string is_pick
    string pick_lego_name
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new lego_pickupRequest(null);
    if (msg.reference_frame !== undefined) {
      resolved.reference_frame = msg.reference_frame;
    }
    else {
      resolved.reference_frame = ''
    }

    if (msg.is_pick !== undefined) {
      resolved.is_pick = msg.is_pick;
    }
    else {
      resolved.is_pick = ''
    }

    if (msg.pick_lego_name !== undefined) {
      resolved.pick_lego_name = msg.pick_lego_name;
    }
    else {
      resolved.pick_lego_name = ''
    }

    return resolved;
    }
};

class lego_pickupResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.finished = null;
    }
    else {
      if (initObj.hasOwnProperty('finished')) {
        this.finished = initObj.finished
      }
      else {
        this.finished = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type lego_pickupResponse
    // Serialize message field [finished]
    bufferOffset = _serializer.bool(obj.finished, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type lego_pickupResponse
    let len;
    let data = new lego_pickupResponse(null);
    // Deserialize message field [finished]
    data.finished = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'stmotion_controller/lego_pickupResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e2797c797e620a950ba704492d72873b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool finished
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new lego_pickupResponse(null);
    if (msg.finished !== undefined) {
      resolved.finished = msg.finished;
    }
    else {
      resolved.finished = false
    }

    return resolved;
    }
};

module.exports = {
  Request: lego_pickupRequest,
  Response: lego_pickupResponse,
  md5sum() { return '054657831fad2fea203e93682e8ffae0'; },
  datatype() { return 'stmotion_controller/lego_pickup'; }
};
