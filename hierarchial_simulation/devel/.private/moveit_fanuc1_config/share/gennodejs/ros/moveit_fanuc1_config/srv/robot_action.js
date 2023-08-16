// Auto-generated. Do not edit!

// (in-package moveit_fanuc1_config.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class robot_actionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.des = null;
      this.src = null;
    }
    else {
      if (initObj.hasOwnProperty('des')) {
        this.des = initObj.des
      }
      else {
        this.des = new geometry_msgs.msg.PoseStamped();
      }
      if (initObj.hasOwnProperty('src')) {
        this.src = initObj.src
      }
      else {
        this.src = new geometry_msgs.msg.PoseStamped();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type robot_actionRequest
    // Serialize message field [des]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.des, buffer, bufferOffset);
    // Serialize message field [src]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.src, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type robot_actionRequest
    let len;
    let data = new robot_actionRequest(null);
    // Deserialize message field [des]
    data.des = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [src]
    data.src = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.des);
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.src);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'moveit_fanuc1_config/robot_actionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4c3cb3bfc4ebe0aeb938436ad71e3af2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/PoseStamped des
    geometry_msgs/PoseStamped src
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
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
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new robot_actionRequest(null);
    if (msg.des !== undefined) {
      resolved.des = geometry_msgs.msg.PoseStamped.Resolve(msg.des)
    }
    else {
      resolved.des = new geometry_msgs.msg.PoseStamped()
    }

    if (msg.src !== undefined) {
      resolved.src = geometry_msgs.msg.PoseStamped.Resolve(msg.src)
    }
    else {
      resolved.src = new geometry_msgs.msg.PoseStamped()
    }

    return resolved;
    }
};

class robot_actionResponse {
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
    // Serializes a message object of type robot_actionResponse
    // Serialize message field [finished]
    bufferOffset = _serializer.bool(obj.finished, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type robot_actionResponse
    let len;
    let data = new robot_actionResponse(null);
    // Deserialize message field [finished]
    data.finished = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'moveit_fanuc1_config/robot_actionResponse';
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
    const resolved = new robot_actionResponse(null);
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
  Request: robot_actionRequest,
  Response: robot_actionResponse,
  md5sum() { return '19a560c38819713863fdccb541e95a18'; },
  datatype() { return 'moveit_fanuc1_config/robot_action'; }
};
