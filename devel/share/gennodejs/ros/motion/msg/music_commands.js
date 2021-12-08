// Auto-generated. Do not edit!

// (in-package motion.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class music_commands {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.tempo = null;
      this.right_arm_motions = null;
      this.left_arm_motions = null;
    }
    else {
      if (initObj.hasOwnProperty('tempo')) {
        this.tempo = initObj.tempo
      }
      else {
        this.tempo = [];
      }
      if (initObj.hasOwnProperty('right_arm_motions')) {
        this.right_arm_motions = initObj.right_arm_motions
      }
      else {
        this.right_arm_motions = [];
      }
      if (initObj.hasOwnProperty('left_arm_motions')) {
        this.left_arm_motions = initObj.left_arm_motions
      }
      else {
        this.left_arm_motions = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type music_commands
    // Serialize message field [tempo]
    bufferOffset = _arraySerializer.float64(obj.tempo, buffer, bufferOffset, null);
    // Serialize message field [right_arm_motions]
    bufferOffset = _arraySerializer.string(obj.right_arm_motions, buffer, bufferOffset, null);
    // Serialize message field [left_arm_motions]
    bufferOffset = _arraySerializer.string(obj.left_arm_motions, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type music_commands
    let len;
    let data = new music_commands(null);
    // Deserialize message field [tempo]
    data.tempo = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [right_arm_motions]
    data.right_arm_motions = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [left_arm_motions]
    data.left_arm_motions = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.tempo.length;
    object.right_arm_motions.forEach((val) => {
      length += 4 + val.length;
    });
    object.left_arm_motions.forEach((val) => {
      length += 4 + val.length;
    });
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'motion/music_commands';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ed9308da9714234f1e05e7985bf4f8c3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] tempo
    string[] right_arm_motions
    string[] left_arm_motions
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new music_commands(null);
    if (msg.tempo !== undefined) {
      resolved.tempo = msg.tempo;
    }
    else {
      resolved.tempo = []
    }

    if (msg.right_arm_motions !== undefined) {
      resolved.right_arm_motions = msg.right_arm_motions;
    }
    else {
      resolved.right_arm_motions = []
    }

    if (msg.left_arm_motions !== undefined) {
      resolved.left_arm_motions = msg.left_arm_motions;
    }
    else {
      resolved.left_arm_motions = []
    }

    return resolved;
    }
};

module.exports = music_commands;
