// Auto-generated. Do not edit!

// (in-package can_ankle.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Torque {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.TorqueValue = null;
      this.ReturnTorqueValue = null;
      this.VelocityValue = null;
      this.ReturnVelocity = null;
      this.PDvelocity = null;
      this.ForceSensortorque = null;
    }
    else {
      if (initObj.hasOwnProperty('TorqueValue')) {
        this.TorqueValue = initObj.TorqueValue
      }
      else {
        this.TorqueValue = 0.0;
      }
      if (initObj.hasOwnProperty('ReturnTorqueValue')) {
        this.ReturnTorqueValue = initObj.ReturnTorqueValue
      }
      else {
        this.ReturnTorqueValue = 0.0;
      }
      if (initObj.hasOwnProperty('VelocityValue')) {
        this.VelocityValue = initObj.VelocityValue
      }
      else {
        this.VelocityValue = 0.0;
      }
      if (initObj.hasOwnProperty('ReturnVelocity')) {
        this.ReturnVelocity = initObj.ReturnVelocity
      }
      else {
        this.ReturnVelocity = 0.0;
      }
      if (initObj.hasOwnProperty('PDvelocity')) {
        this.PDvelocity = initObj.PDvelocity
      }
      else {
        this.PDvelocity = 0.0;
      }
      if (initObj.hasOwnProperty('ForceSensortorque')) {
        this.ForceSensortorque = initObj.ForceSensortorque
      }
      else {
        this.ForceSensortorque = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Torque
    // Serialize message field [TorqueValue]
    bufferOffset = _serializer.float64(obj.TorqueValue, buffer, bufferOffset);
    // Serialize message field [ReturnTorqueValue]
    bufferOffset = _serializer.float64(obj.ReturnTorqueValue, buffer, bufferOffset);
    // Serialize message field [VelocityValue]
    bufferOffset = _serializer.float64(obj.VelocityValue, buffer, bufferOffset);
    // Serialize message field [ReturnVelocity]
    bufferOffset = _serializer.float64(obj.ReturnVelocity, buffer, bufferOffset);
    // Serialize message field [PDvelocity]
    bufferOffset = _serializer.float64(obj.PDvelocity, buffer, bufferOffset);
    // Serialize message field [ForceSensortorque]
    bufferOffset = _serializer.float32(obj.ForceSensortorque, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Torque
    let len;
    let data = new Torque(null);
    // Deserialize message field [TorqueValue]
    data.TorqueValue = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ReturnTorqueValue]
    data.ReturnTorqueValue = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [VelocityValue]
    data.VelocityValue = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ReturnVelocity]
    data.ReturnVelocity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [PDvelocity]
    data.PDvelocity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ForceSensortorque]
    data.ForceSensortorque = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 44;
  }

  static datatype() {
    // Returns string type for a message object
    return 'can_ankle/Torque';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f1c893b945ede6496888af3eab4a52a8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 TorqueValue
    float64 ReturnTorqueValue
    float64 VelocityValue
    float64 ReturnVelocity
    float64 PDvelocity
    float32 ForceSensortorque
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Torque(null);
    if (msg.TorqueValue !== undefined) {
      resolved.TorqueValue = msg.TorqueValue;
    }
    else {
      resolved.TorqueValue = 0.0
    }

    if (msg.ReturnTorqueValue !== undefined) {
      resolved.ReturnTorqueValue = msg.ReturnTorqueValue;
    }
    else {
      resolved.ReturnTorqueValue = 0.0
    }

    if (msg.VelocityValue !== undefined) {
      resolved.VelocityValue = msg.VelocityValue;
    }
    else {
      resolved.VelocityValue = 0.0
    }

    if (msg.ReturnVelocity !== undefined) {
      resolved.ReturnVelocity = msg.ReturnVelocity;
    }
    else {
      resolved.ReturnVelocity = 0.0
    }

    if (msg.PDvelocity !== undefined) {
      resolved.PDvelocity = msg.PDvelocity;
    }
    else {
      resolved.PDvelocity = 0.0
    }

    if (msg.ForceSensortorque !== undefined) {
      resolved.ForceSensortorque = msg.ForceSensortorque;
    }
    else {
      resolved.ForceSensortorque = 0.0
    }

    return resolved;
    }
};

module.exports = Torque;
