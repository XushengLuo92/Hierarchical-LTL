# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from moveit_fanuc_config/robot_actionRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import std_msgs.msg

class robot_actionRequest(genpy.Message):
  _md5sum = "4c3cb3bfc4ebe0aeb938436ad71e3af2"
  _type = "moveit_fanuc_config/robot_actionRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """geometry_msgs/PoseStamped des
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
"""
  __slots__ = ['des','src']
  _slot_types = ['geometry_msgs/PoseStamped','geometry_msgs/PoseStamped']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       des,src

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(robot_actionRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.des is None:
        self.des = geometry_msgs.msg.PoseStamped()
      if self.src is None:
        self.src = geometry_msgs.msg.PoseStamped()
    else:
      self.des = geometry_msgs.msg.PoseStamped()
      self.src = geometry_msgs.msg.PoseStamped()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.des.header.seq, _x.des.header.stamp.secs, _x.des.header.stamp.nsecs))
      _x = self.des.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_7d3I().pack(_x.des.pose.position.x, _x.des.pose.position.y, _x.des.pose.position.z, _x.des.pose.orientation.x, _x.des.pose.orientation.y, _x.des.pose.orientation.z, _x.des.pose.orientation.w, _x.src.header.seq, _x.src.header.stamp.secs, _x.src.header.stamp.nsecs))
      _x = self.src.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_7d().pack(_x.src.pose.position.x, _x.src.pose.position.y, _x.src.pose.position.z, _x.src.pose.orientation.x, _x.src.pose.orientation.y, _x.src.pose.orientation.z, _x.src.pose.orientation.w))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.des is None:
        self.des = geometry_msgs.msg.PoseStamped()
      if self.src is None:
        self.src = geometry_msgs.msg.PoseStamped()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.des.header.seq, _x.des.header.stamp.secs, _x.des.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.des.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.des.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 68
      (_x.des.pose.position.x, _x.des.pose.position.y, _x.des.pose.position.z, _x.des.pose.orientation.x, _x.des.pose.orientation.y, _x.des.pose.orientation.z, _x.des.pose.orientation.w, _x.src.header.seq, _x.src.header.stamp.secs, _x.src.header.stamp.nsecs,) = _get_struct_7d3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.src.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.src.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.src.pose.position.x, _x.src.pose.position.y, _x.src.pose.position.z, _x.src.pose.orientation.x, _x.src.pose.orientation.y, _x.src.pose.orientation.z, _x.src.pose.orientation.w,) = _get_struct_7d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.des.header.seq, _x.des.header.stamp.secs, _x.des.header.stamp.nsecs))
      _x = self.des.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_7d3I().pack(_x.des.pose.position.x, _x.des.pose.position.y, _x.des.pose.position.z, _x.des.pose.orientation.x, _x.des.pose.orientation.y, _x.des.pose.orientation.z, _x.des.pose.orientation.w, _x.src.header.seq, _x.src.header.stamp.secs, _x.src.header.stamp.nsecs))
      _x = self.src.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_7d().pack(_x.src.pose.position.x, _x.src.pose.position.y, _x.src.pose.position.z, _x.src.pose.orientation.x, _x.src.pose.orientation.y, _x.src.pose.orientation.z, _x.src.pose.orientation.w))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.des is None:
        self.des = geometry_msgs.msg.PoseStamped()
      if self.src is None:
        self.src = geometry_msgs.msg.PoseStamped()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.des.header.seq, _x.des.header.stamp.secs, _x.des.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.des.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.des.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 68
      (_x.des.pose.position.x, _x.des.pose.position.y, _x.des.pose.position.z, _x.des.pose.orientation.x, _x.des.pose.orientation.y, _x.des.pose.orientation.z, _x.des.pose.orientation.w, _x.src.header.seq, _x.src.header.stamp.secs, _x.src.header.stamp.nsecs,) = _get_struct_7d3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.src.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.src.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.src.pose.position.x, _x.src.pose.position.y, _x.src.pose.position.z, _x.src.pose.orientation.x, _x.src.pose.orientation.y, _x.src.pose.orientation.z, _x.src.pose.orientation.w,) = _get_struct_7d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_7d = None
def _get_struct_7d():
    global _struct_7d
    if _struct_7d is None:
        _struct_7d = struct.Struct("<7d")
    return _struct_7d
_struct_7d3I = None
def _get_struct_7d3I():
    global _struct_7d3I
    if _struct_7d3I is None:
        _struct_7d3I = struct.Struct("<7d3I")
    return _struct_7d3I
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from moveit_fanuc_config/robot_actionResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class robot_actionResponse(genpy.Message):
  _md5sum = "e2797c797e620a950ba704492d72873b"
  _type = "moveit_fanuc_config/robot_actionResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """bool finished
"""
  __slots__ = ['finished']
  _slot_types = ['bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       finished

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(robot_actionResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.finished is None:
        self.finished = False
    else:
      self.finished = False

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self.finished
      buff.write(_get_struct_B().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 1
      (self.finished,) = _get_struct_B().unpack(str[start:end])
      self.finished = bool(self.finished)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.finished
      buff.write(_get_struct_B().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 1
      (self.finished,) = _get_struct_B().unpack(str[start:end])
      self.finished = bool(self.finished)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
class robot_action(object):
  _type          = 'moveit_fanuc_config/robot_action'
  _md5sum = '19a560c38819713863fdccb541e95a18'
  _request_class  = robot_actionRequest
  _response_class = robot_actionResponse
