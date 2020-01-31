# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from pangyo_control/ControlCommand.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class ControlCommand(genpy.Message):
  _md5sum = "37feea1cd0d627db0c18584be77b9973"
  _type = "pangyo_control/ControlCommand"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """uint16 gear
uint16 speed
float32 steer
uint16 brake
"""
  __slots__ = ['gear','speed','steer','brake']
  _slot_types = ['uint16','uint16','float32','uint16']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       gear,speed,steer,brake

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ControlCommand, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.gear is None:
        self.gear = 0
      if self.speed is None:
        self.speed = 0
      if self.steer is None:
        self.steer = 0.
      if self.brake is None:
        self.brake = 0
    else:
      self.gear = 0
      self.speed = 0
      self.steer = 0.
      self.brake = 0

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
      buff.write(_get_struct_2HfH().pack(_x.gear, _x.speed, _x.steer, _x.brake))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 10
      (_x.gear, _x.speed, _x.steer, _x.brake,) = _get_struct_2HfH().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_2HfH().pack(_x.gear, _x.speed, _x.steer, _x.brake))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 10
      (_x.gear, _x.speed, _x.steer, _x.brake,) = _get_struct_2HfH().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2HfH = None
def _get_struct_2HfH():
    global _struct_2HfH
    if _struct_2HfH is None:
        _struct_2HfH = struct.Struct("<2HfH")
    return _struct_2HfH
