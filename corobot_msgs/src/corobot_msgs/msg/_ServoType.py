"""autogenerated by genpy from corobot_msgs/ServoType.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class ServoType(genpy.Message):
  _md5sum = "1838b6e393c9a582039957a5d550ab79"
  _type = "corobot_msgs/ServoType"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """#
# Morgan Cormier <mcormier@coroware.com>
#
# Service used to set the type of servo motor connected at the specified index on the phidget servo controller
#
# Index is the index of the servo motor connected to the phidget device. 
# The maximum value of index depends on the Phidget device and how many connections it accepts
#
# type is the type of motor. See enum CPhidget_ServoType of the Phidget library for the list of type.

int8 index

int8 type

"""
  __slots__ = ['index','type']
  _slot_types = ['int8','int8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       index,type

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ServoType, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.index is None:
        self.index = 0
      if self.type is None:
        self.type = 0
    else:
      self.index = 0
      self.type = 0

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
      buff.write(_struct_2b.pack(_x.index, _x.type))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 2
      (_x.index, _x.type,) = _struct_2b.unpack(str[start:end])
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
      buff.write(_struct_2b.pack(_x.index, _x.type))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

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
      end += 2
      (_x.index, _x.type,) = _struct_2b.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2b = struct.Struct("<2b")
