# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from ublox_msgs/CfgGNSS_Block.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class CfgGNSS_Block(genpy.Message):
  _md5sum = "f786023414ba20add907814936842e32"
  _type = "ublox_msgs/CfgGNSS_Block"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# see Cfg-GNSS message
#

uint8 gnssId            # System identifier (see Satellite Numbering)

uint8 GNSS_ID_GPS = 0
uint8 GNSS_ID_SBAS = 1
uint8 GNSS_ID_GALILEO = 2
uint8 GNSS_ID_BEIDOU = 3
uint8 GNSS_ID_IMES = 4
uint8 GNSS_ID_QZSS = 5
uint8 GNSS_ID_GLONASS = 6

uint8 resTrkCh          # (Read only in protocol versions greater than 23)
                        # Number of reserved (minimum) tracking channels 
                        # for this GNSS system
uint8 RES_TRK_CH_GPS = 8
uint8 RES_TRK_CH_QZSS = 0
uint8 RES_TRK_CH_SBAS = 0
uint8 RES_TRK_CH_GLONASS = 8
uint8 maxTrkCh          # (Read only in protocol versions greater than 23)
                        # Maximum number of tracking channels used for this 
                        # system. Must be > 0, >= resTrkChn, <= numTrkChUse and
                        # <= maximum number of tracking channels supported for 
                        # this system
uint8 MAX_TRK_CH_MAJOR_MIN = 4         # maxTrkCh must have this minimum value
                                       # for each enabled major GNSS
uint8 MAX_TRK_CH_GPS = 16
uint8 MAX_TRK_CH_GLONASS = 14
uint8 MAX_TRK_CH_QZSS = 3
uint8 MAX_TRK_CH_SBAS = 3

uint8 reserved1         # Reserved

uint32 flags            # Bitfield of flags. At least one signal must be
                        # configured in every enabled system. 
uint32 FLAGS_ENABLE = 1                # Enable this system
uint32 FLAGS_SIG_CFG_MASK = 16711680   # Signal configuration mask
uint32 SIG_CFG_GPS_L1CA = 65536        # When gnssId is 0 (GPS)
                                       # * 0x01 = GPS L1C/A
uint32 SIG_CFG_SBAS_L1CA = 65536       # When gnssId is 1 (SBAS)
                                       # * 0x01 = SBAS L1C/A
uint32 SIG_CFG_GALILEO_E1OS = 65536    # When gnssId is 2 (Galileo)
                                       # * 0x01 = Galileo E1OS (not supported in 
                                       #   protocol versions less than 18)
uint32 SIG_CFG_BEIDOU_B1I = 65536      # When gnssId is 3 (BeiDou)
                                       # * 0x01 = BeiDou B1I 
uint32 SIG_CFG_IMES_L1 = 65536         # When gnssId is 4 (IMES)
                                       # * 0x01 = IMES L1
uint32 SIG_CFG_QZSS_L1CA = 65536       # When gnssId is 5 (QZSS)
                                       # * 0x01 = QZSS L1C/A
uint32 SIG_CFG_QZSS_L1SAIF = 262144    # * 0x04 = QZSS L1SAIF
uint32 SIG_CFG_GLONASS_L1OF = 65536    # When gnssId is 6 (GLONASS)
                                       # * 0x01 = GLONASS L1OF"""
  # Pseudo-constants
  GNSS_ID_GPS = 0
  GNSS_ID_SBAS = 1
  GNSS_ID_GALILEO = 2
  GNSS_ID_BEIDOU = 3
  GNSS_ID_IMES = 4
  GNSS_ID_QZSS = 5
  GNSS_ID_GLONASS = 6
  RES_TRK_CH_GPS = 8
  RES_TRK_CH_QZSS = 0
  RES_TRK_CH_SBAS = 0
  RES_TRK_CH_GLONASS = 8
  MAX_TRK_CH_MAJOR_MIN = 4
  MAX_TRK_CH_GPS = 16
  MAX_TRK_CH_GLONASS = 14
  MAX_TRK_CH_QZSS = 3
  MAX_TRK_CH_SBAS = 3
  FLAGS_ENABLE = 1
  FLAGS_SIG_CFG_MASK = 16711680
  SIG_CFG_GPS_L1CA = 65536
  SIG_CFG_SBAS_L1CA = 65536
  SIG_CFG_GALILEO_E1OS = 65536
  SIG_CFG_BEIDOU_B1I = 65536
  SIG_CFG_IMES_L1 = 65536
  SIG_CFG_QZSS_L1CA = 65536
  SIG_CFG_QZSS_L1SAIF = 262144
  SIG_CFG_GLONASS_L1OF = 65536

  __slots__ = ['gnssId','resTrkCh','maxTrkCh','reserved1','flags']
  _slot_types = ['uint8','uint8','uint8','uint8','uint32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       gnssId,resTrkCh,maxTrkCh,reserved1,flags

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(CfgGNSS_Block, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.gnssId is None:
        self.gnssId = 0
      if self.resTrkCh is None:
        self.resTrkCh = 0
      if self.maxTrkCh is None:
        self.maxTrkCh = 0
      if self.reserved1 is None:
        self.reserved1 = 0
      if self.flags is None:
        self.flags = 0
    else:
      self.gnssId = 0
      self.resTrkCh = 0
      self.maxTrkCh = 0
      self.reserved1 = 0
      self.flags = 0

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
      buff.write(_get_struct_4BI().pack(_x.gnssId, _x.resTrkCh, _x.maxTrkCh, _x.reserved1, _x.flags))
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
      _x = self
      start = end
      end += 8
      (_x.gnssId, _x.resTrkCh, _x.maxTrkCh, _x.reserved1, _x.flags,) = _get_struct_4BI().unpack(str[start:end])
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
      buff.write(_get_struct_4BI().pack(_x.gnssId, _x.resTrkCh, _x.maxTrkCh, _x.reserved1, _x.flags))
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
      _x = self
      start = end
      end += 8
      (_x.gnssId, _x.resTrkCh, _x.maxTrkCh, _x.reserved1, _x.flags,) = _get_struct_4BI().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_4BI = None
def _get_struct_4BI():
    global _struct_4BI
    if _struct_4BI is None:
        _struct_4BI = struct.Struct("<4BI")
    return _struct_4BI
