// Generated by gencpp from file gps_common/GPSFix.msg
// DO NOT EDIT!


#ifndef GPS_COMMON_MESSAGE_GPSFIX_H
#define GPS_COMMON_MESSAGE_GPSFIX_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <gps_common/GPSStatus.h>

namespace gps_common
{
template <class ContainerAllocator>
struct GPSFix_
{
  typedef GPSFix_<ContainerAllocator> Type;

  GPSFix_()
    : header()
    , status()
    , latitude(0.0)
    , longitude(0.0)
    , altitude(0.0)
    , track(0.0)
    , speed(0.0)
    , climb(0.0)
    , pitch(0.0)
    , roll(0.0)
    , dip(0.0)
    , time(0.0)
    , gdop(0.0)
    , pdop(0.0)
    , hdop(0.0)
    , vdop(0.0)
    , tdop(0.0)
    , err(0.0)
    , err_horz(0.0)
    , err_vert(0.0)
    , err_track(0.0)
    , err_speed(0.0)
    , err_climb(0.0)
    , err_time(0.0)
    , err_pitch(0.0)
    , err_roll(0.0)
    , err_dip(0.0)
    , position_covariance()
    , position_covariance_type(0)  {
      position_covariance.assign(0.0);
  }
  GPSFix_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , status(_alloc)
    , latitude(0.0)
    , longitude(0.0)
    , altitude(0.0)
    , track(0.0)
    , speed(0.0)
    , climb(0.0)
    , pitch(0.0)
    , roll(0.0)
    , dip(0.0)
    , time(0.0)
    , gdop(0.0)
    , pdop(0.0)
    , hdop(0.0)
    , vdop(0.0)
    , tdop(0.0)
    , err(0.0)
    , err_horz(0.0)
    , err_vert(0.0)
    , err_track(0.0)
    , err_speed(0.0)
    , err_climb(0.0)
    , err_time(0.0)
    , err_pitch(0.0)
    , err_roll(0.0)
    , err_dip(0.0)
    , position_covariance()
    , position_covariance_type(0)  {
  (void)_alloc;
      position_covariance.assign(0.0);
  }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::gps_common::GPSStatus_<ContainerAllocator>  _status_type;
  _status_type status;

   typedef double _latitude_type;
  _latitude_type latitude;

   typedef double _longitude_type;
  _longitude_type longitude;

   typedef double _altitude_type;
  _altitude_type altitude;

   typedef double _track_type;
  _track_type track;

   typedef double _speed_type;
  _speed_type speed;

   typedef double _climb_type;
  _climb_type climb;

   typedef double _pitch_type;
  _pitch_type pitch;

   typedef double _roll_type;
  _roll_type roll;

   typedef double _dip_type;
  _dip_type dip;

   typedef double _time_type;
  _time_type time;

   typedef double _gdop_type;
  _gdop_type gdop;

   typedef double _pdop_type;
  _pdop_type pdop;

   typedef double _hdop_type;
  _hdop_type hdop;

   typedef double _vdop_type;
  _vdop_type vdop;

   typedef double _tdop_type;
  _tdop_type tdop;

   typedef double _err_type;
  _err_type err;

   typedef double _err_horz_type;
  _err_horz_type err_horz;

   typedef double _err_vert_type;
  _err_vert_type err_vert;

   typedef double _err_track_type;
  _err_track_type err_track;

   typedef double _err_speed_type;
  _err_speed_type err_speed;

   typedef double _err_climb_type;
  _err_climb_type err_climb;

   typedef double _err_time_type;
  _err_time_type err_time;

   typedef double _err_pitch_type;
  _err_pitch_type err_pitch;

   typedef double _err_roll_type;
  _err_roll_type err_roll;

   typedef double _err_dip_type;
  _err_dip_type err_dip;

   typedef boost::array<double, 9>  _position_covariance_type;
  _position_covariance_type position_covariance;

   typedef uint8_t _position_covariance_type_type;
  _position_covariance_type_type position_covariance_type;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(COVARIANCE_TYPE_UNKNOWN)
  #undef COVARIANCE_TYPE_UNKNOWN
#endif
#if defined(_WIN32) && defined(COVARIANCE_TYPE_APPROXIMATED)
  #undef COVARIANCE_TYPE_APPROXIMATED
#endif
#if defined(_WIN32) && defined(COVARIANCE_TYPE_DIAGONAL_KNOWN)
  #undef COVARIANCE_TYPE_DIAGONAL_KNOWN
#endif
#if defined(_WIN32) && defined(COVARIANCE_TYPE_KNOWN)
  #undef COVARIANCE_TYPE_KNOWN
#endif

  enum {
    COVARIANCE_TYPE_UNKNOWN = 0u,
    COVARIANCE_TYPE_APPROXIMATED = 1u,
    COVARIANCE_TYPE_DIAGONAL_KNOWN = 2u,
    COVARIANCE_TYPE_KNOWN = 3u,
  };


  typedef boost::shared_ptr< ::gps_common::GPSFix_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::gps_common::GPSFix_<ContainerAllocator> const> ConstPtr;

}; // struct GPSFix_

typedef ::gps_common::GPSFix_<std::allocator<void> > GPSFix;

typedef boost::shared_ptr< ::gps_common::GPSFix > GPSFixPtr;
typedef boost::shared_ptr< ::gps_common::GPSFix const> GPSFixConstPtr;

// constants requiring out of line definition

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::gps_common::GPSFix_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::gps_common::GPSFix_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::gps_common::GPSFix_<ContainerAllocator1> & lhs, const ::gps_common::GPSFix_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.status == rhs.status &&
    lhs.latitude == rhs.latitude &&
    lhs.longitude == rhs.longitude &&
    lhs.altitude == rhs.altitude &&
    lhs.track == rhs.track &&
    lhs.speed == rhs.speed &&
    lhs.climb == rhs.climb &&
    lhs.pitch == rhs.pitch &&
    lhs.roll == rhs.roll &&
    lhs.dip == rhs.dip &&
    lhs.time == rhs.time &&
    lhs.gdop == rhs.gdop &&
    lhs.pdop == rhs.pdop &&
    lhs.hdop == rhs.hdop &&
    lhs.vdop == rhs.vdop &&
    lhs.tdop == rhs.tdop &&
    lhs.err == rhs.err &&
    lhs.err_horz == rhs.err_horz &&
    lhs.err_vert == rhs.err_vert &&
    lhs.err_track == rhs.err_track &&
    lhs.err_speed == rhs.err_speed &&
    lhs.err_climb == rhs.err_climb &&
    lhs.err_time == rhs.err_time &&
    lhs.err_pitch == rhs.err_pitch &&
    lhs.err_roll == rhs.err_roll &&
    lhs.err_dip == rhs.err_dip &&
    lhs.position_covariance == rhs.position_covariance &&
    lhs.position_covariance_type == rhs.position_covariance_type;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::gps_common::GPSFix_<ContainerAllocator1> & lhs, const ::gps_common::GPSFix_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace gps_common

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::gps_common::GPSFix_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gps_common::GPSFix_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gps_common::GPSFix_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gps_common::GPSFix_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gps_common::GPSFix_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gps_common::GPSFix_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::gps_common::GPSFix_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3db3d0a7bc53054c67c528af84710b70";
  }

  static const char* value(const ::gps_common::GPSFix_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3db3d0a7bc53054cULL;
  static const uint64_t static_value2 = 0x67c528af84710b70ULL;
};

template<class ContainerAllocator>
struct DataType< ::gps_common::GPSFix_<ContainerAllocator> >
{
  static const char* value()
  {
    return "gps_common/GPSFix";
  }

  static const char* value(const ::gps_common::GPSFix_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::gps_common::GPSFix_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# A more complete GPS fix to supplement sensor_msgs/NavSatFix.\n"
"Header header\n"
"\n"
"GPSStatus status\n"
"\n"
"# Latitude (degrees). Positive is north of equator; negative is south.\n"
"float64 latitude\n"
"\n"
"# Longitude (degrees). Positive is east of prime meridian, negative west.\n"
"float64 longitude\n"
"\n"
"# Altitude (meters). Positive is above reference (e.g., sea level).\n"
"float64 altitude\n"
"\n"
"# Direction (degrees from north)\n"
"float64 track\n"
"\n"
"# Ground speed (meters/second)\n"
"float64 speed\n"
"\n"
"# Vertical speed (meters/second)\n"
"float64 climb\n"
"\n"
"# Device orientation (units in degrees)\n"
"float64 pitch\n"
"float64 roll\n"
"float64 dip\n"
"\n"
"# GPS time\n"
"float64 time\n"
"\n"
"## Dilution of precision; Xdop<=0 means the value is unknown\n"
"\n"
"# Total (positional-temporal) dilution of precision\n"
"float64 gdop\n"
"\n"
"# Positional (3D) dilution of precision\n"
"float64 pdop\n"
"\n"
"# Horizontal dilution of precision\n"
"float64 hdop\n"
"\n"
"# Vertical dilution of precision\n"
"float64 vdop\n"
"\n"
"# Temporal dilution of precision\n"
"float64 tdop\n"
"\n"
"## Uncertainty of measurement, 95% confidence\n"
"\n"
"# Spherical position uncertainty (meters) [epe]\n"
"float64 err\n"
"\n"
"# Horizontal position uncertainty (meters) [eph]\n"
"float64 err_horz\n"
"\n"
"# Vertical position uncertainty (meters) [epv]\n"
"float64 err_vert\n"
"\n"
"# Track uncertainty (degrees) [epd]\n"
"float64 err_track\n"
"\n"
"# Ground speed uncertainty (meters/second) [eps]\n"
"float64 err_speed\n"
"\n"
"# Vertical speed uncertainty (meters/second) [epc]\n"
"float64 err_climb\n"
"\n"
"# Temporal uncertainty [ept]\n"
"float64 err_time\n"
"\n"
"# Orientation uncertainty (degrees)\n"
"float64 err_pitch\n"
"float64 err_roll\n"
"float64 err_dip\n"
"\n"
"# Position covariance [m^2] defined relative to a tangential plane\n"
"# through the reported position. The components are East, North, and\n"
"# Up (ENU), in row-major order.\n"
"\n"
"float64[9] position_covariance\n"
"\n"
"uint8 COVARIANCE_TYPE_UNKNOWN = 0\n"
"uint8 COVARIANCE_TYPE_APPROXIMATED = 1\n"
"uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2\n"
"uint8 COVARIANCE_TYPE_KNOWN = 3\n"
"\n"
"uint8 position_covariance_type\n"
"\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: gps_common/GPSStatus\n"
"Header header\n"
"\n"
"# Satellites used in solution\n"
"uint16 satellites_used # Number of satellites\n"
"int32[] satellite_used_prn # PRN identifiers\n"
"\n"
"# Satellites visible\n"
"uint16 satellites_visible\n"
"int32[] satellite_visible_prn # PRN identifiers\n"
"int32[] satellite_visible_z # Elevation of satellites\n"
"int32[] satellite_visible_azimuth # Azimuth of satellites\n"
"int32[] satellite_visible_snr # Signal-to-noise ratios (dB)\n"
"\n"
"# Measurement status\n"
"int16 STATUS_NO_FIX=-1   # Unable to fix position\n"
"int16 STATUS_FIX=0       # Normal fix\n"
"int16 STATUS_SBAS_FIX=1  # Fixed using a satellite-based augmentation system\n"
"int16 STATUS_GBAS_FIX=2  #          or a ground-based augmentation system\n"
"int16 STATUS_DGPS_FIX=18 # Fixed with DGPS\n"
"int16 STATUS_WAAS_FIX=33 # Fixed with WAAS\n"
"int16 status\n"
"\n"
"uint16 SOURCE_NONE=0 # No information is available\n"
"uint16 SOURCE_GPS=1 # Using standard GPS location [only valid for position_source]\n"
"uint16 SOURCE_POINTS=2 # Motion/orientation fix is derived from successive points\n"
"uint16 SOURCE_DOPPLER=4 # Motion is derived using the Doppler effect\n"
"uint16 SOURCE_ALTIMETER=8 # Using an altimeter\n"
"uint16 SOURCE_MAGNETIC=16 # Using magnetic sensors\n"
"uint16 SOURCE_GYRO=32 # Using gyroscopes\n"
"uint16 SOURCE_ACCEL=64 # Using accelerometers\n"
"\n"
"uint16 motion_source # Source for speed, climb and track\n"
"uint16 orientation_source # Source for device orientation\n"
"uint16 position_source # Source for position\n"
"\n"
;
  }

  static const char* value(const ::gps_common::GPSFix_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::gps_common::GPSFix_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.status);
      stream.next(m.latitude);
      stream.next(m.longitude);
      stream.next(m.altitude);
      stream.next(m.track);
      stream.next(m.speed);
      stream.next(m.climb);
      stream.next(m.pitch);
      stream.next(m.roll);
      stream.next(m.dip);
      stream.next(m.time);
      stream.next(m.gdop);
      stream.next(m.pdop);
      stream.next(m.hdop);
      stream.next(m.vdop);
      stream.next(m.tdop);
      stream.next(m.err);
      stream.next(m.err_horz);
      stream.next(m.err_vert);
      stream.next(m.err_track);
      stream.next(m.err_speed);
      stream.next(m.err_climb);
      stream.next(m.err_time);
      stream.next(m.err_pitch);
      stream.next(m.err_roll);
      stream.next(m.err_dip);
      stream.next(m.position_covariance);
      stream.next(m.position_covariance_type);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GPSFix_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::gps_common::GPSFix_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::gps_common::GPSFix_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "status: ";
    s << std::endl;
    Printer< ::gps_common::GPSStatus_<ContainerAllocator> >::stream(s, indent + "  ", v.status);
    s << indent << "latitude: ";
    Printer<double>::stream(s, indent + "  ", v.latitude);
    s << indent << "longitude: ";
    Printer<double>::stream(s, indent + "  ", v.longitude);
    s << indent << "altitude: ";
    Printer<double>::stream(s, indent + "  ", v.altitude);
    s << indent << "track: ";
    Printer<double>::stream(s, indent + "  ", v.track);
    s << indent << "speed: ";
    Printer<double>::stream(s, indent + "  ", v.speed);
    s << indent << "climb: ";
    Printer<double>::stream(s, indent + "  ", v.climb);
    s << indent << "pitch: ";
    Printer<double>::stream(s, indent + "  ", v.pitch);
    s << indent << "roll: ";
    Printer<double>::stream(s, indent + "  ", v.roll);
    s << indent << "dip: ";
    Printer<double>::stream(s, indent + "  ", v.dip);
    s << indent << "time: ";
    Printer<double>::stream(s, indent + "  ", v.time);
    s << indent << "gdop: ";
    Printer<double>::stream(s, indent + "  ", v.gdop);
    s << indent << "pdop: ";
    Printer<double>::stream(s, indent + "  ", v.pdop);
    s << indent << "hdop: ";
    Printer<double>::stream(s, indent + "  ", v.hdop);
    s << indent << "vdop: ";
    Printer<double>::stream(s, indent + "  ", v.vdop);
    s << indent << "tdop: ";
    Printer<double>::stream(s, indent + "  ", v.tdop);
    s << indent << "err: ";
    Printer<double>::stream(s, indent + "  ", v.err);
    s << indent << "err_horz: ";
    Printer<double>::stream(s, indent + "  ", v.err_horz);
    s << indent << "err_vert: ";
    Printer<double>::stream(s, indent + "  ", v.err_vert);
    s << indent << "err_track: ";
    Printer<double>::stream(s, indent + "  ", v.err_track);
    s << indent << "err_speed: ";
    Printer<double>::stream(s, indent + "  ", v.err_speed);
    s << indent << "err_climb: ";
    Printer<double>::stream(s, indent + "  ", v.err_climb);
    s << indent << "err_time: ";
    Printer<double>::stream(s, indent + "  ", v.err_time);
    s << indent << "err_pitch: ";
    Printer<double>::stream(s, indent + "  ", v.err_pitch);
    s << indent << "err_roll: ";
    Printer<double>::stream(s, indent + "  ", v.err_roll);
    s << indent << "err_dip: ";
    Printer<double>::stream(s, indent + "  ", v.err_dip);
    s << indent << "position_covariance[]" << std::endl;
    for (size_t i = 0; i < v.position_covariance.size(); ++i)
    {
      s << indent << "  position_covariance[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.position_covariance[i]);
    }
    s << indent << "position_covariance_type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.position_covariance_type);
  }
};

} // namespace message_operations
} // namespace ros

#endif // GPS_COMMON_MESSAGE_GPSFIX_H
