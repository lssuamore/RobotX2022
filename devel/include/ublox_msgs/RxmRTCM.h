// Generated by gencpp from file ublox_msgs/RxmRTCM.msg
// DO NOT EDIT!


#ifndef UBLOX_MSGS_MESSAGE_RXMRTCM_H
#define UBLOX_MSGS_MESSAGE_RXMRTCM_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ublox_msgs
{
template <class ContainerAllocator>
struct RxmRTCM_
{
  typedef RxmRTCM_<ContainerAllocator> Type;

  RxmRTCM_()
    : version(0)
    , flags(0)
    , reserved0()
    , refStation(0)
    , msgType(0)  {
      reserved0.assign(0);
  }
  RxmRTCM_(const ContainerAllocator& _alloc)
    : version(0)
    , flags(0)
    , reserved0()
    , refStation(0)
    , msgType(0)  {
  (void)_alloc;
      reserved0.assign(0);
  }



   typedef uint8_t _version_type;
  _version_type version;

   typedef uint8_t _flags_type;
  _flags_type flags;

   typedef boost::array<uint8_t, 2>  _reserved0_type;
  _reserved0_type reserved0;

   typedef uint16_t _refStation_type;
  _refStation_type refStation;

   typedef uint16_t _msgType_type;
  _msgType_type msgType;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(CLASS_ID)
  #undef CLASS_ID
#endif
#if defined(_WIN32) && defined(MESSAGE_ID)
  #undef MESSAGE_ID
#endif
#if defined(_WIN32) && defined(FLAGS_CRC_FAILED)
  #undef FLAGS_CRC_FAILED
#endif

  enum {
    CLASS_ID = 2u,
    MESSAGE_ID = 50u,
    FLAGS_CRC_FAILED = 1u,
  };


  typedef boost::shared_ptr< ::ublox_msgs::RxmRTCM_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ublox_msgs::RxmRTCM_<ContainerAllocator> const> ConstPtr;

}; // struct RxmRTCM_

typedef ::ublox_msgs::RxmRTCM_<std::allocator<void> > RxmRTCM;

typedef boost::shared_ptr< ::ublox_msgs::RxmRTCM > RxmRTCMPtr;
typedef boost::shared_ptr< ::ublox_msgs::RxmRTCM const> RxmRTCMConstPtr;

// constants requiring out of line definition

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ublox_msgs::RxmRTCM_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ublox_msgs::RxmRTCM_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ublox_msgs::RxmRTCM_<ContainerAllocator1> & lhs, const ::ublox_msgs::RxmRTCM_<ContainerAllocator2> & rhs)
{
  return lhs.version == rhs.version &&
    lhs.flags == rhs.flags &&
    lhs.reserved0 == rhs.reserved0 &&
    lhs.refStation == rhs.refStation &&
    lhs.msgType == rhs.msgType;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ublox_msgs::RxmRTCM_<ContainerAllocator1> & lhs, const ::ublox_msgs::RxmRTCM_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ublox_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::RxmRTCM_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::RxmRTCM_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::RxmRTCM_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::RxmRTCM_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::RxmRTCM_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::RxmRTCM_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ublox_msgs::RxmRTCM_<ContainerAllocator> >
{
  static const char* value()
  {
    return "86ca768ef7c0009454812a5f8c6badfc";
  }

  static const char* value(const ::ublox_msgs::RxmRTCM_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x86ca768ef7c00094ULL;
  static const uint64_t static_value2 = 0x54812a5f8c6badfcULL;
};

template<class ContainerAllocator>
struct DataType< ::ublox_msgs::RxmRTCM_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ublox_msgs/RxmRTCM";
  }

  static const char* value(const ::ublox_msgs::RxmRTCM_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ublox_msgs::RxmRTCM_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# RXM-RTCM (0x02 0x32)\n"
"# RTCM input status\n"
"#\n"
"# Output upon processing of an RTCM input message\n"
"# Supported on:\n"
"# - u-blox 8 / u-blox M8 from protocol version 20.01 up to version 23.01\n"
"# \n"
"\n"
"uint8 CLASS_ID = 2\n"
"uint8 MESSAGE_ID = 50\n"
"\n"
"uint8 version                 # Message version (0x02 for this version)\n"
"uint8 flags                   # RTCM input status flags\n"
"uint8 FLAGS_CRC_FAILED = 1    # 0 when RTCM message received and passed CRC \n"
"                              # check, 1 when failed in which case refStation\n"
"                              # and msgType might be corrupted and misleading\n"
"\n"
"uint8[2] reserved0            # Reserved\n"
"\n"
"uint16 refStation             # Reference station ID\n"
"uint16 msgType                # Message type\n"
;
  }

  static const char* value(const ::ublox_msgs::RxmRTCM_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ublox_msgs::RxmRTCM_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.version);
      stream.next(m.flags);
      stream.next(m.reserved0);
      stream.next(m.refStation);
      stream.next(m.msgType);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RxmRTCM_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ublox_msgs::RxmRTCM_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ublox_msgs::RxmRTCM_<ContainerAllocator>& v)
  {
    s << indent << "version: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.version);
    s << indent << "flags: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.flags);
    s << indent << "reserved0[]" << std::endl;
    for (size_t i = 0; i < v.reserved0.size(); ++i)
    {
      s << indent << "  reserved0[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.reserved0[i]);
    }
    s << indent << "refStation: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.refStation);
    s << indent << "msgType: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.msgType);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UBLOX_MSGS_MESSAGE_RXMRTCM_H
