// Generated by gencpp from file ublox_msgs/CfgSBAS.msg
// DO NOT EDIT!


#ifndef UBLOX_MSGS_MESSAGE_CFGSBAS_H
#define UBLOX_MSGS_MESSAGE_CFGSBAS_H


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
struct CfgSBAS_
{
  typedef CfgSBAS_<ContainerAllocator> Type;

  CfgSBAS_()
    : mode(0)
    , usage(0)
    , maxSBAS(0)
    , scanmode2(0)
    , scanmode1(0)  {
    }
  CfgSBAS_(const ContainerAllocator& _alloc)
    : mode(0)
    , usage(0)
    , maxSBAS(0)
    , scanmode2(0)
    , scanmode1(0)  {
  (void)_alloc;
    }



   typedef uint8_t _mode_type;
  _mode_type mode;

   typedef uint8_t _usage_type;
  _usage_type usage;

   typedef uint8_t _maxSBAS_type;
  _maxSBAS_type maxSBAS;

   typedef uint8_t _scanmode2_type;
  _scanmode2_type scanmode2;

   typedef uint32_t _scanmode1_type;
  _scanmode1_type scanmode1;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(CLASS_ID)
  #undef CLASS_ID
#endif
#if defined(_WIN32) && defined(MESSAGE_ID)
  #undef MESSAGE_ID
#endif
#if defined(_WIN32) && defined(MODE_ENABLED)
  #undef MODE_ENABLED
#endif
#if defined(_WIN32) && defined(MODE_TEST)
  #undef MODE_TEST
#endif
#if defined(_WIN32) && defined(USAGE_RANGE)
  #undef USAGE_RANGE
#endif
#if defined(_WIN32) && defined(USAGE_DIFF_CORR)
  #undef USAGE_DIFF_CORR
#endif
#if defined(_WIN32) && defined(USAGE_INTEGRITY)
  #undef USAGE_INTEGRITY
#endif

  enum {
    CLASS_ID = 6u,
    MESSAGE_ID = 22u,
    MODE_ENABLED = 1u,
    MODE_TEST = 2u,
    USAGE_RANGE = 1u,
    USAGE_DIFF_CORR = 2u,
    USAGE_INTEGRITY = 4u,
  };


  typedef boost::shared_ptr< ::ublox_msgs::CfgSBAS_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ublox_msgs::CfgSBAS_<ContainerAllocator> const> ConstPtr;

}; // struct CfgSBAS_

typedef ::ublox_msgs::CfgSBAS_<std::allocator<void> > CfgSBAS;

typedef boost::shared_ptr< ::ublox_msgs::CfgSBAS > CfgSBASPtr;
typedef boost::shared_ptr< ::ublox_msgs::CfgSBAS const> CfgSBASConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ublox_msgs::CfgSBAS_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ublox_msgs::CfgSBAS_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ublox_msgs::CfgSBAS_<ContainerAllocator1> & lhs, const ::ublox_msgs::CfgSBAS_<ContainerAllocator2> & rhs)
{
  return lhs.mode == rhs.mode &&
    lhs.usage == rhs.usage &&
    lhs.maxSBAS == rhs.maxSBAS &&
    lhs.scanmode2 == rhs.scanmode2 &&
    lhs.scanmode1 == rhs.scanmode1;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ublox_msgs::CfgSBAS_<ContainerAllocator1> & lhs, const ::ublox_msgs::CfgSBAS_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ublox_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::CfgSBAS_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::CfgSBAS_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::CfgSBAS_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::CfgSBAS_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::CfgSBAS_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::CfgSBAS_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ublox_msgs::CfgSBAS_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b03a1b853ac45d2da104aafaa036e7e8";
  }

  static const char* value(const ::ublox_msgs::CfgSBAS_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb03a1b853ac45d2dULL;
  static const uint64_t static_value2 = 0xa104aafaa036e7e8ULL;
};

template<class ContainerAllocator>
struct DataType< ::ublox_msgs::CfgSBAS_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ublox_msgs/CfgSBAS";
  }

  static const char* value(const ::ublox_msgs::CfgSBAS_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ublox_msgs::CfgSBAS_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# CFG-SBAS (0x06 0x16)\n"
"# SBAS Configuration\n"
"#\n"
"# This message configures the SBAS receiver subsystem (i.e. WAAS, EGNOS, MSAS).\n"
"# See the SBAS Configuration Settings Description for a detailed description of\n"
"# how these settings affect receiver operation\n"
"#\n"
"\n"
"uint8 CLASS_ID = 6\n"
"uint8 MESSAGE_ID = 22\n"
"\n"
"uint8 mode              # SBAS Mode\n"
"uint8 MODE_ENABLED = 1    # SBAS Enabled (1) / Disabled (0)\n"
"                          #  This field is deprecated; use UBX-CFG-GNSS to \n"
"                          # enable/disable SBAS operation\n"
"uint8 MODE_TEST = 2       # SBAS Testbed: Use data anyhow (1) / Ignore data when \n"
"                          # in Test Mode (SBAS Msg 0)\n"
"\n"
"uint8 usage             # SBAS Usage\n"
"uint8 USAGE_RANGE = 1     # Use SBAS GEOs as a ranging source (for navigation)\n"
"uint8 USAGE_DIFF_CORR = 2 # Use SBAS Differential Corrections\n"
"uint8 USAGE_INTEGRITY = 4 # Use SBAS Integrity Information\n"
"\n"
"uint8 maxSBAS           # Maximum Number of SBAS prioritized tracking\n"
"                        # channels (valid range: 0 - 3) to use\n"
"                        # (obsolete and superseeded by UBX-CFG-GNSS in protocol\n"
"                        # versions 14+).\n"
"\n"
"\n"
"uint8 scanmode2         # Continuation of scanmode bitmask below\n"
"                        # PRN 152...158\n"
"uint32 scanmode1        # Which SBAS PRN numbers to search for (Bitmask)\n"
"                        # If all Bits are set to zero, auto-scan (i.e. all valid\n"
"                        # PRNs) are searched. Every bit corresponds to a PRN \n"
"                        # number.\n"
"                        # PRN 120..151\n"
;
  }

  static const char* value(const ::ublox_msgs::CfgSBAS_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ublox_msgs::CfgSBAS_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mode);
      stream.next(m.usage);
      stream.next(m.maxSBAS);
      stream.next(m.scanmode2);
      stream.next(m.scanmode1);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CfgSBAS_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ublox_msgs::CfgSBAS_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ublox_msgs::CfgSBAS_<ContainerAllocator>& v)
  {
    s << indent << "mode: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.mode);
    s << indent << "usage: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.usage);
    s << indent << "maxSBAS: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.maxSBAS);
    s << indent << "scanmode2: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.scanmode2);
    s << indent << "scanmode1: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.scanmode1);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UBLOX_MSGS_MESSAGE_CFGSBAS_H
