// Generated by gencpp from file ublox_msgs/NavSAT_SV.msg
// DO NOT EDIT!


#ifndef UBLOX_MSGS_MESSAGE_NAVSAT_SV_H
#define UBLOX_MSGS_MESSAGE_NAVSAT_SV_H


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
struct NavSAT_SV_
{
  typedef NavSAT_SV_<ContainerAllocator> Type;

  NavSAT_SV_()
    : gnssId(0)
    , svId(0)
    , cno(0)
    , elev(0)
    , azim(0)
    , prRes(0)
    , flags(0)  {
    }
  NavSAT_SV_(const ContainerAllocator& _alloc)
    : gnssId(0)
    , svId(0)
    , cno(0)
    , elev(0)
    , azim(0)
    , prRes(0)
    , flags(0)  {
  (void)_alloc;
    }



   typedef uint8_t _gnssId_type;
  _gnssId_type gnssId;

   typedef uint8_t _svId_type;
  _svId_type svId;

   typedef uint8_t _cno_type;
  _cno_type cno;

   typedef int8_t _elev_type;
  _elev_type elev;

   typedef int16_t _azim_type;
  _azim_type azim;

   typedef int16_t _prRes_type;
  _prRes_type prRes;

   typedef uint32_t _flags_type;
  _flags_type flags;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(FLAGS_QUALITY_IND_MASK)
  #undef FLAGS_QUALITY_IND_MASK
#endif
#if defined(_WIN32) && defined(QUALITY_IND_NO_SIGNAL)
  #undef QUALITY_IND_NO_SIGNAL
#endif
#if defined(_WIN32) && defined(QUALITY_IND_SEARCHING_SIGNAL)
  #undef QUALITY_IND_SEARCHING_SIGNAL
#endif
#if defined(_WIN32) && defined(QUALITY_IND_SIGNAL_ACQUIRED)
  #undef QUALITY_IND_SIGNAL_ACQUIRED
#endif
#if defined(_WIN32) && defined(QUALITY_IND_SIGNAL_DETECTED_BUT_UNUSABLE)
  #undef QUALITY_IND_SIGNAL_DETECTED_BUT_UNUSABLE
#endif
#if defined(_WIN32) && defined(QUALITY_IND_CODE_LOCKED_AND_TIME_SYNC)
  #undef QUALITY_IND_CODE_LOCKED_AND_TIME_SYNC
#endif
#if defined(_WIN32) && defined(QUALITY_IND_CODE_AND_CARR_LOCK_AND_TIME_SYNC1)
  #undef QUALITY_IND_CODE_AND_CARR_LOCK_AND_TIME_SYNC1
#endif
#if defined(_WIN32) && defined(QUALITY_IND_CODE_AND_CARR_LOCK_AND_TIME_SYNC2)
  #undef QUALITY_IND_CODE_AND_CARR_LOCK_AND_TIME_SYNC2
#endif
#if defined(_WIN32) && defined(QUALITY_IND_CODE_AND_CARR_LOCK_AND_TIME_SYNC3)
  #undef QUALITY_IND_CODE_AND_CARR_LOCK_AND_TIME_SYNC3
#endif
#if defined(_WIN32) && defined(FLAGS_SV_USED)
  #undef FLAGS_SV_USED
#endif
#if defined(_WIN32) && defined(FLAGS_HEALTH_MASK)
  #undef FLAGS_HEALTH_MASK
#endif
#if defined(_WIN32) && defined(HEALTH_UNKNOWN)
  #undef HEALTH_UNKNOWN
#endif
#if defined(_WIN32) && defined(HEALTH_HEALTHY)
  #undef HEALTH_HEALTHY
#endif
#if defined(_WIN32) && defined(HEALTH_UNHEALTHY)
  #undef HEALTH_UNHEALTHY
#endif
#if defined(_WIN32) && defined(FLAGS_DIFF_CORR)
  #undef FLAGS_DIFF_CORR
#endif
#if defined(_WIN32) && defined(FLAGS_SMOOTHED)
  #undef FLAGS_SMOOTHED
#endif
#if defined(_WIN32) && defined(FLAGS_ORBIT_SOURCE_MASK)
  #undef FLAGS_ORBIT_SOURCE_MASK
#endif
#if defined(_WIN32) && defined(ORBIT_SOURCE_UNAVAILABLE)
  #undef ORBIT_SOURCE_UNAVAILABLE
#endif
#if defined(_WIN32) && defined(ORBIT_SOURCE_EPH)
  #undef ORBIT_SOURCE_EPH
#endif
#if defined(_WIN32) && defined(ORBIT_SOURCE_ALM)
  #undef ORBIT_SOURCE_ALM
#endif
#if defined(_WIN32) && defined(ORBIT_SOURCE_ASSIST_OFFLINE)
  #undef ORBIT_SOURCE_ASSIST_OFFLINE
#endif
#if defined(_WIN32) && defined(ORBIT_SOURCE_ASSIST_AUTONOMOUS)
  #undef ORBIT_SOURCE_ASSIST_AUTONOMOUS
#endif
#if defined(_WIN32) && defined(ORBIT_SOURCE_OTHER1)
  #undef ORBIT_SOURCE_OTHER1
#endif
#if defined(_WIN32) && defined(ORBIT_SOURCE_OTHER2)
  #undef ORBIT_SOURCE_OTHER2
#endif
#if defined(_WIN32) && defined(ORBIT_SOURCE_OTHER3)
  #undef ORBIT_SOURCE_OTHER3
#endif
#if defined(_WIN32) && defined(FLAGS_EPH_AVAIL)
  #undef FLAGS_EPH_AVAIL
#endif
#if defined(_WIN32) && defined(FLAGS_ALM_AVAIL)
  #undef FLAGS_ALM_AVAIL
#endif
#if defined(_WIN32) && defined(FLAGS_ANO_AVAIL)
  #undef FLAGS_ANO_AVAIL
#endif
#if defined(_WIN32) && defined(FLAGS_AOP_AVAIL)
  #undef FLAGS_AOP_AVAIL
#endif
#if defined(_WIN32) && defined(FLAGS_SBAS_CORR_USED)
  #undef FLAGS_SBAS_CORR_USED
#endif
#if defined(_WIN32) && defined(FLAGS_RTCM_CORR_USED)
  #undef FLAGS_RTCM_CORR_USED
#endif
#if defined(_WIN32) && defined(FLAGS_PR_CORR_USED)
  #undef FLAGS_PR_CORR_USED
#endif
#if defined(_WIN32) && defined(FLAGS_CR_CORR_USED)
  #undef FLAGS_CR_CORR_USED
#endif
#if defined(_WIN32) && defined(FLAGS_DO_CORR_USED)
  #undef FLAGS_DO_CORR_USED
#endif

  enum {
    FLAGS_QUALITY_IND_MASK = 7u,
    QUALITY_IND_NO_SIGNAL = 0u,
    QUALITY_IND_SEARCHING_SIGNAL = 1u,
    QUALITY_IND_SIGNAL_ACQUIRED = 2u,
    QUALITY_IND_SIGNAL_DETECTED_BUT_UNUSABLE = 3u,
    QUALITY_IND_CODE_LOCKED_AND_TIME_SYNC = 4u,
    QUALITY_IND_CODE_AND_CARR_LOCK_AND_TIME_SYNC1 = 5u,
    QUALITY_IND_CODE_AND_CARR_LOCK_AND_TIME_SYNC2 = 6u,
    QUALITY_IND_CODE_AND_CARR_LOCK_AND_TIME_SYNC3 = 7u,
    FLAGS_SV_USED = 8u,
    FLAGS_HEALTH_MASK = 48u,
    HEALTH_UNKNOWN = 0u,
    HEALTH_HEALTHY = 1u,
    HEALTH_UNHEALTHY = 2u,
    FLAGS_DIFF_CORR = 64u,
    FLAGS_SMOOTHED = 128u,
    FLAGS_ORBIT_SOURCE_MASK = 1792u,
    ORBIT_SOURCE_UNAVAILABLE = 0u,
    ORBIT_SOURCE_EPH = 256u,
    ORBIT_SOURCE_ALM = 512u,
    ORBIT_SOURCE_ASSIST_OFFLINE = 768u,
    ORBIT_SOURCE_ASSIST_AUTONOMOUS = 1024u,
    ORBIT_SOURCE_OTHER1 = 1280u,
    ORBIT_SOURCE_OTHER2 = 1536u,
    ORBIT_SOURCE_OTHER3 = 1792u,
    FLAGS_EPH_AVAIL = 2048u,
    FLAGS_ALM_AVAIL = 4096u,
    FLAGS_ANO_AVAIL = 8192u,
    FLAGS_AOP_AVAIL = 16384u,
    FLAGS_SBAS_CORR_USED = 65536u,
    FLAGS_RTCM_CORR_USED = 131072u,
    FLAGS_PR_CORR_USED = 1048576u,
    FLAGS_CR_CORR_USED = 2097152u,
    FLAGS_DO_CORR_USED = 4194304u,
  };


  typedef boost::shared_ptr< ::ublox_msgs::NavSAT_SV_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ublox_msgs::NavSAT_SV_<ContainerAllocator> const> ConstPtr;

}; // struct NavSAT_SV_

typedef ::ublox_msgs::NavSAT_SV_<std::allocator<void> > NavSAT_SV;

typedef boost::shared_ptr< ::ublox_msgs::NavSAT_SV > NavSAT_SVPtr;
typedef boost::shared_ptr< ::ublox_msgs::NavSAT_SV const> NavSAT_SVConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ublox_msgs::NavSAT_SV_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ublox_msgs::NavSAT_SV_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ublox_msgs::NavSAT_SV_<ContainerAllocator1> & lhs, const ::ublox_msgs::NavSAT_SV_<ContainerAllocator2> & rhs)
{
  return lhs.gnssId == rhs.gnssId &&
    lhs.svId == rhs.svId &&
    lhs.cno == rhs.cno &&
    lhs.elev == rhs.elev &&
    lhs.azim == rhs.azim &&
    lhs.prRes == rhs.prRes &&
    lhs.flags == rhs.flags;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ublox_msgs::NavSAT_SV_<ContainerAllocator1> & lhs, const ::ublox_msgs::NavSAT_SV_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ublox_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::NavSAT_SV_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::NavSAT_SV_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::NavSAT_SV_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::NavSAT_SV_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::NavSAT_SV_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::NavSAT_SV_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ublox_msgs::NavSAT_SV_<ContainerAllocator> >
{
  static const char* value()
  {
    return "902ea92ca9ebf53188dcf1cdef64a9a1";
  }

  static const char* value(const ::ublox_msgs::NavSAT_SV_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x902ea92ca9ebf531ULL;
  static const uint64_t static_value2 = 0x88dcf1cdef64a9a1ULL;
};

template<class ContainerAllocator>
struct DataType< ::ublox_msgs::NavSAT_SV_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ublox_msgs/NavSAT_SV";
  }

  static const char* value(const ::ublox_msgs::NavSAT_SV_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ublox_msgs::NavSAT_SV_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# see NAV-SAT message\n"
"#\n"
"\n"
"uint8 gnssId      # GNSS identifier\n"
"uint8 svId        # Satellite identifier\n"
"\n"
"uint8 cno         # Carrier to noise ratio (signal strength) ]dBHz\n"
"int8 elev         # Elevation (range: +/-90), unknown if out of range [deg]\n"
"int16 azim        # Azimuth (range 0-360), unknown if elevation is out of range \n"
"                  # [deg]\n"
"int16 prRes       # Pseudo range residual [0.1 m]\n"
"\n"
"uint32 flags      # Bitmask\n"
"uint32 FLAGS_QUALITY_IND_MASK = 7     # Signal quality indicator:\n"
"uint8 QUALITY_IND_NO_SIGNAL = 0                     # no signal\n"
"uint8 QUALITY_IND_SEARCHING_SIGNAL = 1              # searching signal\n"
"uint8 QUALITY_IND_SIGNAL_ACQUIRED = 2               # signal acquired\n"
"uint8 QUALITY_IND_SIGNAL_DETECTED_BUT_UNUSABLE = 3  # signal detected but \n"
"                                                    # unusable\n"
"uint8 QUALITY_IND_CODE_LOCKED_AND_TIME_SYNC = 4     # code locked and time \n"
"                                                    # synchronized\n"
"uint8 QUALITY_IND_CODE_AND_CARR_LOCK_AND_TIME_SYNC1 = 5 # code and carrier \n"
"                                                        # locked and time \n"
"                                                        # synchronized, \n"
"                                                        # quality = 1\n"
"uint8 QUALITY_IND_CODE_AND_CARR_LOCK_AND_TIME_SYNC2 = 6 # code and carrier \n"
"                                                        # locked and time \n"
"                                                        # synchronized, \n"
"                                                        # quality = 2\n"
"uint8 QUALITY_IND_CODE_AND_CARR_LOCK_AND_TIME_SYNC3 = 7 # code and carrier \n"
"                                                        # locked and time \n"
"                                                        # synchronized, \n"
"                                                        # quality = 3\n"
"# Note: Since IMES signals are not time synchronized, a channel tracking an IMES \n"
"# signal can never reach a quality indicator value of higher than 3.\n"
"uint32 FLAGS_SV_USED = 8                      # whether SV is currently being \n"
"                                              # used for navigation\n"
"uint32 FLAGS_HEALTH_MASK = 48                 # SV health flag:\n"
"uint32 HEALTH_UNKNOWN = 0                       # unknown\n"
"uint32 HEALTH_HEALTHY = 1                       # healthy\n"
"uint32 HEALTH_UNHEALTHY = 2                     # unhealthy\n"
"uint32 FLAGS_DIFF_CORR = 64                   # whether differential correction \n"
"                                              # data is available for this SV\n"
"uint32 FLAGS_SMOOTHED = 128                   # whether carrier smoothed \n"
"                                              # pseudorange used\n"
"uint32 FLAGS_ORBIT_SOURCE_MASK = 1792         # Orbit source:\n"
"uint32 ORBIT_SOURCE_UNAVAILABLE = 0             # no orbit information is \n"
"                                              # available for this SV\n"
"uint32 ORBIT_SOURCE_EPH = 256                   # ephemeris is used\n"
"uint32 ORBIT_SOURCE_ALM = 512                   # almanac is used\n"
"uint32 ORBIT_SOURCE_ASSIST_OFFLINE = 768        # AssistNow Offline orbit is \n"
"                                                # used\n"
"uint32 ORBIT_SOURCE_ASSIST_AUTONOMOUS = 1024    # AssistNow Autonomous orbit is \n"
"                                                # used\n"
"uint32 ORBIT_SOURCE_OTHER1 = 1280               # other orbit information is \n"
"                                                # used\n"
"uint32 ORBIT_SOURCE_OTHER2 = 1536               # other orbit information is \n"
"                                                # used\n"
"uint32 ORBIT_SOURCE_OTHER3 = 1792               # other orbit information is \n"
"                                                # used\n"
"uint32 FLAGS_EPH_AVAIL = 2048                 # whether ephemeris is available \n"
"                                              # for this SV\n"
"uint32 FLAGS_ALM_AVAIL = 4096                 # whether almanac is available for \n"
"                                              # this SV\n"
"uint32 FLAGS_ANO_AVAIL = 8192                 # whether AssistNow Offline data \n"
"                                              # is available for this SV\n"
"uint32 FLAGS_AOP_AVAIL = 16384                # whether AssistNow Autonomous \n"
"                                              # data is available for this SV\n"
"uint32 FLAGS_SBAS_CORR_USED = 65536           # whether SBAS corrections have \n"
"                                              # been used for this SV\n"
"uint32 FLAGS_RTCM_CORR_USED = 131072          # whether RTCM corrections have \n"
"                                              # been used for this SV\n"
"uint32 FLAGS_PR_CORR_USED = 1048576           # whether Pseudorange corrections \n"
"                                              # have been used for this SV\n"
"uint32 FLAGS_CR_CORR_USED = 2097152           # whether Carrier range \n"
"                                              # corrections have been used for \n"
"                                              # this SV\n"
"uint32 FLAGS_DO_CORR_USED = 4194304           # whether Range rate (Doppler) \n"
"                                              # corrections have been used for \n"
"                                              # this SV\n"
;
  }

  static const char* value(const ::ublox_msgs::NavSAT_SV_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ublox_msgs::NavSAT_SV_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.gnssId);
      stream.next(m.svId);
      stream.next(m.cno);
      stream.next(m.elev);
      stream.next(m.azim);
      stream.next(m.prRes);
      stream.next(m.flags);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct NavSAT_SV_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ublox_msgs::NavSAT_SV_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ublox_msgs::NavSAT_SV_<ContainerAllocator>& v)
  {
    s << indent << "gnssId: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.gnssId);
    s << indent << "svId: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.svId);
    s << indent << "cno: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.cno);
    s << indent << "elev: ";
    Printer<int8_t>::stream(s, indent + "  ", v.elev);
    s << indent << "azim: ";
    Printer<int16_t>::stream(s, indent + "  ", v.azim);
    s << indent << "prRes: ";
    Printer<int16_t>::stream(s, indent + "  ", v.prRes);
    s << indent << "flags: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.flags);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UBLOX_MSGS_MESSAGE_NAVSAT_SV_H
