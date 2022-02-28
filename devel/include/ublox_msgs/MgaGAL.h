// Generated by gencpp from file ublox_msgs/MgaGAL.msg
// DO NOT EDIT!


#ifndef UBLOX_MSGS_MESSAGE_MGAGAL_H
#define UBLOX_MSGS_MESSAGE_MGAGAL_H


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
struct MgaGAL_
{
  typedef MgaGAL_<ContainerAllocator> Type;

  MgaGAL_()
    : type(0)
    , version(0)
    , svid(0)
    , reserved0(0)
    , iodNav(0)
    , deltaN(0)
    , m0(0)
    , e(0)
    , sqrtA(0)
    , omega0(0)
    , i0(0)
    , omega(0)
    , omegaDot(0)
    , iDot(0)
    , cuc(0)
    , cus(0)
    , crc(0)
    , crs(0)
    , cic(0)
    , cis(0)
    , toe(0)
    , af0(0)
    , af1(0)
    , af2(0)
    , sisaindexE1E5b(0)
    , toc(0)
    , bgdE1E5b(0)
    , reserved1()
    , healthE1B(0)
    , dataValidityE1B(0)
    , healthE5b(0)
    , dataValidityE5b(0)
    , reserved2()  {
      reserved1.assign(0);

      reserved2.assign(0);
  }
  MgaGAL_(const ContainerAllocator& _alloc)
    : type(0)
    , version(0)
    , svid(0)
    , reserved0(0)
    , iodNav(0)
    , deltaN(0)
    , m0(0)
    , e(0)
    , sqrtA(0)
    , omega0(0)
    , i0(0)
    , omega(0)
    , omegaDot(0)
    , iDot(0)
    , cuc(0)
    , cus(0)
    , crc(0)
    , crs(0)
    , cic(0)
    , cis(0)
    , toe(0)
    , af0(0)
    , af1(0)
    , af2(0)
    , sisaindexE1E5b(0)
    , toc(0)
    , bgdE1E5b(0)
    , reserved1()
    , healthE1B(0)
    , dataValidityE1B(0)
    , healthE5b(0)
    , dataValidityE5b(0)
    , reserved2()  {
  (void)_alloc;
      reserved1.assign(0);

      reserved2.assign(0);
  }



   typedef uint8_t _type_type;
  _type_type type;

   typedef uint8_t _version_type;
  _version_type version;

   typedef uint8_t _svid_type;
  _svid_type svid;

   typedef uint8_t _reserved0_type;
  _reserved0_type reserved0;

   typedef uint16_t _iodNav_type;
  _iodNav_type iodNav;

   typedef int16_t _deltaN_type;
  _deltaN_type deltaN;

   typedef int32_t _m0_type;
  _m0_type m0;

   typedef uint32_t _e_type;
  _e_type e;

   typedef uint32_t _sqrtA_type;
  _sqrtA_type sqrtA;

   typedef int32_t _omega0_type;
  _omega0_type omega0;

   typedef int32_t _i0_type;
  _i0_type i0;

   typedef int32_t _omega_type;
  _omega_type omega;

   typedef int32_t _omegaDot_type;
  _omegaDot_type omegaDot;

   typedef int16_t _iDot_type;
  _iDot_type iDot;

   typedef int16_t _cuc_type;
  _cuc_type cuc;

   typedef int16_t _cus_type;
  _cus_type cus;

   typedef int16_t _crc_type;
  _crc_type crc;

   typedef int16_t _crs_type;
  _crs_type crs;

   typedef int16_t _cic_type;
  _cic_type cic;

   typedef int16_t _cis_type;
  _cis_type cis;

   typedef uint16_t _toe_type;
  _toe_type toe;

   typedef int32_t _af0_type;
  _af0_type af0;

   typedef int32_t _af1_type;
  _af1_type af1;

   typedef int8_t _af2_type;
  _af2_type af2;

   typedef uint8_t _sisaindexE1E5b_type;
  _sisaindexE1E5b_type sisaindexE1E5b;

   typedef uint16_t _toc_type;
  _toc_type toc;

   typedef int16_t _bgdE1E5b_type;
  _bgdE1E5b_type bgdE1E5b;

   typedef boost::array<uint8_t, 2>  _reserved1_type;
  _reserved1_type reserved1;

   typedef uint8_t _healthE1B_type;
  _healthE1B_type healthE1B;

   typedef uint8_t _dataValidityE1B_type;
  _dataValidityE1B_type dataValidityE1B;

   typedef uint8_t _healthE5b_type;
  _healthE5b_type healthE5b;

   typedef uint8_t _dataValidityE5b_type;
  _dataValidityE5b_type dataValidityE5b;

   typedef boost::array<uint8_t, 4>  _reserved2_type;
  _reserved2_type reserved2;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(CLASS_ID)
  #undef CLASS_ID
#endif
#if defined(_WIN32) && defined(MESSAGE_ID)
  #undef MESSAGE_ID
#endif

  enum {
    CLASS_ID = 19u,
    MESSAGE_ID = 2u,
  };


  typedef boost::shared_ptr< ::ublox_msgs::MgaGAL_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ublox_msgs::MgaGAL_<ContainerAllocator> const> ConstPtr;

}; // struct MgaGAL_

typedef ::ublox_msgs::MgaGAL_<std::allocator<void> > MgaGAL;

typedef boost::shared_ptr< ::ublox_msgs::MgaGAL > MgaGALPtr;
typedef boost::shared_ptr< ::ublox_msgs::MgaGAL const> MgaGALConstPtr;

// constants requiring out of line definition

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ublox_msgs::MgaGAL_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ublox_msgs::MgaGAL_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ublox_msgs::MgaGAL_<ContainerAllocator1> & lhs, const ::ublox_msgs::MgaGAL_<ContainerAllocator2> & rhs)
{
  return lhs.type == rhs.type &&
    lhs.version == rhs.version &&
    lhs.svid == rhs.svid &&
    lhs.reserved0 == rhs.reserved0 &&
    lhs.iodNav == rhs.iodNav &&
    lhs.deltaN == rhs.deltaN &&
    lhs.m0 == rhs.m0 &&
    lhs.e == rhs.e &&
    lhs.sqrtA == rhs.sqrtA &&
    lhs.omega0 == rhs.omega0 &&
    lhs.i0 == rhs.i0 &&
    lhs.omega == rhs.omega &&
    lhs.omegaDot == rhs.omegaDot &&
    lhs.iDot == rhs.iDot &&
    lhs.cuc == rhs.cuc &&
    lhs.cus == rhs.cus &&
    lhs.crc == rhs.crc &&
    lhs.crs == rhs.crs &&
    lhs.cic == rhs.cic &&
    lhs.cis == rhs.cis &&
    lhs.toe == rhs.toe &&
    lhs.af0 == rhs.af0 &&
    lhs.af1 == rhs.af1 &&
    lhs.af2 == rhs.af2 &&
    lhs.sisaindexE1E5b == rhs.sisaindexE1E5b &&
    lhs.toc == rhs.toc &&
    lhs.bgdE1E5b == rhs.bgdE1E5b &&
    lhs.reserved1 == rhs.reserved1 &&
    lhs.healthE1B == rhs.healthE1B &&
    lhs.dataValidityE1B == rhs.dataValidityE1B &&
    lhs.healthE5b == rhs.healthE5b &&
    lhs.dataValidityE5b == rhs.dataValidityE5b &&
    lhs.reserved2 == rhs.reserved2;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ublox_msgs::MgaGAL_<ContainerAllocator1> & lhs, const ::ublox_msgs::MgaGAL_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ublox_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::MgaGAL_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::MgaGAL_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::MgaGAL_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::MgaGAL_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::MgaGAL_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::MgaGAL_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ublox_msgs::MgaGAL_<ContainerAllocator> >
{
  static const char* value()
  {
    return "916efe401cfebd852654e34c3cd97512";
  }

  static const char* value(const ::ublox_msgs::MgaGAL_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x916efe401cfebd85ULL;
  static const uint64_t static_value2 = 0x2654e34c3cd97512ULL;
};

template<class ContainerAllocator>
struct DataType< ::ublox_msgs::MgaGAL_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ublox_msgs/MgaGAL";
  }

  static const char* value(const ::ublox_msgs::MgaGAL_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ublox_msgs::MgaGAL_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# MGA-GAL (0x13 0x02)\n"
"# Galileo Ephemeris Assistance\n"
"#\n"
"# This message allows the delivery of Galileo ephemeris assistance to a \n"
"# receiver. See the description of AssistNow Online for details.\n"
"# \n"
"\n"
"uint8 CLASS_ID = 19\n"
"uint8 MESSAGE_ID = 2\n"
"\n"
"uint8 type              # Message type (0x01 for this type)\n"
"uint8 version           # Message version (0x00 for this version)\n"
"uint8 svid              # Galileo Satellite identifier\n"
"\n"
"uint8 reserved0         # Reserved\n"
"\n"
"uint16 iodNav           # Ephemeris and clock correction issue of Data\n"
"int16 deltaN            # Mean motion difference from computed value \n"
"                        # [semi-cir cles/s * 2^-43]\n"
"int32 m0                # Mean anomaly at reference time [semi-cir cles 2^-31]\n"
"uint32 e                # Eccentricity [2^-33]\n"
"uint32 sqrtA            # Square root of the semi-major axis [m^0.5 * 2^-19]\n"
"int32 omega0            # Longitude of ascending node of orbital plane at weekly\n"
"                        # epoch [semi-cir cles 2^-31]\n"
"int32 i0                # inclination angle at reference time \n"
"                        # [semi-cir cles 2^-31]\n"
"int32 omega             # Argument of perigee [semi-cir cles 2^-31]\n"
"int32 omegaDot          # Rate of change of right ascension \n"
"                        # [semi-cir cles/s 2^-43]\n"
"int16 iDot              # Rate of change of inclination angle \n"
"                        # [semi-cir cles/s 2^-43]\n"
"int16 cuc               # Amplitude of the cosine harmonic correction term to \n"
"                        # the argument of latitude [radians * 2^-29]\n"
"int16 cus               # Amplitude of the sine harmonic correction term to \n"
"                        # the argument of latitude [radians * 2^-29]\n"
"int16 crc               # Amplitude of the cosine harmonic correction term \n"
"                        # to the orbit radius [radians * 2^-5]\n"
"int16 crs               # Amplitude of the sine harmonic correction term to the \n"
"                        # orbit radius [radians * 2^-5]\n"
"int16 cic               # Amplitude of the cosine harmonic correction term to \n"
"                        # the angle of inclination [radians * 2^-29]\n"
"int16 cis               # Amplitude of the sine harmonic correction term to the \n"
"                        # angle of inclination [radians * 2^-29]\n"
"uint16 toe              # Ephemeris reference time [60 * s]\n"
"int32 af0               # clock bias correction coefficient [s * 2^-34]\n"
"int32 af1               # SV clock drift correction coefficient [s/s * 2^-46]\n"
"int8 af2               # SV clock drift rate correction coefficient \n"
"                        # [s/s^2 * 2^-59]\n"
"uint8 sisaindexE1E5b   # Signal-in-Space Accuracy index for dual frequency \n"
"                        # E1-E5b\n"
"uint16 toc              # Clock correction data reference Time of Week [60 * s]\n"
"int16 bgdE1E5b          # E1-E5b Broadcast Group Delay\n"
"\n"
"uint8[2] reserved1     # Reserved\n"
"\n"
"uint8 healthE1B        # E1-B Signal Health Status\n"
"uint8 dataValidityE1B  # E1-B Data Validity Status\n"
"uint8 healthE5b        # E5b Signal Health Status\n"
"uint8 dataValidityE5b  # E5b Data Validity Status\n"
"\n"
"uint8[4] reserved2     # Reserved\n"
;
  }

  static const char* value(const ::ublox_msgs::MgaGAL_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ublox_msgs::MgaGAL_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.type);
      stream.next(m.version);
      stream.next(m.svid);
      stream.next(m.reserved0);
      stream.next(m.iodNav);
      stream.next(m.deltaN);
      stream.next(m.m0);
      stream.next(m.e);
      stream.next(m.sqrtA);
      stream.next(m.omega0);
      stream.next(m.i0);
      stream.next(m.omega);
      stream.next(m.omegaDot);
      stream.next(m.iDot);
      stream.next(m.cuc);
      stream.next(m.cus);
      stream.next(m.crc);
      stream.next(m.crs);
      stream.next(m.cic);
      stream.next(m.cis);
      stream.next(m.toe);
      stream.next(m.af0);
      stream.next(m.af1);
      stream.next(m.af2);
      stream.next(m.sisaindexE1E5b);
      stream.next(m.toc);
      stream.next(m.bgdE1E5b);
      stream.next(m.reserved1);
      stream.next(m.healthE1B);
      stream.next(m.dataValidityE1B);
      stream.next(m.healthE5b);
      stream.next(m.dataValidityE5b);
      stream.next(m.reserved2);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MgaGAL_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ublox_msgs::MgaGAL_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ublox_msgs::MgaGAL_<ContainerAllocator>& v)
  {
    s << indent << "type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.type);
    s << indent << "version: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.version);
    s << indent << "svid: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.svid);
    s << indent << "reserved0: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.reserved0);
    s << indent << "iodNav: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.iodNav);
    s << indent << "deltaN: ";
    Printer<int16_t>::stream(s, indent + "  ", v.deltaN);
    s << indent << "m0: ";
    Printer<int32_t>::stream(s, indent + "  ", v.m0);
    s << indent << "e: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.e);
    s << indent << "sqrtA: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.sqrtA);
    s << indent << "omega0: ";
    Printer<int32_t>::stream(s, indent + "  ", v.omega0);
    s << indent << "i0: ";
    Printer<int32_t>::stream(s, indent + "  ", v.i0);
    s << indent << "omega: ";
    Printer<int32_t>::stream(s, indent + "  ", v.omega);
    s << indent << "omegaDot: ";
    Printer<int32_t>::stream(s, indent + "  ", v.omegaDot);
    s << indent << "iDot: ";
    Printer<int16_t>::stream(s, indent + "  ", v.iDot);
    s << indent << "cuc: ";
    Printer<int16_t>::stream(s, indent + "  ", v.cuc);
    s << indent << "cus: ";
    Printer<int16_t>::stream(s, indent + "  ", v.cus);
    s << indent << "crc: ";
    Printer<int16_t>::stream(s, indent + "  ", v.crc);
    s << indent << "crs: ";
    Printer<int16_t>::stream(s, indent + "  ", v.crs);
    s << indent << "cic: ";
    Printer<int16_t>::stream(s, indent + "  ", v.cic);
    s << indent << "cis: ";
    Printer<int16_t>::stream(s, indent + "  ", v.cis);
    s << indent << "toe: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.toe);
    s << indent << "af0: ";
    Printer<int32_t>::stream(s, indent + "  ", v.af0);
    s << indent << "af1: ";
    Printer<int32_t>::stream(s, indent + "  ", v.af1);
    s << indent << "af2: ";
    Printer<int8_t>::stream(s, indent + "  ", v.af2);
    s << indent << "sisaindexE1E5b: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.sisaindexE1E5b);
    s << indent << "toc: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.toc);
    s << indent << "bgdE1E5b: ";
    Printer<int16_t>::stream(s, indent + "  ", v.bgdE1E5b);
    s << indent << "reserved1[]" << std::endl;
    for (size_t i = 0; i < v.reserved1.size(); ++i)
    {
      s << indent << "  reserved1[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.reserved1[i]);
    }
    s << indent << "healthE1B: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.healthE1B);
    s << indent << "dataValidityE1B: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.dataValidityE1B);
    s << indent << "healthE5b: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.healthE5b);
    s << indent << "dataValidityE5b: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.dataValidityE5b);
    s << indent << "reserved2[]" << std::endl;
    for (size_t i = 0; i < v.reserved2.size(); ++i)
    {
      s << indent << "  reserved2[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.reserved2[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // UBLOX_MSGS_MESSAGE_MGAGAL_H
