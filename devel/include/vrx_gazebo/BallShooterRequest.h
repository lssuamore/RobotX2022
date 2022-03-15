// Generated by gencpp from file vrx_gazebo/BallShooterRequest.msg
// DO NOT EDIT!


#ifndef VRX_GAZEBO_MESSAGE_BALLSHOOTERREQUEST_H
#define VRX_GAZEBO_MESSAGE_BALLSHOOTERREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace vrx_gazebo
{
template <class ContainerAllocator>
struct BallShooterRequest_
{
  typedef BallShooterRequest_<ContainerAllocator> Type;

  BallShooterRequest_()
    : shoot(false)  {
    }
  BallShooterRequest_(const ContainerAllocator& _alloc)
    : shoot(false)  {
  (void)_alloc;
    }



   typedef uint8_t _shoot_type;
  _shoot_type shoot;





  typedef boost::shared_ptr< ::vrx_gazebo::BallShooterRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vrx_gazebo::BallShooterRequest_<ContainerAllocator> const> ConstPtr;

}; // struct BallShooterRequest_

typedef ::vrx_gazebo::BallShooterRequest_<std::allocator<void> > BallShooterRequest;

typedef boost::shared_ptr< ::vrx_gazebo::BallShooterRequest > BallShooterRequestPtr;
typedef boost::shared_ptr< ::vrx_gazebo::BallShooterRequest const> BallShooterRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vrx_gazebo::BallShooterRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vrx_gazebo::BallShooterRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::vrx_gazebo::BallShooterRequest_<ContainerAllocator1> & lhs, const ::vrx_gazebo::BallShooterRequest_<ContainerAllocator2> & rhs)
{
  return lhs.shoot == rhs.shoot;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::vrx_gazebo::BallShooterRequest_<ContainerAllocator1> & lhs, const ::vrx_gazebo::BallShooterRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace vrx_gazebo

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::vrx_gazebo::BallShooterRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vrx_gazebo::BallShooterRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vrx_gazebo::BallShooterRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vrx_gazebo::BallShooterRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vrx_gazebo::BallShooterRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vrx_gazebo::BallShooterRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vrx_gazebo::BallShooterRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6650cf67c2d9602e920f52a11329e98f";
  }

  static const char* value(const ::vrx_gazebo::BallShooterRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6650cf67c2d9602eULL;
  static const uint64_t static_value2 = 0x920f52a11329e98fULL;
};

template<class ContainerAllocator>
struct DataType< ::vrx_gazebo::BallShooterRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vrx_gazebo/BallShooterRequest";
  }

  static const char* value(const ::vrx_gazebo::BallShooterRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vrx_gazebo::BallShooterRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Ball shooter service\n"
"\n"
"bool shoot\n"
;
  }

  static const char* value(const ::vrx_gazebo::BallShooterRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vrx_gazebo::BallShooterRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.shoot);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BallShooterRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vrx_gazebo::BallShooterRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vrx_gazebo::BallShooterRequest_<ContainerAllocator>& v)
  {
    s << indent << "shoot: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.shoot);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VRX_GAZEBO_MESSAGE_BALLSHOOTERREQUEST_H
