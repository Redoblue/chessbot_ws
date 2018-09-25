// Generated by gencpp from file chessbot_control/RG2Request.msg
// DO NOT EDIT!


#ifndef CHESSBOT_CONTROL_MESSAGE_RG2REQUEST_H
#define CHESSBOT_CONTROL_MESSAGE_RG2REQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Float64.h>

namespace chessbot_control
{
template <class ContainerAllocator>
struct RG2Request_
{
  typedef RG2Request_<ContainerAllocator> Type;

  RG2Request_()
    : target_width()  {
    }
  RG2Request_(const ContainerAllocator& _alloc)
    : target_width(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Float64_<ContainerAllocator>  _target_width_type;
  _target_width_type target_width;





  typedef boost::shared_ptr< ::chessbot_control::RG2Request_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::chessbot_control::RG2Request_<ContainerAllocator> const> ConstPtr;

}; // struct RG2Request_

typedef ::chessbot_control::RG2Request_<std::allocator<void> > RG2Request;

typedef boost::shared_ptr< ::chessbot_control::RG2Request > RG2RequestPtr;
typedef boost::shared_ptr< ::chessbot_control::RG2Request const> RG2RequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::chessbot_control::RG2Request_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::chessbot_control::RG2Request_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace chessbot_control

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::chessbot_control::RG2Request_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::chessbot_control::RG2Request_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::chessbot_control::RG2Request_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::chessbot_control::RG2Request_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::chessbot_control::RG2Request_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::chessbot_control::RG2Request_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::chessbot_control::RG2Request_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0c47b5151a9d2cd1addabbbc4d278908";
  }

  static const char* value(const ::chessbot_control::RG2Request_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0c47b5151a9d2cd1ULL;
  static const uint64_t static_value2 = 0xaddabbbc4d278908ULL;
};

template<class ContainerAllocator>
struct DataType< ::chessbot_control::RG2Request_<ContainerAllocator> >
{
  static const char* value()
  {
    return "chessbot_control/RG2Request";
  }

  static const char* value(const ::chessbot_control::RG2Request_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::chessbot_control::RG2Request_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
std_msgs/Float64 target_width\n\
\n\
================================================================================\n\
MSG: std_msgs/Float64\n\
float64 data\n\
";
  }

  static const char* value(const ::chessbot_control::RG2Request_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::chessbot_control::RG2Request_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.target_width);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RG2Request_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::chessbot_control::RG2Request_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::chessbot_control::RG2Request_<ContainerAllocator>& v)
  {
    s << indent << "target_width: ";
    s << std::endl;
    Printer< ::std_msgs::Float64_<ContainerAllocator> >::stream(s, indent + "  ", v.target_width);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CHESSBOT_CONTROL_MESSAGE_RG2REQUEST_H