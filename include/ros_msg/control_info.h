// Generated by gencpp from file ros_msg/control_info.msg
// DO NOT EDIT!


#ifndef ros_msg_MESSAGE_CONTROL_INFO_H
#define ros_msg_MESSAGE_CONTROL_INFO_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Bool.h>

namespace ros_msg
{
template <class ContainerAllocator>
struct control_info_
{
  typedef control_info_<ContainerAllocator> Type;

  control_info_()
    : type()  {
    }
  control_info_(const ContainerAllocator& _alloc)
    : type(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Bool_<ContainerAllocator>  _type_type;
  _type_type type;





  typedef boost::shared_ptr< ::ros_msg::control_info_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ros_msg::control_info_<ContainerAllocator> const> ConstPtr;

}; // struct control_info_

typedef ::ros_msg::control_info_<std::allocator<void> > control_info;

typedef boost::shared_ptr< ::ros_msg::control_info > control_infoPtr;
typedef boost::shared_ptr< ::ros_msg::control_info const> control_infoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ros_msg::control_info_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ros_msg::control_info_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ros_msg::control_info_<ContainerAllocator1> & lhs, const ::ros_msg::control_info_<ContainerAllocator2> & rhs)
{
  return lhs.type == rhs.type;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ros_msg::control_info_<ContainerAllocator1> & lhs, const ::ros_msg::control_info_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ros_msg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::ros_msg::control_info_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ros_msg::control_info_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ros_msg::control_info_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ros_msg::control_info_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_msg::control_info_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_msg::control_info_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ros_msg::control_info_<ContainerAllocator> >
{
  static const char* value()
  {
    return "09499361eedd5d1fda1dc56db26b0f09";
  }

  static const char* value(const ::ros_msg::control_info_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x09499361eedd5d1fULL;
  static const uint64_t static_value2 = 0xda1dc56db26b0f09ULL;
};

template<class ContainerAllocator>
struct DataType< ::ros_msg::control_info_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ros_msg/control_info";
  }

  static const char* value(const ::ros_msg::control_info_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ros_msg::control_info_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Bool type\n"
"================================================================================\n"
"MSG: std_msgs/Bool\n"
"bool data\n"
;
  }

  static const char* value(const ::ros_msg::control_info_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ros_msg::control_info_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.type);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct control_info_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ros_msg::control_info_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ros_msg::control_info_<ContainerAllocator>& v)
  {
    s << indent << "type: ";
    s << std::endl;
    Printer< ::std_msgs::Bool_<ContainerAllocator> >::stream(s, indent + "  ", v.type);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ros_msg_MESSAGE_CONTROL_INFO_H