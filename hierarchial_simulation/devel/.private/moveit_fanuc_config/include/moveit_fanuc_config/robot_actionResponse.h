// Generated by gencpp from file moveit_fanuc_config/robot_actionResponse.msg
// DO NOT EDIT!


#ifndef MOVEIT_FANUC_CONFIG_MESSAGE_ROBOT_ACTIONRESPONSE_H
#define MOVEIT_FANUC_CONFIG_MESSAGE_ROBOT_ACTIONRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace moveit_fanuc_config
{
template <class ContainerAllocator>
struct robot_actionResponse_
{
  typedef robot_actionResponse_<ContainerAllocator> Type;

  robot_actionResponse_()
    : finished(false)  {
    }
  robot_actionResponse_(const ContainerAllocator& _alloc)
    : finished(false)  {
  (void)_alloc;
    }



   typedef uint8_t _finished_type;
  _finished_type finished;





  typedef boost::shared_ptr< ::moveit_fanuc_config::robot_actionResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::moveit_fanuc_config::robot_actionResponse_<ContainerAllocator> const> ConstPtr;

}; // struct robot_actionResponse_

typedef ::moveit_fanuc_config::robot_actionResponse_<std::allocator<void> > robot_actionResponse;

typedef boost::shared_ptr< ::moveit_fanuc_config::robot_actionResponse > robot_actionResponsePtr;
typedef boost::shared_ptr< ::moveit_fanuc_config::robot_actionResponse const> robot_actionResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::moveit_fanuc_config::robot_actionResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::moveit_fanuc_config::robot_actionResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::moveit_fanuc_config::robot_actionResponse_<ContainerAllocator1> & lhs, const ::moveit_fanuc_config::robot_actionResponse_<ContainerAllocator2> & rhs)
{
  return lhs.finished == rhs.finished;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::moveit_fanuc_config::robot_actionResponse_<ContainerAllocator1> & lhs, const ::moveit_fanuc_config::robot_actionResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace moveit_fanuc_config

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::moveit_fanuc_config::robot_actionResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::moveit_fanuc_config::robot_actionResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::moveit_fanuc_config::robot_actionResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::moveit_fanuc_config::robot_actionResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::moveit_fanuc_config::robot_actionResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::moveit_fanuc_config::robot_actionResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::moveit_fanuc_config::robot_actionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e2797c797e620a950ba704492d72873b";
  }

  static const char* value(const ::moveit_fanuc_config::robot_actionResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe2797c797e620a95ULL;
  static const uint64_t static_value2 = 0x0ba704492d72873bULL;
};

template<class ContainerAllocator>
struct DataType< ::moveit_fanuc_config::robot_actionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "moveit_fanuc_config/robot_actionResponse";
  }

  static const char* value(const ::moveit_fanuc_config::robot_actionResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::moveit_fanuc_config::robot_actionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool finished\n"
;
  }

  static const char* value(const ::moveit_fanuc_config::robot_actionResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::moveit_fanuc_config::robot_actionResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.finished);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct robot_actionResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::moveit_fanuc_config::robot_actionResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::moveit_fanuc_config::robot_actionResponse_<ContainerAllocator>& v)
  {
    s << indent << "finished: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.finished);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOVEIT_FANUC_CONFIG_MESSAGE_ROBOT_ACTIONRESPONSE_H
