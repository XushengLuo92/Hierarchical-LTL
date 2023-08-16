// Generated by gencpp from file hierarchial_simulation/lego_pickupResponse.msg
// DO NOT EDIT!


#ifndef HIERARCHIAL_SIMULATION_MESSAGE_LEGO_PICKUPRESPONSE_H
#define HIERARCHIAL_SIMULATION_MESSAGE_LEGO_PICKUPRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace hierarchial_simulation
{
template <class ContainerAllocator>
struct lego_pickupResponse_
{
  typedef lego_pickupResponse_<ContainerAllocator> Type;

  lego_pickupResponse_()
    : finished(false)  {
    }
  lego_pickupResponse_(const ContainerAllocator& _alloc)
    : finished(false)  {
  (void)_alloc;
    }



   typedef uint8_t _finished_type;
  _finished_type finished;





  typedef boost::shared_ptr< ::hierarchial_simulation::lego_pickupResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hierarchial_simulation::lego_pickupResponse_<ContainerAllocator> const> ConstPtr;

}; // struct lego_pickupResponse_

typedef ::hierarchial_simulation::lego_pickupResponse_<std::allocator<void> > lego_pickupResponse;

typedef boost::shared_ptr< ::hierarchial_simulation::lego_pickupResponse > lego_pickupResponsePtr;
typedef boost::shared_ptr< ::hierarchial_simulation::lego_pickupResponse const> lego_pickupResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hierarchial_simulation::lego_pickupResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hierarchial_simulation::lego_pickupResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hierarchial_simulation::lego_pickupResponse_<ContainerAllocator1> & lhs, const ::hierarchial_simulation::lego_pickupResponse_<ContainerAllocator2> & rhs)
{
  return lhs.finished == rhs.finished;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hierarchial_simulation::lego_pickupResponse_<ContainerAllocator1> & lhs, const ::hierarchial_simulation::lego_pickupResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hierarchial_simulation

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hierarchial_simulation::lego_pickupResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hierarchial_simulation::lego_pickupResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hierarchial_simulation::lego_pickupResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hierarchial_simulation::lego_pickupResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hierarchial_simulation::lego_pickupResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hierarchial_simulation::lego_pickupResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hierarchial_simulation::lego_pickupResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e2797c797e620a950ba704492d72873b";
  }

  static const char* value(const ::hierarchial_simulation::lego_pickupResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe2797c797e620a95ULL;
  static const uint64_t static_value2 = 0x0ba704492d72873bULL;
};

template<class ContainerAllocator>
struct DataType< ::hierarchial_simulation::lego_pickupResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hierarchial_simulation/lego_pickupResponse";
  }

  static const char* value(const ::hierarchial_simulation::lego_pickupResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hierarchial_simulation::lego_pickupResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool finished\n"
;
  }

  static const char* value(const ::hierarchial_simulation::lego_pickupResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hierarchial_simulation::lego_pickupResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.finished);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct lego_pickupResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hierarchial_simulation::lego_pickupResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hierarchial_simulation::lego_pickupResponse_<ContainerAllocator>& v)
  {
    s << indent << "finished: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.finished);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HIERARCHIAL_SIMULATION_MESSAGE_LEGO_PICKUPRESPONSE_H
