// Generated by gencpp from file fiducial_msgs/FiducialMapEntryArray.msg
// DO NOT EDIT!


#ifndef FIDUCIAL_MSGS_MESSAGE_FIDUCIALMAPENTRYARRAY_H
#define FIDUCIAL_MSGS_MESSAGE_FIDUCIALMAPENTRYARRAY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <fiducial_msgs/FiducialMapEntry.h>

namespace fiducial_msgs
{
template <class ContainerAllocator>
struct FiducialMapEntryArray_
{
  typedef FiducialMapEntryArray_<ContainerAllocator> Type;

  FiducialMapEntryArray_()
    : fiducials()  {
    }
  FiducialMapEntryArray_(const ContainerAllocator& _alloc)
    : fiducials(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::fiducial_msgs::FiducialMapEntry_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::fiducial_msgs::FiducialMapEntry_<ContainerAllocator> >::other >  _fiducials_type;
  _fiducials_type fiducials;





  typedef boost::shared_ptr< ::fiducial_msgs::FiducialMapEntryArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fiducial_msgs::FiducialMapEntryArray_<ContainerAllocator> const> ConstPtr;

}; // struct FiducialMapEntryArray_

typedef ::fiducial_msgs::FiducialMapEntryArray_<std::allocator<void> > FiducialMapEntryArray;

typedef boost::shared_ptr< ::fiducial_msgs::FiducialMapEntryArray > FiducialMapEntryArrayPtr;
typedef boost::shared_ptr< ::fiducial_msgs::FiducialMapEntryArray const> FiducialMapEntryArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::fiducial_msgs::FiducialMapEntryArray_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::fiducial_msgs::FiducialMapEntryArray_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::fiducial_msgs::FiducialMapEntryArray_<ContainerAllocator1> & lhs, const ::fiducial_msgs::FiducialMapEntryArray_<ContainerAllocator2> & rhs)
{
  return lhs.fiducials == rhs.fiducials;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::fiducial_msgs::FiducialMapEntryArray_<ContainerAllocator1> & lhs, const ::fiducial_msgs::FiducialMapEntryArray_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace fiducial_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::fiducial_msgs::FiducialMapEntryArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::fiducial_msgs::FiducialMapEntryArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::fiducial_msgs::FiducialMapEntryArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::fiducial_msgs::FiducialMapEntryArray_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fiducial_msgs::FiducialMapEntryArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fiducial_msgs::FiducialMapEntryArray_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::fiducial_msgs::FiducialMapEntryArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f3d7e1cb2717bda61be54acdb77f4f79";
  }

  static const char* value(const ::fiducial_msgs::FiducialMapEntryArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf3d7e1cb2717bda6ULL;
  static const uint64_t static_value2 = 0x1be54acdb77f4f79ULL;
};

template<class ContainerAllocator>
struct DataType< ::fiducial_msgs::FiducialMapEntryArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fiducial_msgs/FiducialMapEntryArray";
  }

  static const char* value(const ::fiducial_msgs::FiducialMapEntryArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::fiducial_msgs::FiducialMapEntryArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "FiducialMapEntry[] fiducials\n"
"\n"
"================================================================================\n"
"MSG: fiducial_msgs/FiducialMapEntry\n"
"# pose of a fiducial\n"
"int32 fiducial_id\n"
"# translation\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"# rotation\n"
"float64 rx\n"
"float64 ry\n"
"float64 rz\n"
"\n"
;
  }

  static const char* value(const ::fiducial_msgs::FiducialMapEntryArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::fiducial_msgs::FiducialMapEntryArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.fiducials);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FiducialMapEntryArray_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::fiducial_msgs::FiducialMapEntryArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::fiducial_msgs::FiducialMapEntryArray_<ContainerAllocator>& v)
  {
    s << indent << "fiducials[]" << std::endl;
    for (size_t i = 0; i < v.fiducials.size(); ++i)
    {
      s << indent << "  fiducials[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::fiducial_msgs::FiducialMapEntry_<ContainerAllocator> >::stream(s, indent + "    ", v.fiducials[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // FIDUCIAL_MSGS_MESSAGE_FIDUCIALMAPENTRYARRAY_H
